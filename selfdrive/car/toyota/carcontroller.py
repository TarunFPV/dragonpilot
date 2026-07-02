from cereal import car
from openpilot.common.numpy_fast import clip, interp
from openpilot.selfdrive.car import apply_meas_steer_torque_limits, apply_std_steer_angle_limits, common_fault_avoidance, \
                          create_gas_interceptor_command, make_can_msg
from openpilot.selfdrive.car.toyota import toyotacan
from openpilot.selfdrive.car.toyota.values import CAR, STATIC_DSU_MSGS, NO_STOP_TIMER_CAR, TSS2_CAR, \
                                        MIN_ACC_SPEED, PEDAL_TRANSITION, CarControllerParams, ToyotaFlags, \
                                        UNSUPPORTED_DSU_CAR
from opendbc.can.packer import CANPacker
from openpilot.common.conversions import Conversions as CV
from openpilot.common.params import Params

SteerControlType = car.CarParams.SteerControlType
VisualAlert = car.CarControl.HUDControl.VisualAlert

# LKA limits
# EPS faults if you apply torque while the steering rate is above 100 deg/s for too long
MAX_STEER_RATE = 100  # deg/s
MAX_STEER_RATE_FRAMES = 18  # tx control frames needed before torque can be cut

# EPS allows user torque above threshold for 50 frames before permanently faulting
MAX_USER_TORQUE = 500

# LTA limits
# EPS ignores commands above this angle and causes PCS to fault
MAX_LTA_ANGLE = 94.9461  # deg
MAX_LTA_DRIVER_TORQUE_ALLOWANCE = 150  # slightly above steering pressed allows some resistance when changing lanes

# rick - toyota auto lock / unlock
GearShifter = car.CarState.GearShifter
UNLOCK_CMD = b'\x40\x05\x30\x11\x00\x40\x00\x00'
LOCK_CMD = b'\x40\x05\x30\x11\x00\x80\x00\x00'
LOCK_AT_SPEED = 10 * CV.KPH_TO_MS

# Blindspot codes
LEFT_BLINDSPOT = b'\x41'
RIGHT_BLINDSPOT = b'\x42'

def set_blindspot_debug_mode(lr,enable):
  if enable:
    m = lr + b'\x02\x10\x60\x00\x00\x00\x00'
  else:
    m = lr + b'\x02\x10\x01\x00\x00\x00\x00'
  return make_can_msg(0x750, m, 0)


def poll_blindspot_status(lr):
  m = lr + b'\x02\x21\x69\x00\x00\x00\x00'
  return make_can_msg(0x750, m, 0)

class CarController:
  def __init__(self, dbc_name, CP, VM):
    self.CP = CP
    self.params = CarControllerParams(self.CP)
    self.frame = 0
    self.last_steer = 0
    self.last_angle = 0
    self.alert_active = False
    self.last_standstill = False
    self.standstill_req = False
    self.standstill_hack = False  
    self.steer_rate_counter = 0

    self.packer = CANPacker(dbc_name)
    self.gas = 0
    self.accel = 0

    self.dp_toyota_auto_lock_gear_prev = GearShifter.park
    self.dp_toyota_auto_lock_once = False
    p = Params()
    self.dp_toyota_auto_lock = p.get_bool("dp_toyota_auto_lock")
    self.dp_toyota_auto_unlock = p.get_bool("dp_toyota_auto_unlock")
    self.dp_toyota_sng = p.get_bool("dp_toyota_sng")

    # dp - bsm
    self.dp_toyota_enhanced_bsm = p.get_bool("dp_toyota_enhanced_bsm")
    self._blindspot_debug_enabled_left = False
    self._blindspot_debug_enabled_right = False
    self._blindspot_frame = 0
    if self.CP.carFingerprint in TSS2_CAR:
      self._blindspot_rate = 2
      self._blindspot_always_on = True
    else:
      self._blindspot_rate = 20
      self._blindspot_always_on = False


  def update(self, CC, CS, now_nanos):
    actuators = CC.actuators
    hud_control = CC.hudControl
    pcm_cancel_cmd = CC.cruiseControl.cancel
    lat_active = CC.latActive and abs(CS.out.steeringTorque) < MAX_USER_TORQUE

    # *** control msgs ***
    can_sends = []

    # dp - door auto lock / unlock logic
    if not CS.out.doorOpen:
      gear = CS.out.gearShifter
      if gear == GearShifter.park and self.dp_toyota_auto_lock_gear_prev != gear:
        if self.dp_toyota_auto_unlock:
          can_sends.append(make_can_msg(0x750, UNLOCK_CMD, 0))
        self.dp_toyota_auto_lock_once = False
      elif gear == GearShifter.drive and not self.dp_toyota_auto_lock_once and CS.out.vEgo >= LOCK_AT_SPEED:
        if self.dp_toyota_auto_lock:
          can_sends.append(make_can_msg(0x750, LOCK_CMD, 0))
        self.dp_toyota_auto_lock_once = True
      self.dp_toyota_auto_lock_gear_prev = gear

    # Enable blindspot debug mode once (@arne182)
    if self.dp_toyota_enhanced_bsm:
      if not self._blindspot_debug_enabled_left:
        if (self._blindspot_always_on or (CS.out.leftBlinker and CS.out.vEgo > 6)):
          can_sends.append(set_blindspot_debug_mode(LEFT_BLINDSPOT, True))
          self._blindspot_debug_enabled_left = True
      else:
        if not self._blindspot_always_on and not CS.out.leftBlinker and self.frame - self._blindspot_frame > 50:
          can_sends.append(set_blindspot_debug_mode(LEFT_BLINDSPOT, False))
          self._blindspot_debug_enabled_left = False
        if self.frame % self._blindspot_rate == 0:
          can_sends.append(poll_blindspot_status(LEFT_BLINDSPOT))
          if CS.out.leftBlinker:
            self._blindspot_frame = self.frame

      if not self._blindspot_debug_enabled_right:
        if (self._blindspot_always_on or (CS.out.rightBlinker and CS.out.vEgo > 6)):
          can_sends.append(set_blindspot_debug_mode(RIGHT_BLINDSPOT, True))
          self._blindspot_debug_enabled_right = True
      else:
        if not self._blindspot_always_on and not CS.out.rightBlinker and self.frame - self._blindspot_frame > 50:
          can_sends.append(set_blindspot_debug_mode(RIGHT_BLINDSPOT, False))
          self._blindspot_debug_enabled_right = False
        if self.frame % self._blindspot_rate == self._blindspot_rate/2:
          can_sends.append(poll_blindspot_status(RIGHT_BLINDSPOT))
          if CS.out.rightBlinker:
            self._blindspot_frame = self.frame

    # *** steer torque ***
    new_steer = int(round(actuators.steer * self.params.STEER_MAX))
    apply_steer = apply_meas_steer_torque_limits(new_steer, self.last_steer, CS.out.steeringTorqueEps, self.params)

    # >100 degree/sec steering fault prevention
    self.steer_rate_counter, apply_steer_req = common_fault_avoidance(abs(CS.out.steeringRateDeg) >= MAX_STEER_RATE, CC.latActive,
                                                                      self.steer_rate_counter, MAX_STEER_RATE_FRAMES)

    if not CC.latActive:
      apply_steer = 0

    # *** steer angle ***
    if self.CP.steerControlType == SteerControlType.angle:
      apply_steer = 0
      apply_steer_req = False
      if self.frame % 2 == 0:
        apply_angle = actuators.steeringAngleDeg + CS.out.steeringAngleOffsetDeg
        apply_angle = apply_std_steer_angle_limits(apply_angle, self.last_angle, CS.out.vEgoRaw, self.params)
        if not lat_active:
          apply_angle = CS.out.steeringAngleDeg + CS.out.steeringAngleOffsetDeg
        self.last_angle = clip(apply_angle, -MAX_LTA_ANGLE, MAX_LTA_ANGLE)

    self.last_steer = apply_steer

    can_sends.append(toyotacan.create_steer_command(self.packer, apply_steer, apply_steer_req))

    if self.frame % 2 == 0 and self.CP.carFingerprint in TSS2_CAR:
      lta_active = lat_active and self.CP.steerControlType == SteerControlType.angle
      full_torque_condition = (abs(CS.out.steeringTorqueEps) < self.params.STEER_MAX and
                               abs(CS.out.steeringTorque) < MAX_LTA_DRIVER_TORQUE_ALLOWANCE)

      torque_wind_down = 100 if lta_active and full_torque_condition else 0
      can_sends.append(toyotacan.create_lta_steer_command(self.packer, self.CP.steerControlType, self.last_angle,
                                                          lta_active, self.frame // 2, torque_wind_down))

    # *** gas and brake ***
    if self.CP.enableGasInterceptor and CC.longActive:
      MAX_INTERCEPTOR_GAS = 0.5
      if self.CP.carFingerprint in (CAR.RAV4, CAR.RAV4H, CAR.HIGHLANDER):
        PEDAL_SCALE = interp(CS.out.vEgo, [0.0, MIN_ACC_SPEED, MIN_ACC_SPEED + PEDAL_TRANSITION], [0.15, 0.3, 0.0])
      elif self.CP.carFingerprint in (CAR.COROLLA,):
        PEDAL_SCALE = interp(CS.out.vEgo, [0.0, MIN_ACC_SPEED, MIN_ACC_SPEED + PEDAL_TRANSITION], [0.3, 0.4, 0.0])
      else:
        PEDAL_SCALE = interp(CS.out.vEgo, [0.0, MIN_ACC_SPEED, MIN_ACC_SPEED + PEDAL_TRANSITION], [0.4, 0.5, 0.0])
      
      pedal_offset = interp(CS.out.vEgo, [0.0, 2.3, MIN_ACC_SPEED + PEDAL_TRANSITION], [-.4, 0.0, 0.2])
      pedal_command = PEDAL_SCALE * (actuators.accel + pedal_offset)
      interceptor_gas_cmd = clip(pedal_command, 0., MAX_INTERCEPTOR_GAS)
    else:
      interceptor_gas_cmd = 0.
    pcm_accel_cmd = clip(actuators.accel, self.params.ACCEL_MIN, self.params.ACCEL_MAX)

    if not CC.enabled and CS.pcm_acc_status:
      pcm_cancel_cmd = 1

    if CS.out.standstill and not self.last_standstill and (self.CP.carFingerprint not in NO_STOP_TIMER_CAR or self.CP.enableGasInterceptor) and not self.standstill_hack:
      self.standstill_req = True
    if CS.pcm_acc_status != 8:
      self.standstill_req = False

    self.last_standstill = CS.out.standstill

    fcw_alert = hud_control.visualAlert == VisualAlert.fcw
    steer_alert = hud_control.visualAlert in (VisualAlert.steerRequired, VisualAlert.ldw)

    if (self.frame % 3 == 0 and self.CP.openpilotLongitudinalControl) or pcm_cancel_cmd:
      lead = hud_control.leadVisible or CS.out.vEgo < 12.  

      if pcm_cancel_cmd and self.CP.carFingerprint in UNSUPPORTED_DSU_CAR:
        can_sends.append(toyotacan.create_acc_cancel_command(self.packer))
      elif self.CP.openpilotLongitudinalControl:
        # [MODIFIED]: ใส่ตัวแปรระยะห่างกลับคืนมา เพื่อส่งเข้า toyotacan ตัวใหม่ที่เราแก้ไขแล้ว
        can_sends.append(toyotacan.create_accel_command(self.packer, pcm_accel_cmd, pcm_cancel_cmd, self.standstill_req, lead, CS.acc_type, getattr(CS, 'distance_btn', 0)))
        self.accel = pcm_accel_cmd
      else:
        # [MODIFIED]: ใส่ตัวแปรระยะห่างกลับคืนมา
        can_sends.append(toyotacan.create_accel_command(self.packer, 0, pcm_cancel_cmd, False, lead, CS.acc_type, getattr(CS, 'distance_btn', 0)))

    if self.frame % 2 == 0 and self.CP.enableGasInterceptor and self.CP.openpilotLongitudinalControl:
      can_sends.append(create_gas_interceptor_command(self.packer, interceptor_gas_cmd, self.frame // 2))
      self.gas = interceptor_gas_cmd

    # *** hud ui ***
    if self.CP.carFingerprint != CAR.PRIUS_V:
      send_ui = False
      if ((fcw_alert or steer_alert) and not self.alert_active) or \
         (not (fcw_alert or steer_alert) and self.alert_active):
        send_ui = True
        self.alert_active = not self.alert_active
      elif pcm_cancel_cmd:
        send_ui = True

      if self.frame % 20 == 0 or send_ui:
        can_sends.append(toyotacan.create_ui_command(self.packer, steer_alert, pcm_cancel_cmd, hud_control.leftLaneVisible,
                                                     hud_control.rightLaneVisible, hud_control.leftLaneDepart,
                                                     hud_control.rightLaneDepart, CC.enabled, CS.lkas_hud))

      if (self.frame % 100 == 0 or send_ui) and (self.CP.enableDsu or self.CP.flags & ToyotaFlags.DISABLE_RADAR.value):
        can_sends.append(toyotacan.create_fcw_command(self.packer, fcw_alert))

    # *** static msgs ***
    for addr, cars, bus, fr_step, vl in STATIC_DSU_MSGS:
      if self.frame % fr_step == 0 and self.CP.enableDsu and self.CP.carFingerprint in cars:
        can_sends.append(make_can_msg(addr, vl, bus))

    if self.frame % 20 == 0 and self.CP.flags & ToyotaFlags.DISABLE_RADAR.value:
      can_sends.append([0x750, 0, b"\x0F\x02\x3E\x00\x00\x00\x00\x00", 0])

    new_actuators = actuators.copy()
    new_actuators.steer = apply_steer / self.params.STEER_MAX
    new_actuators.steerOutputCan = apply_steer
    new_actuators.steeringAngleDeg = self.last_angle
    new_actuators.accel = self.accel
    new_actuators.gas = self.gas

    self.frame += 1
    return new_actuators, can_sends
