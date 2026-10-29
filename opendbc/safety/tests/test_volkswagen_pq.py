#!/usr/bin/env python3
import unittest

from opendbc.car.volkswagen.values import VolkswagenSafetyFlags
from opendbc.car.structs import CarParams
import opendbc.safety.tests.common as common

MSG_HCA_1 = 0x0D2             # TX by OP, Heading Control Assist steering torque
MSG_ACC_SYSTEM = 0x368        # TX by OP, longitudinal acceleration controls
MSG_GRA_NEU = 0x38A           # TX by OP, ACC control buttons for cancel/resume
MSG_ACC_GRA_ANZEIGE = 0x56A   # TX by OP, ACC HUD
MSG_LDW_1 = 0x5BE             # TX by OP, Lane line recognition and text alerts


class TestVolkswagenPqSafetyBase(common.CarSafetyTest, common.DriverTorqueSteeringSafetyTest):
  DBC = "vw_pq"
  cruise_engaged = False

  RELAY_MALFUNCTION_ADDRS = {0: (MSG_HCA_1, MSG_LDW_1)}

  MAX_RATE_UP = 6
  MAX_RATE_DOWN = 10
  MAX_TORQUE_LOOKUP = [0], [300]
  MAX_RT_DELTA = 113

  DRIVER_TORQUE_ALLOWANCE = 80
  DRIVER_TORQUE_FACTOR = 3

  def _set_prev_torque(self, t):
    self.safety.set_desired_torque_last(t)
    self.safety.set_rt_torque_last(t)

  # Ego speed (Bremse_1)
  def _speed_msg(self, speed):
    values = {"BR1_Rad_kmh": speed}
    return self.packer.make_can_msg_safety("Bremse_1", 0, values)

  # Brake light switch (shared message Motor_2)
  def _user_brake_msg(self, brake):
    # since this signal is used for engagement status, preserve current state
    return self._motor_2_msg(brake_pressed=brake, cruise_engaged=self.safety.get_controls_allowed())

  # ACC engaged status (shared message Motor_2)
  def _pcm_status_msg(self, enable):
    self.__class__.cruise_engaged = enable
    return self._motor_2_msg(cruise_engaged=enable)

  # Acceleration request to drivetrain coordinator
  def _accel_msg(self, accel):
    values = {"ACS_Sollbeschl": accel}
    return self.packer.make_can_msg_safety("ACC_System", 0, values)

  # Driver steering input torque
  def _torque_driver_msg(self, torque):
    values = {"LH3_LM": abs(torque), "LH3_LMSign": torque < 0}
    return self.packer.make_can_msg_safety("Lenkhilfe_3", 0, values)

  # openpilot steering output torque
  def _torque_cmd_msg(self, torque, steer_req=1, hca_status=7):
    values = {"LM_Offset": abs(torque), "LM_OffSign": torque < 0, "HCA_Status": hca_status if steer_req else 3}
    return self.packer.make_can_msg_safety("HCA_1", 0, values)

  # ACC engagement and brake light switch status
  # Called indirectly for compatibility with common.py tests
  def _motor_2_msg(self, brake_pressed=False, cruise_engaged=False):
    values = {"MO2_BLS": brake_pressed,
              "MO2_Sta_GRA": cruise_engaged}
    return self.packer.make_can_msg_safety("Motor_2", 0, values)

  # ACC main switch status
  def _motor_5_msg(self, main_switch=False):
    values = {"MO5_GRA_Hauptsch": main_switch}
    return self.packer.make_can_msg_safety("Motor_5", 0, values)

  # Driver throttle input (Motor_3)
  def _user_gas_msg(self, gas):
    values = {"MO3_Pedalwert": gas}
    return self.packer.make_can_msg_safety("Motor_3", 0, values)

  # Cruise control buttons (GRA_Neu)
  def _button_msg(self, _set=False, resume=False, cancel=False, bus=2):
    values = {"GRA_Neu_Setzen": _set, "GRA_Recall": resume, "GRA_Abbrechen": cancel}
    return self.packer.make_can_msg_safety("GRA_Neu", bus, values)


class TestVolkswagenPqStockSafety(TestVolkswagenPqSafetyBase):
  # Transmit of GRA_Neu is allowed on bus 0 and 2 to keep compatibility with gateway and camera integration
  SAFETY_MODEL = CarParams.SafetyModel.volkswagenPq

  TX_MSGS = [[MSG_HCA_1, 0], [MSG_GRA_NEU, 0], [MSG_GRA_NEU, 2], [MSG_LDW_1, 0]]
  FWD_BLACKLISTED_ADDRS = {2: [MSG_HCA_1, MSG_LDW_1]}

  def test_spam_cancel_safety_check(self):
    self.safety.set_controls_allowed(0)
    self.assertTrue(self._tx(self._button_msg(cancel=True)))
    self.assertFalse(self._tx(self._button_msg(resume=True)))
    self.assertFalse(self._tx(self._button_msg(_set=True)))
    # do not block resume if we are engaged already
    self.safety.set_controls_allowed(1)
    self.assertTrue(self._tx(self._button_msg(resume=True)))


class TestVolkswagenPqCcSafetyBase(TestVolkswagenPqSafetyBase):
  SAFETY_MODEL = CarParams.SafetyModel.volkswagenPq
  SAFETY_PARAM = VolkswagenSafetyFlags.PQ_CC_ONLY
  RX_BUS = 0
  STEER_BUS = 0
  FWD_BUS_LOOKUP = {2: 0}
  FWD_BLACKLISTED_ADDRS = {2: [MSG_HCA_1, MSG_LDW_1]}
  TX_MSGS = [[MSG_HCA_1, 0], [MSG_LDW_1, 0], [MSG_GRA_NEU, 0]]

  def _pt_msg(self, msg):
    msg[0].bus = self.RX_BUS
    return msg

  def _speed_msg(self, speed):
    return self._pt_msg(super()._speed_msg(speed))

  def _motor_2_msg(self, brake_pressed=False, cruise_engaged=False):
    return self._pt_msg(super()._motor_2_msg(brake_pressed, cruise_engaged))

  def _motor_5_msg(self, main_switch=False):
    return self._pt_msg(super()._motor_5_msg(main_switch))

  def _user_gas_msg(self, gas):
    return self._pt_msg(super()._user_gas_msg(gas))

  def _torque_driver_msg(self, torque):
    return self._pt_msg(super()._torque_driver_msg(torque))

  def _torque_cmd_msg(self, torque, steer_req=1, hca_status=7):
    msg = super()._torque_cmd_msg(torque, steer_req, hca_status)
    msg[0].bus = self.STEER_BUS
    return msg

  def test_only_cancel_buttons_allowed(self):
    for allowed in (False, True):
      self.safety.set_controls_allowed(allowed)
      self.assertTrue(self._tx(self._button_msg(cancel=True, bus=self.RX_BUS)))
      self.assertFalse(self._tx(self._button_msg(resume=True, bus=self.RX_BUS)))
      self.assertFalse(self._tx(self._button_msg(_set=True, bus=self.RX_BUS)))

  def test_longitudinal_flag_cannot_enable_acc_commands(self):
    self.safety.set_safety_hooks(self.SAFETY_MODEL, self.SAFETY_PARAM | VolkswagenSafetyFlags.LONG_CONTROL)
    self.safety.set_controls_allowed(True)
    self.assertFalse(self._tx(self._accel_msg(0.0)))

  def test_cruise_on_wrong_bus_cannot_enable(self):
    msg = self._motor_2_msg(cruise_engaged=True)
    msg[0].bus = 1 - self.RX_BUS
    self._rx(msg)
    self.assertFalse(self.safety.get_controls_allowed())


class TestVolkswagenPqCcSafety(TestVolkswagenPqCcSafetyBase):
  pass


class TestVolkswagenPqGatewayCcSafety(TestVolkswagenPqCcSafetyBase):
  SAFETY_PARAM = VolkswagenSafetyFlags.PQ_CC_ONLY | VolkswagenSafetyFlags.PQ_POWERTRAIN_BUS1
  RX_BUS = 1
  TX_MSGS = [[MSG_HCA_1, 0], [MSG_LDW_1, 0], [MSG_GRA_NEU, 1]]


class TestVolkswagenPqLowlineCcSafety(TestVolkswagenPqGatewayCcSafety):
  SAFETY_PARAM = TestVolkswagenPqGatewayCcSafety.SAFETY_PARAM | VolkswagenSafetyFlags.PQ_STEERING_BUS1
  STEER_BUS = 1
  TX_MSGS = [[MSG_HCA_1, 1], [MSG_LDW_1, 1], [MSG_GRA_NEU, 1]]
  RELAY_MALFUNCTION_ADDRS = {1: (MSG_HCA_1, MSG_LDW_1)}
  FWD_BLACKLISTED_ADDRS = {}


class TestVolkswagenPqLongSafety(TestVolkswagenPqSafetyBase, common.LongitudinalAccelSafetyTest):
  SAFETY_MODEL = CarParams.SafetyModel.volkswagenPq
  SAFETY_PARAM = VolkswagenSafetyFlags.LONG_CONTROL

  TX_MSGS = [[MSG_HCA_1, 0], [MSG_LDW_1, 0], [MSG_ACC_SYSTEM, 0], [MSG_ACC_GRA_ANZEIGE, 0]]
  FWD_BLACKLISTED_ADDRS = {2: [MSG_HCA_1, MSG_LDW_1, MSG_ACC_SYSTEM, MSG_ACC_GRA_ANZEIGE]}
  RELAY_MALFUNCTION_ADDRS = {0: (MSG_HCA_1, MSG_LDW_1, MSG_ACC_SYSTEM, MSG_ACC_GRA_ANZEIGE)}
  INACTIVE_ACCEL = 3.01

  # stock cruise controls are entirely bypassed under openpilot longitudinal control
  def test_disable_control_allowed_from_cruise(self):
    pass

  def test_enable_control_allowed_from_cruise(self):
    pass

  def test_cruise_engaged_prev(self):
    pass

  def test_set_and_resume_buttons(self):
    for button in ["set", "resume"]:
      # ACC main switch must be on, engage on falling edge
      self.safety.set_controls_allowed(0)
      self._rx(self._motor_5_msg(main_switch=False))
      self._rx(self._button_msg(_set=(button == "set"), resume=(button == "resume"), bus=0))
      self._rx(self._button_msg(bus=0))
      self.assertFalse(self.safety.get_controls_allowed(), f"controls allowed on {button} with main switch off")
      self._rx(self._motor_5_msg(main_switch=True))
      self._rx(self._button_msg(_set=(button == "set"), resume=(button == "resume"), bus=0))
      self.assertFalse(self.safety.get_controls_allowed(), f"controls allowed on {button} rising edge")
      self._rx(self._button_msg(bus=0))
      self.assertTrue(self.safety.get_controls_allowed(), f"controls not allowed on {button} falling edge")

  def test_cancel_button(self):
    # Disable on rising edge of cancel button
    self._rx(self._motor_5_msg(main_switch=True))
    self.safety.set_controls_allowed(1)
    self._rx(self._button_msg(cancel=True, bus=0))
    self.assertFalse(self.safety.get_controls_allowed(), "controls allowed after cancel")

  def test_main_switch(self):
    # Disable as soon as main switch turns off
    self._rx(self._motor_5_msg(main_switch=True))
    self.safety.set_controls_allowed(1)
    self._rx(self._motor_5_msg(main_switch=False))
    self.assertFalse(self.safety.get_controls_allowed(), "controls allowed after ACC main switch off")

  def test_torque_cmd_enable_variants(self):
    # The EPS rack accepts either 5 or 7 for an enabled status, with different low speed tuning behavior
    self.safety.set_controls_allowed(1)
    for enabled_status in (5, 7):
      self.assertTrue(self._tx(self._torque_cmd_msg(self.MAX_RATE_UP, steer_req=1, hca_status=enabled_status)),
                      f"torque cmd rejected with {enabled_status=}")


class TestVolkswagenPqAolSafety(TestVolkswagenPqCcSafetyBase):
  SAFETY_PARAM = VolkswagenSafetyFlags.PQ_CC_ONLY | VolkswagenSafetyFlags.PQ_AOL

  def test_enable_control_allowed_from_cruise(self):
    self._rx(self._motor_5_msg(main_switch=True))
    self.safety.set_controls_allowed(False)
    self._rx(self._pcm_status_msg(False))
    self._rx(self._pcm_status_msg(True))
    self.assertTrue(self.safety.get_controls_allowed())

  def test_disable_control_allowed_from_cruise(self):
    # AOL is independent of stock cruise engagement; main off still kills it.
    self._rx(self._motor_5_msg(main_switch=True))
    self._rx(self._pcm_status_msg(True))
    self._rx(self._pcm_status_msg(False))
    self.assertTrue(self.safety.get_controls_allowed())
    self._rx(self._motor_5_msg(main_switch=False))
    self.assertFalse(self.safety.get_controls_allowed())

  def test_allow_user_brake_at_zero_speed(self):
    # Braking preserves existing lateral permission; it cannot arm it.
    self._rx(self._speed_msg(0))
    self._rx(self._user_brake_msg(True))
    self.safety.set_controls_allowed(True)
    self._rx(self._user_brake_msg(True))
    self.assertTrue(self.safety.get_controls_allowed())

  def test_main_rising_only(self):
    # An already-on switch at startup does not arm lateral control.
    self._rx(self._motor_5_msg(main_switch=True))
    self.assertFalse(self.safety.get_controls_allowed())
    self._rx(self._motor_5_msg(main_switch=False))
    self._rx(self._motor_5_msg(main_switch=True))
    self.assertTrue(self.safety.get_controls_allowed())
    self.safety.set_controls_allowed(False)
    self._rx(self._motor_5_msg(main_switch=True))
    self.assertFalse(self.safety.get_controls_allowed())

  def test_main_cycle_blocked_by_brake_or_fault(self):
    for brake, fault in ((True, False), (False, True)):
      self._rx(self._motor_5_msg(main_switch=False))
      self._rx(self._motor_2_msg(brake_pressed=brake, cruise_engaged=3 if fault else 0))
      self._rx(self._motor_5_msg(main_switch=True))
      self.assertFalse(self.safety.get_controls_allowed())
      self._rx(self._motor_2_msg(brake_pressed=False, cruise_engaged=0))
      self._rx(self._motor_5_msg(main_switch=True))
      self.assertFalse(self.safety.get_controls_allowed())

  def test_main_cycle_does_not_bypass_torque_or_longitudinal_limits(self):
    self._rx(self._motor_5_msg(main_switch=False))
    self._rx(self._motor_5_msg(main_switch=True))
    self.assertTrue(self.safety.get_controls_allowed())
    self.assertTrue(self._tx(self._torque_cmd_msg(self.MAX_RATE_UP)))
    self.assertFalse(self._tx(self._torque_cmd_msg(self.MAX_RATE_UP * 3)))
    self.assertFalse(self._tx(self._accel_msg(0)))
    self.assertFalse(self._tx(self._button_msg(_set=True, bus=self.RX_BUS)))
    self.assertFalse(self._tx(self._button_msg(resume=True, bus=self.RX_BUS)))

  def test_not_allow_user_brake_when_moving(self):
    # Override the stock-cruise expectation only in explicit CC-only AOL.
    for speed in (0, 1, 10):
      self._rx(self._speed_msg(speed))
      self._rx(self._user_brake_msg(True))
      self.safety.set_controls_allowed(True)
      self._rx(self._user_brake_msg(True))
      self.assertTrue(self.safety.get_controls_allowed())

  def test_brake_transitions_to_lateral_only_and_cancel_still_stops(self):
    self._rx(self._motor_5_msg(main_switch=True))
    self._rx(self._pcm_status_msg(True))
    self.assertTrue(self.safety.get_controls_allowed())
    self._rx(self._speed_msg(20))
    self._rx(self._motor_2_msg(brake_pressed=True, cruise_engaged=False))
    self.assertTrue(self.safety.get_controls_allowed())
    self.assertTrue(self._tx(self._torque_cmd_msg(self.MAX_RATE_UP)))
    self.assertFalse(self._tx(self._accel_msg(0)))
    self._rx(self._button_msg(cancel=True, bus=self.RX_BUS))
    self.assertFalse(self.safety.get_controls_allowed())
    self.assertFalse(self._tx(self._torque_cmd_msg(self.MAX_RATE_UP)))

  def test_brake_retention_does_not_leak_to_another_mode(self):
    self.safety.set_safety_hooks(CarParams.SafetyModel.volkswagen, 0)
    packer = common.CANPackerSafety('vw_mqb')
    self.safety.set_controls_allowed(True)
    msg = packer.make_can_msg_safety('Motor_14', 0, {'MO_Fahrer_bremst': True})
    self._rx(msg)
    self.assertFalse(self.safety.get_controls_allowed())

  def test_button_release_and_cancel(self):
    for button in ('set', 'resume'):
      self._rx(self._motor_5_msg(main_switch=True))
      self.safety.set_controls_allowed(False)
      self._rx(self._button_msg(_set=button == 'set', resume=button == 'resume', bus=self.RX_BUS))
      self.assertFalse(self.safety.get_controls_allowed())
      self._rx(self._button_msg(bus=self.RX_BUS))
      self.assertTrue(self.safety.get_controls_allowed())
      self._rx(self._button_msg(cancel=True, bus=self.RX_BUS))
      self.assertFalse(self.safety.get_controls_allowed())
      self._rx(self._button_msg(bus=self.RX_BUS))
      self._rx(self._motor_5_msg(main_switch=True))
      self.assertFalse(self.safety.get_controls_allowed())

  def test_brake_held_prevents_arming(self):
    self._rx(self._user_brake_msg(True))
    self._rx(self._motor_5_msg(main_switch=True))
    self._rx(self._button_msg(_set=True, bus=self.RX_BUS))
    self._rx(self._button_msg(bus=self.RX_BUS))
    self.assertFalse(self.safety.get_controls_allowed())
    self._rx(self._user_brake_msg(False))
    self._rx(self._motor_5_msg(main_switch=True))
    self.assertFalse(self.safety.get_controls_allowed())

  def test_wrong_bus_main_and_buttons(self):
    msg = self._motor_5_msg(main_switch=True)
    msg[0].bus = 1 - self.RX_BUS
    self._rx(msg)
    self.assertFalse(self.safety.get_controls_allowed())
    self._rx(self._motor_5_msg(main_switch=True))
    self.safety.set_controls_allowed(False)
    self._rx(self._button_msg(_set=True, bus=1 - self.RX_BUS))
    self._rx(self._button_msg(bus=1 - self.RX_BUS))
    self.assertFalse(self.safety.get_controls_allowed())

  def test_cancel_dominates_simultaneous_release(self):
    self._rx(self._motor_5_msg(main_switch=True))
    self._rx(self._button_msg(_set=True, bus=self.RX_BUS))
    self._rx(self._button_msg(cancel=True, bus=self.RX_BUS))
    self.assertFalse(self.safety.get_controls_allowed())
    self._rx(self._button_msg(bus=self.RX_BUS))
    self.assertFalse(self.safety.get_controls_allowed())

  def test_cruise_fault_blocks_buttons(self):
    self._rx(self._motor_5_msg(main_switch=True))
    self._rx(self._motor_2_msg(cruise_engaged=3))
    self.assertFalse(self.safety.get_controls_allowed())
    self._rx(self._button_msg(_set=True, bus=self.RX_BUS))
    self._rx(self._button_msg(bus=self.RX_BUS))
    self.assertFalse(self.safety.get_controls_allowed())

  def test_aol_requires_cc_only_flag(self):
    self.safety.set_safety_hooks(self.SAFETY_MODEL, VolkswagenSafetyFlags.PQ_AOL)
    self._rx(self._motor_5_msg(main_switch=True))
    self.assertFalse(self.safety.get_controls_allowed())

  def test_fault_recovery_does_not_arm_stale_button(self):
    self._rx(self._motor_5_msg(main_switch=True))
    self._rx(self._motor_2_msg(cruise_engaged=3))
    self._rx(self._button_msg(_set=True, bus=self.RX_BUS))
    self._rx(self._motor_2_msg(cruise_engaged=False))
    self._rx(self._button_msg(bus=self.RX_BUS))
    self.assertFalse(self.safety.get_controls_allowed())

  def test_cancel_held_blocks_main_cycle(self):
    self._rx(self._button_msg(cancel=True, bus=self.RX_BUS))
    self._rx(self._motor_5_msg(main_switch=False))
    self._rx(self._motor_5_msg(main_switch=True))
    self.assertFalse(self.safety.get_controls_allowed())


class TestVolkswagenPqGatewayAolSafety(TestVolkswagenPqAolSafety):
  SAFETY_PARAM = TestVolkswagenPqAolSafety.SAFETY_PARAM | VolkswagenSafetyFlags.PQ_POWERTRAIN_BUS1
  RX_BUS = 1
  TX_MSGS = [[MSG_HCA_1, 0], [MSG_LDW_1, 0], [MSG_GRA_NEU, 1]]


class TestVolkswagenPqLowlineAolSafety(TestVolkswagenPqGatewayAolSafety):
  SAFETY_PARAM = TestVolkswagenPqGatewayAolSafety.SAFETY_PARAM | VolkswagenSafetyFlags.PQ_STEERING_BUS1
  STEER_BUS = 1
  TX_MSGS = [[MSG_HCA_1, 1], [MSG_LDW_1, 1], [MSG_GRA_NEU, 1]]
  RELAY_MALFUNCTION_ADDRS = {1: (MSG_HCA_1, MSG_LDW_1)}
  FWD_BLACKLISTED_ADDRS = {}


if __name__ == "__main__":
  unittest.main()
