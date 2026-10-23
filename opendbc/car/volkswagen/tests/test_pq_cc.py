import pytest

from opendbc.can import CANPacker
from opendbc.car import Bus, gen_empty_fingerprint, structs
from opendbc.car.volkswagen.carcontroller import CarController
from opendbc.car.volkswagen.carstate import CarState
from opendbc.car.volkswagen.interface import CarInterface
from opendbc.car.volkswagen.values import CAR, DBC, CanBus, VolkswagenFlags, VolkswagenSafetyFlags


def cc_params(pt_bus=1, lowline=False, acc=False, candidate=CAR.VOLKSWAGEN_PASSAT_NMS, alpha_long=True):
  fp = gen_empty_fingerprint()
  fp[pt_bus] = {0x288: 8, 0x380: 8, 0x480: 8, 0x440: 8, 0x1A0: 8, 0xC2: 8}
  if pt_bus == 1 and not lowline:
    fp[0] = {0xD0: 6}
  if acc:
    fp[2][0x368] = 8
  return CarInterface.get_params(candidate, fp, [], alpha_long, is_release=True, docs=False)


@pytest.mark.parametrize(('pt_bus', 'lowline', 'steer_bus'), [(0, False, 0), (1, False, 0), (1, True, 1)])
def test_factory_cc_parameters(pt_bus, lowline, steer_bus):
  cp = cc_params(pt_bus, lowline)
  assert cp.flags & VolkswagenFlags.PQ_CC_ONLY
  assert not cp.dashcamOnly
  assert cp.pcmCruise and cp.radarUnavailable
  assert not cp.openpilotLongitudinalControl
  assert not cp.alphaLongitudinalAvailable
  assert cp.transmissionType == structs.CarParams.TransmissionType.automatic
  assert not cp.safetyConfigs[-1].safetyParam & VolkswagenSafetyFlags.LONG_CONTROL
  buses = CanBus(cp)
  assert buses.powertrain == pt_bus
  assert buses.pt == steer_bus
  assert buses.ext == pt_bus
  assert CarState.get_can_parsers(cp)[Bus.pt].bus == pt_bus


@pytest.mark.parametrize('candidate', [CAR.VOLKSWAGEN_PASSAT_NMS, CAR.VOLKSWAGEN_JETTA_MK6])
def test_other_configurations_remain_dashcam(candidate):
  cp = cc_params(acc=True, candidate=candidate, alpha_long=False)
  assert not cp.flags & VolkswagenFlags.PQ_CC_ONLY
  assert cp.dashcamOnly


def test_empty_fingerprint_does_not_enable_cc():
  cp = CarInterface.get_params(CAR.VOLKSWAGEN_PASSAT_NMS, gen_empty_fingerprint(), [], False, True, False)
  assert not cp.flags & VolkswagenFlags.PQ_CC_ONLY
  assert cp.dashcamOnly


def test_powertrain_cluster_message_does_not_imply_acc_radar():
  # Captured Golf route: engine/steering data on buses 0 and 1, cluster 0x56A only on 1.
  fp = gen_empty_fingerprint()
  fp[0] = {0x288: 8, 0x380: 8, 0x480: 8, 0xD0: 6}
  fp[1] = {**fp[0], 0x440: 8, 0x56A: 8}
  cp = CarInterface.get_params(CAR.VOLKSWAGEN_PASSAT_NMS, fp, [], True, True, False)
  assert cp.flags & VolkswagenFlags.PQ_CC_ONLY
  assert not cp.dashcamOnly
  assert CanBus(cp).powertrain == 1
  assert CanBus(cp).pt == 0
  assert cp.pcmCruise and not cp.openpilotLongitudinalControl


@pytest.mark.parametrize('status', [0, 1, 2, 3])
@pytest.mark.parametrize('pt_bus', [0, 1])
def test_engine_cruise_without_radar_or_camera(status, pt_bus):
  cp = cc_params(pt_bus)
  state = CarState(cp)
  parsers = state.get_can_parsers(cp)
  packer = CANPacker('vw_pq')
  # First update registers the dynamically discovered signals in the real CAN parsers.
  state.update(parsers)
  frames = [
    packer.make_can_msg('Motor_2', pt_bus, {'MO2_Sta_GRA': status, 'MO2_GRA_Soll': 96}),
    packer.make_can_msg('Motor_5', pt_bus, {'MO5_GRA_Hauptsch': 1}),
  ]
  for parser in parsers.values():
    parser.update([(1_000_000_000, frames)])
  out = state.update(parsers)
  assert out.cruiseState.enabled == (status in (1, 2))
  assert out.accFaulted == (status == 3)
  assert out.cruiseState.available == (status != 3)
  assert out.cruiseState.nonAdaptive
  assert out.cruiseState.speed == pytest.approx(96 / 3.6)
  assert not {0x368, 0x56A} & parsers[Bus.pt].addresses
  assert not parsers[Bus.cam].addresses
  assert not parsers[Bus.alt].addresses


@pytest.mark.parametrize('lowline', [False, True])
def test_controller_only_steers_and_cancels(lowline):
  cp = cc_params(lowline=lowline)
  state = CarState(cp)
  state.out = state.update(state.get_can_parsers(cp))
  controller = CarController(DBC[cp.carFingerprint], cp)
  command = structs.CarControl()
  command.cruiseControl.resume = True
  _, sends = controller.update(command.as_reader(), state, 0)
  assert all(msg[0] not in (0x368, 0x56A, 0x38A) for msg in sends)
  assert next(msg for msg in sends if msg[0] == 0xD2)[2] == (1 if lowline else 0)
  controller.gra_acc_counter_last = None
  command.cruiseControl.cancel = True
  _, sends = controller.update(command.as_reader(), state, 10_000_000)
  assert next(msg for msg in sends if msg[0] == 0x38A)[2] == 1
