from cereal import car
from selfdrive.car import make_can_msg
from selfdrive.car.proton.protoncan import create_can_steer_command, create_hud, create_lead_detect, send_buttons, create_acc_cmd
from selfdrive.car.proton.values import CAR, DBC
from selfdrive.controls.lib.desire_helper import LANE_CHANGE_SPEED_MIN
from opendbc.can.packer import CANPacker
from common.numpy_fast import clip, interp
from common.realtime import DT_CTRL
from common.params import Params
import cereal.messaging as messaging

from common.features import Features

RES_INTERVAL = 500
RES_LEN = 2  # Press resume for 2 frames

def apply_proton_steer_torque_limits(apply_torque, apply_torque_last, driver_torque, limits):
    """
    Limits steer torque based on driver torque and set values.

    Args:
        apply_torque: Desired torque from control system.
        apply_torque_last: Last applied torque.
        driver_torque: Estimated driver torque input.
        limits: Car specific limits for steer torque.

    Returns:
        Limited steer torque value.
    """

    driver_max_torque = limits.STEER_MAX + driver_torque * 25.5
    driver_min_torque = -limits.STEER_MAX + driver_torque * 25.5
    max_steer_allowed = max(min(limits.STEER_MAX, driver_max_torque), 0)
    min_steer_allowed = min(max(-limits.STEER_MAX, driver_min_torque), 0)
    apply_torque = clip(apply_torque, min_steer_allowed, max_steer_allowed)

    # Slower rate increase for higher torque changes
    if apply_torque_last > 0:
        delta_up = limits.STEER_DELTA_UP
        delta_down = limits.STEER_DELTA_DOWN / 2.0  # Slower decrease
    else:
        delta_up = limits.STEER_DELTA_UP / 2.0  # Slower increase
        delta_down = limits.STEER_DELTA_DOWN

    apply_torque = clip(apply_torque, max(apply_torque_last - delta_down, -delta_up),
                       apply_torque_last + delta_up)

    return int(round(float(apply_torque)))


class CarControllerParams:
    """
    Stores car specific parameters for the CarController.
    """
    def __init__(self, CP):
        self.STEER_MAX = CP.lateralParams.torqueV[0]
        assert len(CP.lateralParams.torqueV) == 1, "Proton only has one max steer torque value"

        # Limits for steer torque rate changes
        self.STEER_DELTA_UP = 20  # Torque increase per refresh
        self.STEER_DELTA_DOWN = 30  # Torque decrease per refresh


class CarController:
    def __init__(self, dbc_name, CP, VM):
        self.last_steer = 0
        self.steer_rate_limited = False
        self.steering_direction = False
        self.params = CarControllerParams(CP)
        self.packer = CANPacker(DBC[CP.carFingerprint]['pt'])
        self.disable_radar = Params().get_bool("DisableRadar")
        self.num_cruise_btn_sent = 0
        self.temp_lead_dist = 0  # The last lead distance before standstill
        self.last_res_press_frame = 0  # The frame where the last resume press was finished
        self.resume_counter = 0  # Counter for tracking the progress of a resume press

        self.mads = Features().has("StockAcc")  # Check for MADS support

    def update(self, enabled, CS, frame, actuators, lead_visible, rlane_visible, llane_visible, pcm_cancel, ldw, laneActive):
        can_sends = []
        lat_active = enabled and laneActive

        # Disable radar with tester present (no response)
        if CS.CP.openpilotLongitudinalControl and self.disable_radar:
            if frame % 10 == 0:
                can_sends.append([0x7D0, 0
