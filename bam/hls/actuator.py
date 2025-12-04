# Copyright 2025 Marc Duclusaud & Grégoire Passault

# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at:

#     http://www.apache.org/licenses/LICENSE-2.0

import numpy as np
# from bam.actuator import Actuator
# from bam.parameter import Parameter
# from bam.testbench import Testbench, Pendulum
import sys
import os

sys.path.append("/media/jdx/jdx/bam/bam")
from actuator import Actuator
from parameter import Parameter
from testbench import Testbench, Pendulum
class HlsActuator(Actuator):
    def __init__(self, testbench_class: Testbench, damping=20.0):
        super().__init__(testbench_class)

        # Maximum current [A]
        self.max_amps = 1.3

        # Maximum input voltage [V]
        self.max_volts = 6.0

        # Damping factor
        self.damping = damping

    def initialize(self):
        # Torque constant [Nm/A] or [V/(rad/s)]
        self.model.kt = Parameter(0.45, 0.0, 20.0)

        # Motor resistance [Ohm]
        # self.model.R = Parameter(2.8, 1.0, 3.5)

        # Motor armature / apparent inertia [kg m^2]
        self.model.armature = Parameter(0.005, 0.001, 2.0)
        self.model.error_gain_ratio = Parameter(0.1, 0.0, 10.0)
        self.model.error_gain_ratio1 = Parameter(0.1, 0.0, 10.0)
        # Adjusting upper bounds for identification
        self.model.max_friction_base = 10.0
        self.model.max_load_friction = 1.0
        self.model.max_viscous_friction = 30.0

    def load_log(self, log: dict):
        super().load_log(log)

        self.kp = log["kp"]

        if "damping" in log:
            self.damping = log["damping"]

    def control_unit(self) -> str:
        return "amps"

    def compute_control(
        self, q_target: float, q: float, dq: float, dt: float
    ) -> float | None:
        # Target velocity is assumed to be 0
        amps = (q_target - q) * self.kp/4 * self.model.error_gain_ratio1.value + self.damping/8 * (0.0 - dq) * self.model.error_gain_ratio.value
        amps = np.clip(amps, -self.max_amps, self.max_amps)

        return amps

    def compute_torque(
        self, control: float | None, torque_enable: bool, q: float, dq: float
    ) -> float:
        # Computing the torque given the control signal
        # With eRob, control=None actually meany amps=0, and not a disconnection of the motor
        amps = control * torque_enable
        torque = self.model.kt.value * amps

        # Computing the torque boundaries given the maximum voltage and the back EMF
        # volts_bounded_torque = (
        #     self.model.kt.value / self.model.R.value
        # ) * self.max_volts
        # emf = (self.model.kt.value**2) * dq / self.model.R.value

        # min_torque = -volts_bounded_torque - emf
        # max_torque = volts_bounded_torque - emf
        # torque = np.clip(torque, min_torque, max_torque)

        return torque

    def get_extra_inertia(self) -> float:
        return self.model.armature.value