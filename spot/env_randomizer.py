"""
CODE BASED ON EXAMPLE FROM:
@misc{coumans2017pybullet,
  title={Pybullet, a python module for physics simulation in robotics, games and machine learning},
  author={Coumans, Erwin and Bai, Yunfei},
  url={www.pybullet.org},
  year={2017},
}

Example: minitaur_env_randomizer.py
https://github.com/bulletphysics/bullet3/blob/master/examples/pybullet/gym/pybullet_envs/minitaur/envs/env_randomizers/minitaur_env_randomizer.py
"""

import numpy as np
from . import env_randomizer_base

# Relative range for mass errors.
spot_BASE_MASS_ERROR_RANGE = (-0.2, 0.2)  # 0.2 means ±20% relative error
spot_LEG_MASS_ERROR_RANGE = (-0.2, 0.2)  # 0.2 means ±20% relative error

# Absolute range for other parameters.
BATTERY_VOLTAGE_RANGE = (7.0, 8.4)  # Unit: Volt
MOTOR_VISCOUS_DAMPING_RANGE = (0, 0.01)  # Unit: N*m*s/rad (torque/angular vel)
spot_LEG_FRICTION = (0.8, 1.5)  # Unit: dimensionless


class SpotEnvRandomizer(env_randomizer_base.EnvRandomizerBase):
    """A randomizer that changes the Spot robot's environment during every reset."""
    
    def __init__(self,
                 spot_base_mass_err_range=spot_BASE_MASS_ERROR_RANGE,
                 spot_leg_mass_err_range=spot_LEG_MASS_ERROR_RANGE,
                 battery_voltage_range=BATTERY_VOLTAGE_RANGE,
                 motor_viscous_damping_range=MOTOR_VISCOUS_DAMPING_RANGE):
        """
        Initialize the randomizer with specified ranges for randomization.

        Args:
            spot_base_mass_err_range (tuple): Range for base mass randomization.
            spot_leg_mass_err_range (tuple): Range for leg masses randomization.
            battery_voltage_range (tuple): Range for battery voltage randomization.
            motor_viscous_damping_range (tuple): Range for motor viscous damping randomization.
        """
        self._spot_base_mass_err_range = spot_base_mass_err_range
        self._spot_leg_mass_err_range = spot_leg_mass_err_range
        self._battery_voltage_range = battery_voltage_range
        self._motor_viscous_damping_range = motor_viscous_damping_range

        np.random.seed(0)  # Set random seed for reproducibility

    def randomize_env(self, env):
        """
        Randomize the Spot robot's environment.

        This method is called during environment reset to apply randomizations.

        Args:
            env (object): The environment object containing the Spot robot.
        """
        self._randomize_spot(env.spot)

    def _randomize_spot(self, spot):
        """
        Randomize various physical properties of the Spot robot.

        Args:
            spot (object): The Spot robot instance in the environment.
        """
        # Randomize base mass
        base_mass = spot.GetBaseMassFromURDF()
        randomized_base_mass = np.random.uniform(
            np.array([base_mass]) * (1.0 + self._spot_base_mass_err_range[0]),
            np.array([base_mass]) * (1.0 + self._spot_base_mass_err_range[1]))
        spot.SetBaseMass(randomized_base_mass[0])

        # Randomize leg masses
        leg_masses = spot.GetLegMassesFromURDF()
        leg_masses_lower_bound = np.array(leg_masses) * (
            1.0 + self._spot_leg_mass_err_range[0])
        leg_masses_upper_bound = np.array(leg_masses) * (
            1.0 + self._spot_leg_mass_err_range[1])
        randomized_leg_masses = [
            np.random.uniform(leg_masses_lower_bound[i],
                              leg_masses_upper_bound[i])
            for i in range(len(leg_masses))
        ]
        spot.SetLegMasses(randomized_leg_masses)

        # Randomize battery voltage
        randomized_battery_voltage = np.random.uniform(
            self._battery_voltage_range[0], self._battery_voltage_range[1])
        spot.SetBatteryVoltage(randomized_battery_voltage)

        # Randomize motor viscous damping
        randomized_motor_damping = np.random.uniform(
            self._motor_viscous_damping_range[0],
            self._motor_viscous_damping_range[1])
        spot.SetMotorViscousDamping(randomized_motor_damping)

        # Randomize leg friction
        randomized_foot_friction = np.random.uniform(
            spot_LEG_FRICTION[0], spot_LEG_FRICTION[1])
        spot.SetFootFriction(randomized_foot_friction)
