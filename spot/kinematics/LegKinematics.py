# https://www.researchgate.net/publication/320307716_Inverse_Kinematic_Analysis_Of_A_Quadruped_Robot

import numpy as np

class LegIK():
    def __init__(self,
                legtype="RIGHT",
                shoulder_length=0.04,
                elbow_length=0.1,
                wrist_length=0.125,
                hip_lim=[-0.548, 0.548],
                shoulder_lim=[-2.17, 0.97],
                leg_lim=[-0.1, 2.59]):
        """
        Initialize the LegIK class with the given parameters.

        :param legtype: Type of leg ("RIGHT" or "LEFT")
        :param shoulder_length: Length of the shoulder segment
        :param elbow_length: Length of the elbow segment
        :param wrist_length: Length of the wrist segment
        :param hip_lim: Limits for the hip joint
        :param shoulder_lim: Limits for the shoulder joint
        :param leg_lim: Limits for the leg joint
        """
        self.legtype = legtype
        self.shoulder_length = shoulder_length
        self.elbow_length = elbow_length
        self.wrist_length = wrist_length
        self.hip_lim = hip_lim
        self.shoulder_lim = shoulder_lim
        self.leg_lim = leg_lim

    def get_domain(self, x, y, z):
        """
        Calculates the leg's domain and caps it in case of a breach.

        :param x: Hip-to-foot distance in the x-dimension
        :param y: Hip-to-foot distance in the y-dimension
        :param z: Hip-to-foot distance in the z-dimension
        :return: Leg domain D
        """
        D = (y**2 + (-z)**2 - self.shoulder_length**2 +
             (-x)**2 - self.elbow_length**2 - self.wrist_length**2) / (
                 2 * self.wrist_length * self.elbow_length)
        # Cap D within the range [-1, 1] to prevent domain breach
        if D > 1 or D < -1:
            D = np.clip(D, -1.0, 1.0)
        return D

    def solve(self, xyz_coord):
        """
        Generic leg inverse kinematics solver.

        :param xyz_coord: Hip-to-foot distances in each dimension
        :return: Joint angles required for desired position
        """
        x, y, z = xyz_coord
        D = self.get_domain(x, y, z)
        if self.legtype == "RIGHT":
            return self.RightIK(x, y, z, D)
        else:
            return self.LeftIK(x, y, z, D)

    def RightIK(self, x, y, z, D):
        """
        Right leg inverse kinematics solver.

        :param x: Hip-to-foot distance in the x-dimension
        :param y: Hip-to-foot distance in the y-dimension
        :param z: Hip-to-foot distance in the z-dimension
        :param D: Leg domain
        :return: Joint angles required for desired position
        """
        wrist_angle = np.arctan2(-np.sqrt(1 - D**2), D)
        sqrt_component = y**2 + (-z)**2 - self.shoulder_length**2
        if sqrt_component < 0.0:
            sqrt_component = 0.0  # Prevent negative value under the square root
        shoulder_angle = -np.arctan2(z, y) - np.arctan2(np.sqrt(sqrt_component), -self.shoulder_length)
        elbow_angle = np.arctan2(-x, np.sqrt(sqrt_component)) - np.arctan2(
            self.wrist_length * np.sin(wrist_angle),
            self.elbow_length + self.wrist_length * np.cos(wrist_angle)
        )
        joint_angles = np.array([-shoulder_angle, elbow_angle, wrist_angle])
        return joint_angles

    def LeftIK(self, x, y, z, D):
        """
        Left leg inverse kinematics solver.

        :param x: Hip-to-foot distance in the x-dimension
        :param y: Hip-to-foot distance in the y-dimension
        :param z: Hip-to-foot distance in the z-dimension
        :param D: Leg domain
        :return: Joint angles required for desired position
        """
        wrist_angle = np.arctan2(-np.sqrt(1 - D**2), D)
        sqrt_component = y**2 + (-z)**2 - self.shoulder_length**2
        if sqrt_component < 0.0:
            sqrt_component = 0.0  # Prevent negative value under the square root
        shoulder_angle = -np.arctan2(z, y) - np.arctan2(np.sqrt(sqrt_component), self.shoulder_length)
        elbow_angle = np.arctan2(-x, np.sqrt(sqrt_component)) - np.arctan2(
            self.wrist_length * np.sin(wrist_angle),
            self.elbow_length + self.wrist_length * np.cos(wrist_angle)
        )
        joint_angles = np.array([-shoulder_angle, elbow_angle, wrist_angle])
        return joint_angles
