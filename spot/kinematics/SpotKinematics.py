import numpy as np
from spot.kinematics.LegKinematics import LegIK
from spot.kinematics.LieAlgebra import RpToTrans, TransToRp, TransInv, RPY, TransformVector
from collections import OrderedDict

class SpotModel:
    def __init__(self,
                shoulder_length=0.055,
                elbow_length=0.10652,
                wrist_length=0.145,
                hip_x=0.23,
                hip_y=0.075,
                foot_x=0.23,
                foot_y=0.185,
                height=0.20,
                com_offset=0.016,
                shoulder_lim=[-0.548, 0.548],
                elbow_lim=[-2.17, 0.97],
                wrist_lim=[-0.1, 2.59]):
        """
        Initialize the SpotModel class with the given parameters.
        This class handles the kinematics of the Spot robot.
        """
        # Center of Mass (COM) offset in x direction
        self.com_offset = com_offset

        # Leg Parameters
        self.shoulder_length = shoulder_length
        self.elbow_length = elbow_length
        self.wrist_length = wrist_length

        # Distance Between Hips
        self.hip_x = hip_x  # Length
        self.hip_y = hip_y  # Width

        # Distance Between Feet
        self.foot_x = foot_x  # Length
        self.foot_y = foot_y  # Width

        # Body Height
        self.height = height

        # Joint Parameters
        self.shoulder_lim = shoulder_lim
        self.elbow_lim = elbow_lim
        self.wrist_lim = wrist_lim

        # Dictionary to store Leg IK Solvers
        self.Legs = OrderedDict()
        self.Legs["FL"] = LegIK("LEFT", self.shoulder_length, self.elbow_length, self.wrist_length,
                                self.shoulder_lim, self.elbow_lim, self.wrist_lim)
        self.Legs["FR"] = LegIK("RIGHT", self.shoulder_length, self.elbow_length, self.wrist_length,
                                self.shoulder_lim, self.elbow_lim, self.wrist_lim)
        self.Legs["BL"] = LegIK("LEFT", self.shoulder_length, self.elbow_length, self.wrist_length,
                                self.shoulder_lim, self.elbow_lim, self.wrist_lim)
        self.Legs["BR"] = LegIK("RIGHT", self.shoulder_length, self.elbow_length, self.wrist_length,
                                self.shoulder_lim, self.elbow_lim, self.wrist_lim)

        # Transform of Hip relative to world frame
        Rwb = np.eye(3)  # Rotation matrix (identity)
        self.WorldToHip = OrderedDict()

        # Hip positions
        self.ph_FL = np.array([self.hip_x / 2.0, self.hip_y / 2.0, 0])
        self.WorldToHip["FL"] = RpToTrans(Rwb, self.ph_FL)

        self.ph_FR = np.array([self.hip_x / 2.0, -self.hip_y / 2.0, 0])
        self.WorldToHip["FR"] = RpToTrans(Rwb, self.ph_FR)

        self.ph_BL = np.array([-self.hip_x / 2.0, self.hip_y / 2.0, 0])
        self.WorldToHip["BL"] = RpToTrans(Rwb, self.ph_BL)

        self.ph_BR = np.array([-self.hip_x / 2.0, -self.hip_y / 2.0, 0])
        self.WorldToHip["BR"] = RpToTrans(Rwb, self.ph_BR)

        # Transform of Foot relative to world frame
        self.WorldToFoot = OrderedDict()

        # Foot positions
        self.pf_FL = np.array([self.foot_x / 2.0, self.foot_y / 2.0, -self.height])
        self.WorldToFoot["FL"] = RpToTrans(Rwb, self.pf_FL)

        self.pf_FR = np.array([self.foot_x / 2.0, -self.foot_y / 2.0, -self.height])
        self.WorldToFoot["FR"] = RpToTrans(Rwb, self.pf_FR)

        self.pf_BL = np.array([-self.foot_x / 2.0, self.foot_y / 2.0, -self.height])
        self.WorldToFoot["BL"] = RpToTrans(Rwb, self.pf_BL)

        self.pf_BR = np.array([-self.foot_x / 2.0, -self.foot_y / 2.0, -self.height])
        self.WorldToFoot["BR"] = RpToTrans(Rwb, self.pf_BR)

    def HipToFoot(self, orn, pos, T_bf):
        """
        Converts a desired position and orientation with respect to Spot's
        home position, with a desired body-to-foot Transform into a
        body-to-hip Transform, which is used to extract and return the Hip To Foot Vector.

        :param orn: A 3x1 np.array with Spot's Roll, Pitch, Yaw angles
        :param pos: A 3x1 np.array with Spot's X, Y, Z coordinates
        :param T_bf: Dictionary of desired body-to-foot Transforms.
        :return: Hip To Foot Vector for each of Spot's Legs.
        """
        # Get rotation component from roll, pitch, yaw
        Rb, _ = TransToRp(RPY(orn[0], orn[1], orn[2]))
        pb = pos
        T_wb = RpToTrans(Rb, pb)

        # Dictionary to store vectors
        HipToFoot_List = OrderedDict()

        # Iterate over each hip transform
        for key, T_wh in self.WorldToHip.items():
            # Extract vector component from desired body-to-foot transform
            _, p_bf = TransToRp(T_bf[key])

            # Step 1: Get T_bh for each leg
            T_bh = np.dot(TransInv(T_wb), T_wh)

            # Step 2: Get T_hf for each leg using vector addition and transform method
            _, p_bh = TransToRp(T_bh)
            p_hf0 = p_bf - p_bh

            T_hf = np.dot(TransInv(T_bh), T_bf[key])
            _, p_hf1 = TransToRp(T_hf)

            # They should yield the same result
            if not np.allclose(p_hf1, p_hf0):
                print("NOT EQUAL")

            p_hf = p_hf1
            HipToFoot_List[key] = p_hf

        return HipToFoot_List

    def IK(self, orn, pos, T_bf):
        """
        Uses HipToFoot() to convert a desired position and orientation
        with respect to Spot's home position into a Hip To Foot Vector,
        which is fed into the LegIK solver. Finally, the resultant joint
        angles are returned from the LegIK solver for each leg.

        :param orn: A 3x1 np.array with Spot's Roll, Pitch, Yaw angles
        :param pos: A 3x1 np.array with Spot's X, Y, Z coordinates
        :param T_bf: Dictionary of desired body-to-foot Transforms.
        :return: Joint angles for each of Spot's joints.
        """
        # Adjust x position by COM offset
        pos[0] += self.com_offset

        # Initialize array to store joint angles
        joint_angles = np.zeros((4, 3))

        # Steps 1 and 2 of pipeline: Get HipToFoot vectors
        HipToFoot = self.HipToFoot(orn, pos, T_bf)

        # Iterate over each leg and compute joint angles
        for i, (key, p_hf) in enumerate(HipToFoot.items()):
            joint_angles[i, :] = self.Legs[key].solve(p_hf)

        return joint_angles
