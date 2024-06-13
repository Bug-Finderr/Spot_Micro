import pybullet as pb
import time
import numpy as np
import sys


class gui:
    def __init__(self, quadruped):
        time.sleep(0.5)

        # Camera settings
        self.cyaw = 0
        self.cpitch = -7
        self.cdist = 0.66

        # Adding user debug parameters for GUI
        self.xId = pb.addUserDebugParameter("x", -0.10, 0.10, 0.0)
        self.yId = pb.addUserDebugParameter("y", -0.10, 0.10, 0.0)
        self.zId = pb.addUserDebugParameter("z", -0.055, 0.17, 0.0)
        self.rollId = pb.addUserDebugParameter("roll", -np.pi / 4, np.pi / 4, 0.0)
        self.pitchId = pb.addUserDebugParameter("pitch", -np.pi / 4, np.pi / 4, 0.0)
        self.yawId = pb.addUserDebugParameter("yaw", -np.pi / 4, np.pi / 4, 0.0)
        self.StepLengthID = pb.addUserDebugParameter("Step Length", -0.1, 0.1, 0.0)
        self.YawRateId = pb.addUserDebugParameter("Yaw Rate", -2.0, 2.0, 0.0)
        self.LateralFractionId = pb.addUserDebugParameter("Lateral Fraction", -np.pi / 2.0, np.pi / 2.0, 0.0)
        self.StepVelocityId = pb.addUserDebugParameter("Step Velocity", 0.001, 3.0, 0.001)
        self.SwingPeriodId = pb.addUserDebugParameter("Swing Period", 0.1, 0.4, 0.2)
        self.ClearanceHeightId = pb.addUserDebugParameter("Clearance Height", 0.0, 0.1, 0.045)
        self.PenetrationDepthId = pb.addUserDebugParameter("Penetration Depth", 0.0, 0.05, 0.003)

        self.quadruped = quadruped

    def UserInput(self):
        quadruped_pos, _ = pb.getBasePositionAndOrientation(self.quadruped)
        pb.resetDebugVisualizerCamera(cameraDistance=self.cdist,
                                      cameraYaw=self.cyaw,
                                      cameraPitch=self.cpitch,
                                      cameraTargetPosition=quadruped_pos)
        keys = pb.getKeyboardEvents()

        # Camera control using keyboard keys
        if keys.get(100):  # D key
            self.cyaw += 1
        if keys.get(97):  # A key
            self.cyaw -= 1
        if keys.get(99):  # C key
            self.cpitch += 1
        if keys.get(102):  # F key
            self.cpitch -= 1
        if keys.get(122):  # Z key
            self.cdist += 0.01
        if keys.get(120):  # X key
            self.cdist -= 0.01
        if keys.get(27):  # ESC key
            pb.disconnect()
            sys.exit()

        # Read robot transform from GUI
        pos = np.array([
            pb.readUserDebugParameter(self.xId),
            pb.readUserDebugParameter(self.yId),
            pb.readUserDebugParameter(self.zId)
        ])
        orn = np.array([
            pb.readUserDebugParameter(self.rollId),
            pb.readUserDebugParameter(self.pitchId),
            pb.readUserDebugParameter(self.yawId)
        ])
        StepLength = pb.readUserDebugParameter(self.StepLengthID)
        YawRate = pb.readUserDebugParameter(self.YawRateId)
        LateralFraction = pb.readUserDebugParameter(self.LateralFractionId)
        StepVelocity = pb.readUserDebugParameter(self.StepVelocityId)
        ClearanceHeight = pb.readUserDebugParameter(self.ClearanceHeightId)
        PenetrationDepth = pb.readUserDebugParameter(self.PenetrationDepthId)
        SwingPeriod = pb.readUserDebugParameter(self.SwingPeriodId)

        return pos, orn, StepLength, LateralFraction, YawRate, StepVelocity, ClearanceHeight, PenetrationDepth, SwingPeriod
