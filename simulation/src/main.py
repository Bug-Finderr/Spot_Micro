import numpy as np
import copy
import sys
import os
import argparse

sys.path.append('../../')

from spot.gym_envs.Bezier_env import spotBezierEnv
from spot.util.gui import gui
from spot.kinematics.SpotKinematics import SpotModel
from spot.gait_generators.Bezier import BezierGait
from spot.env_randomizer import SpotEnvRandomizer
from spot.OpenLoopSM.OpenLoop import BezierStepper  # Testing

# ARGUMENTS
descr = "Spot Mini Mini Environment Tester (No Joystick)."
parser = argparse.ArgumentParser(description=descr)
parser.add_argument("-hf",
                    "--HeightField",
                    help="Use HeightField",
                    action='store_true')
parser.add_argument("-r",
                    "--DebugRack",
                    help="Put Spot on an Elevated Rack",
                    action='store_true')
parser.add_argument("-p",
                    "--DebugPath",
                    help="Draw Spot's Foot Path",
                    action='store_true')
parser.add_argument("-ay",
                    "--AutoYaw",
                    help="Automatically Adjust Spot's Yaw",
                    action='store_true')
parser.add_argument("-ar",
                    "--AutoReset",
                    help="Automatically Reset Environment When Spot Falls",
                    action='store_true')
parser.add_argument("-dr",
                    "--DontRandomize",
                    help="Do NOT Randomize State and Environment.",
                    action='store_true')
ARGS = parser.parse_args()

def main():
    """ The main() function. """
    
    print("STARTING SPOT TEST ENV")
    seed = 0
    max_timesteps = 4e6

    # Find absolute path to this file
    my_path = os.path.abspath(os.path.dirname(__file__))
    results_path = os.path.join(my_path, "../results")
    models_path = os.path.join(my_path, "../models")

    # Create results and models directories if they don't exist
    if not os.path.exists(results_path):
        os.makedirs(results_path)

    if not os.path.exists(models_path):
        os.makedirs(models_path)

    # Set configuration based on parsed arguments
    on_rack = ARGS.DebugRack
    draw_foot_path = ARGS.DebugPath
    height_field = ARGS.HeightField

    # Set environment randomizer
    env_randomizer = None if ARGS.DontRandomize else SpotEnvRandomizer()

    # Initialize environment
    env = spotBezierEnv(render=True,
                        on_rack=on_rack,
                        height_field=height_field,
                        draw_foot_path=draw_foot_path,
                        env_randomizer=env_randomizer)

    # Set seeds for reproducibility
    env.seed(seed)
    np.random.seed(seed)

    state_dim = env.observation_space.shape[0]
    print(f"STATE DIM: {state_dim}")
    action_dim = env.action_space.shape[0]
    print(f"ACTION DIM: {action_dim}")
    max_action = float(env.action_space.high[0])

    state = env.reset()

    # Initialize GUI
    g_u_i = gui(env.spot.quadruped)

    # Initialize SpotModel and Bezier Gait
    spot = SpotModel()
    T_bf0 = spot.WorldToFoot
    T_bf = copy.deepcopy(T_bf0)

    bzg = BezierGait(dt=env._time_step)
    bz_step = BezierStepper(dt=env._time_step, mode=0)

    action = env.action_space.sample()

    # Data for plotting
    FL_phases = []
    FR_phases = []
    BL_phases = []
    BR_phases = []
    FL_Elbow = []

    yaw = 0.0

    print("STARTED SPOT TEST ENV")
    t = 0
    while t < int(max_timesteps):
        bz_step.ramp_up()

        # Get state from Bezier stepper
        pos, orn, StepLength, LateralFraction, YawRate, StepVelocity, ClearanceHeight, PenetrationDepth = bz_step.StateMachine()

        # Get user input from GUI
        pos, orn, StepLength, LateralFraction, YawRate, StepVelocity, ClearanceHeight, PenetrationDepth, SwingPeriod = g_u_i.UserInput()

        # Update Swing Period
        bzg.Tswing = SwingPeriod

        # Adjust yaw rate automatically if enabled
        yaw = env.return_yaw()
        P_yaw = 5.0
        if ARGS.AutoYaw:
            YawRate += -yaw * P_yaw

        # Update step parameters
        bz_step.StepLength = StepLength
        bz_step.LateralFraction = LateralFraction
        bz_step.YawRate = YawRate
        bz_step.StepVelocity = StepVelocity

        # Get contacts and leg phases
        contacts = state[-4:]
        FL_phases.append(env.spot.LegPhases[0])
        FR_phases.append(env.spot.LegPhases[1])
        BL_phases.append(env.spot.LegPhases[2])
        BR_phases.append(env.spot.LegPhases[3])

        # Generate desired foot poses using Bezier trajectory
        T_bf = bzg.GenerateTrajectory(StepLength, LateralFraction, YawRate,
                                      StepVelocity, T_bf0, T_bf,
                                      ClearanceHeight, PenetrationDepth,
                                      contacts)

        # Compute joint angles using inverse kinematics
        joint_angles = spot.IK(orn, pos, T_bf)
        FL_Elbow.append(np.degrees(joint_angles[0][-1]))

        # Send joint angles to environment
        env.pass_joint_angles(joint_angles.reshape(-1))

        # Get external observations
        env.spot.GetExternalObservations(bzg, bz_step)

        # Perform environment step
        state, reward, done, _ = env.step(action)

        if done:
            print("DONE")
            if ARGS.AutoReset:
                env.reset()

        t += 1

    env.close()
    print(joint_angles)

if __name__ == '__main__':
    main()
