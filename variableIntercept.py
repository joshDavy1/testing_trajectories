from weedGeneration import generate_weed_locations
import numpy as np
import matplotlib.pyplot as plt
from polynomialTrajectory import PolynomialTrajectory
from linearTrajectory import LinearTrajectory
from robot3D import ThreeJointRobot
from variableInterceptMotionPlanner import variableInterceptMotionPlanner
from plotTrajectories import plotTrajectories
import time


class Main:
    def __init__(self):
        self.velocity = -0.5

        self.spraying_time = 0.5
        self.weeds = generate_weed_locations(-3, 3, 5, 20, 0.3)
        self.original_weeds = self.weeds.copy()

    def main(self):
        """ Main """
        plt.close('all')
        # Robot definition
        robot_def = ThreeJointRobot()
        print(self.weeds)
        # Initial state of the end effector
        start_state_xy = {}
        start_state_xy['position'] = robot_def.forward_kinematics(robot_def.joint_angles)
        start_state_xy['velocity'] = np.array([0, 0])
        start_state_xy['accleration'] = np.array([0, 0])
        # Generate Trajectory
        start_time = time.time()
        motionPlanner = variableInterceptMotionPlanner(robot_def, self.weeds, self.velocity, start_state_xy, self.spraying_time, 10)
        total_trajectory, velocity_traj,total_time = motionPlanner.getTrajectory()
        end_time = time.time()
        print(velocity_traj)
        print(motionPlanner.weighted_avgs)
        #print(np.round(np.diff(total_time),4))
        # Plot Joint Trajectories
        plotTrajectories(robot_def, total_trajectory, total_time)
        # Print Weeds Sprayed
        print("Sprayed: ",motionPlanner.getWeedsSprayed(),"/", self.weeds.shape[1], sep='')
        print("Generation Time (ms):", (end_time-start_time)/1000)
        print("Average Velocity:", np.mean(velocity_traj))
        print("Runtime:", total_time[-1])
        print(velocity_traj)
        # Draw
        draw = input("Draw? Y/N \n")
        if draw == 'y' or draw == 'Y':
            start_time = time.time()
            robot_def.draw_robot(total_trajectory, self.original_weeds, velocity_traj)
            end_time = time.time()
            print("Execution Time:", (end_time-start_time)/1000)
            input()
        # Save
        np.save("trajectory", total_trajectory)
      

if __name__ == "__main__":
    m = Main()
    m.main()