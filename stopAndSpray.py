from weedGeneration import generate_weed_locations, generate_realistic_weed_locations
import numpy as np
import matplotlib.pyplot as plt
from robot3D import ThreeJointRobot
from stopSprayMotionPlanner import stopSprayMotionPlanner
from plotTrajectories import plotTrajectories


class Main:
    def __init__(self):
        self.velocity = -1
        self.spraying_time = 1
        #self.weeds = generate_weed_locations(-3, 3, 3, 20, 1, seed=412)
        self.weeds = generate_realistic_weed_locations(-3, 3, 3, 6, 0.3, seed=412)
        self.original_weeds = self.weeds.copy()

    def main(self):
        """ Main """
        plt.close('all')
        # Robot definition
        robot_def = ThreeJointRobot()
        # Initial state of the end effector
        start_state_xy = {}
        start_state_xy['position'] = robot_def.forward_kinematics(robot_def.joint_angles)
        start_state_xy['velocity'] = np.array([0, 0])
        start_state_xy['accleration'] = np.array([0, 0])
        # Generate Trajectory
        motionPlanner = stopSprayMotionPlanner(robot_def, self.weeds, self.velocity, start_state_xy, self.spraying_time, 10)
        total_trajectory, velocity_traj, total_time = motionPlanner.getTrajectory()
        print(total_trajectory, velocity_traj, total_time)
        # Visualisation
        robot_def.draw_robot(total_trajectory, self.original_weeds, velocity_traj)
        # Plot Trajectories
        plotTrajectories(total_trajectory, total_time)
    
if __name__ == "__main__":
    m = Main()
    m.main()