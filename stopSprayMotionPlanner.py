from roboticstoolbox import robot
import numpy as np
import matplotlib.pyplot as plt
from polynomialTrajectory import PolynomialTrajectory, minimumPolynoialTrajectoryTime
from linearTrajectory import LinearTrajectory
from time import sleep

class stopSprayMotionPlanner:
    def __init__(self, robot_def, weeds, velocity, start_state_xy, spraying_time, stop_point):
        self.spraying_time = spraying_time
        self.weeds = weeds
         # Initial time is zero
        self.time = 0
        self.velocity = velocity
        self.velocity_traj  = [velocity]
        self.total_time = [0]
        current_pose = np.atleast_2d(robot_def.inverse_kinematics(start_state_xy['position'])).T
        self.total_trajectory = current_pose
        print(current_pose)
        self.position = 0
        # While still detected weeds
        while not self.weeds.size == 0:
            print(self.weeds)
            input()
            # Check all weeds
            in_range, weed_pose, weed_index = self.weed_in_range(robot_def)
            # If not in range move forward
            if not in_range:
                if self.velocity_traj[-1] == 0:
                    traj, v_traj, t = self.start(velocity, current_pose)
                    self.total_trajectory = np.append(self.total_trajectory, traj, axis=1)
                    self.total_time = np.append(self.total_time, t)
                    self.velocity_traj = np.append(self.velocity_traj, v_traj)

                self.total_trajectory = np.append(self.total_trajectory, current_pose, axis=1)
                self.total_time = np.append(self.total_time, self.time)
                self.time += 0.1
                self.velocity_traj = np.append(self.velocity_traj, self.velocity)
                self.position += -self.velocity*0.1
                v = np.array([[0],
                    [self.velocity]])
                if not self.weeds.size == 0:
                    self.weeds = self.weeds + v*0.1
                continue
            # If weed in range, stop
            if self.velocity_traj[-1] != 0:
                traj, v_traj, t = self.stop(velocity, current_pose)
                self.total_trajectory = np.append(self.total_trajectory, traj, axis=1)
                self.total_time = np.append(self.total_time, t)
                self.velocity_traj = np.append(self.velocity_traj, v_traj)
            # Generate movement to weed pose
            start_state_xy = {}
            start_state_xy['position'] = robot_def.forward_kinematics(current_pose).T[0]
            start_state_xy['velocity'] = np.array([0, 0])
            start_state_xy['accleration'] = np.array([0, 0])
            print(start_state_xy)
            weed_state_xy = {}
            weed_state_xy['position'] = self.weeds[:, weed_index]
            weed_state_xy['velocity'] = np.array([0, 0])
            weed_state_xy['accleration'] = np.array([0, 0])
            polyTrajectory, _ = minimumPolynoialTrajectoryTime(robot_def, start_state_xy, weed_state_xy, 0)
            # Add to trajectory
            traj, t = polyTrajectory.generate_trajectory()
            self.total_trajectory = np.append(self.total_trajectory, traj, axis=1)
            self.total_time = np.append(self.total_time, t + self.time)
            n = traj.shape[1]
            for i in range(n):
                self.velocity_traj = np.append(self.velocity_traj, 0)
                self.time += 0.1
            # Spray
            current_pose = np.atleast_2d(traj[:, -1]).T
            print(current_pose)
            traj, v_traj, t = self.spray(spraying_time, current_pose)
            self.total_trajectory = np.append(self.total_trajectory, traj, axis=1)
            self.total_time = np.append(self.total_time, t)
            self.velocity_traj = np.append(self.velocity_traj, v_traj)

            # Delete weed from the list
            self.weeds = np.delete(self.weeds, weed_index, 1)
            current_pose = np.atleast_2d(traj[:, -1]).T
        
        print(self.total_trajectory.shape)
        print(self.velocity_traj.shape)
        print(self.total_time.shape)


    def weed_in_range(self, robot_def):
        no_weeds = self.weeds.shape[1]
        for i in range(no_weeds):
            # Check if range
            weed_pose = self.weeds[:, i]
            weed_in_range = robot_def.check_limits_position(weed_pose)
            if weed_in_range:
                weed_index = i
                return True, weed_pose, weed_index
        return False, 0, 0

    def spray(self, spray_time, current_pose):
        v_traj = np.array([])
        traj = current_pose
        t = np.array([])
        step = int(spray_time/0.1)
        for i in range(step):
            self.time += 0.1
            v_traj = np.append(v_traj, 0)
            traj = np.append(traj, current_pose, axis=1)
            t = np.append(t, self.time)

        return traj, v_traj, t

    def stop(self, velocity, current_pose):
        v_traj = np.array([])
        traj = current_pose
        t = np.array([])
        step = int(-velocity/0.1)
        for i in range(step):
            v = np.array([[0],
                    [self.velocity+.1*i]])
            if not self.weeds.size == 0:
                self.weeds = self.weeds + v*0.1
            self.time += 0.1
            self.position += (-velocity-.1*i)*0.1
            v_traj = np.append(v_traj, velocity+0.1*i)
            traj = np.append(traj, current_pose, axis=1)
            t = np.append(t, self.time)

        return traj, v_traj, t

    def start(self, velocity, current_pose):
        v_traj = np.array([])
        traj = current_pose
        t = np.array([])
        step = int(-velocity/0.1)
        for i in range(step):
            v = np.array([[0],
                    [0-.1*i]])
            if not self.weeds.size == 0:
                self.weeds = self.weeds + v*0.1
            self.time += 0.1
            self.position += (-velocity-.1*i)*0.1
            v_traj = np.append(v_traj, 0-0.1*i)
            traj = np.append(traj, current_pose, axis=1)
            t = np.append(t, self.time)

        return traj, v_traj, t
    
    def getTrajectory(self):
        return self.total_trajectory, self.velocity_traj, self.total_time