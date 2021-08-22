from typing import Iterable
from roboticstoolbox import robot
import numpy as np
import matplotlib.pyplot as plt
from fullTrajectory import fullTrajectory
from time import sleep


# def roundDown(x, decimalPlaces):
#     y = 10**(-decimalPlaces)
#     return np.round(np.floor(x/y)*y, decimalPlaces)

# def roundUp(x, decimalPlaces):
#     y = 10**(-decimalPlaces)
#     return np.round(np.ceil(x/y)*y, decimalPlaces)

class variableInterceptMotionPlanner:
    def __init__(self, robot_def, weeds, velocity, start_state_xy, spraying_time, stop_point):
        self.weeds_sprayed = 0
        self.velocity = velocity
        self.weeds = self.orderWeeds(weeds)
        self.spraying_time  = spraying_time
        # Initial time is zero
        time = 0
        self.total_trajectory = np.array([])
        self.total_time = np.array([])
        # While still detected weeds
        while not self.weeds.size == 0:
            start_time = time
            # Select first weed
            weed_pose = self.weeds[:, 0].T
            print(weed_pose)
            result = self.interceptAndSpray(robot_def, start_state_xy, weed_pose, spraying_time, time, velocity)
            # If trajectory generated
            if not result['failed']:
                    self.weeds_sprayed += 1
                    trajectory = result['trajectory']
                    trajectory_time = result['time']
                    # Append to full trajectory array
                    if self.total_trajectory.size == 0:
                        self.total_trajectory = trajectory
                        self.total_time = trajectory_time
                    else:
                        self.total_trajectory = np.concatenate((self.total_trajectory, trajectory), axis=1)
                        self.total_time = np.concatenate((self.total_time, trajectory_time), axis=0)
                    # Execute
                    time = trajectory_time[-1] + 0.1
                    self.update(trajectory_time.shape[0])
                    print(self.total_time)
                    # Next start state is last position
                    start_state_xy = result['final_state_xy']
                    start_time =  time
                    print("START", start_time)
            else:
                print("Trajectory Generation Failed")
            # Remove weed from list
            self.weeds = np.delete(self.weeds, 0, 1)

    def interceptAndSpray(self, robot_def, start_state_xy, weed_pose, spraying_time, time, velocity, dt=0.1, max_iterations=300):
        print("Intercept and Spray")
        #input()
        # Find time when weed first within arm reach
        first_intercept_time = time
        within_arm_range = False
        iterations = 0
        while not within_arm_range:
            iterations += 1
            if iterations > 1000:
                print("Failed as does not enter arm range")
                result = {}
                result['failed'] = True
                return result

            within_arm_range = robot_def.check_limits_position((weed_pose[0],
                                                            weed_pose[1]+velocity*(first_intercept_time-time)))
            #print((weed_pose[0], weed_pose[1]+velocity*(first_intercept_time-time)))
            first_intercept_time += dt

        # Find time to intercept
        can_reach_in_time = False
        within_arm_range = False
        t_intercept = first_intercept_time + dt
        iterations = 0
        while iterations < max_iterations:
            print("INTERATIONS:", iterations)
            print(weed_pose[1]+velocity*(t_intercept-time))
            iterations += 1
            # Calculate trajectory
            weed_state_xy = {}
            weed_state_xy['position'] = np.array([weed_pose[0], weed_pose[1]+velocity*(t_intercept-time)])
            weed_state_xy['velocity'] = np.array([0, velocity])
            weed_state_xy['accleration'] = np.array([0, 0])                            
            fullTraj = fullTrajectory(robot_def, start_state_xy, weed_state_xy, spraying_time, time, t_intercept)
            result = fullTraj.getResult()
            if not result['failed']:
                return result
            t_intercept += dt

            if not robot_def.check_limits_position(weed_state_xy['position']):
                print("Weed no longer feasibly sprayed")
                break
        # Then no valid trajectory has been found
        result = {}
        result['failed'] = True
        return result



            
        

        # Check if can spray within that time
        # If not increase
        # If endpoint gets out of range return failed


    def orderWeeds(self, weeds):
        # Sort by y axis
        order = np.argsort(weeds[1, :])
        sortedWeeds = np.zeros_like(weeds)
        for i in range(len(order)):
            sortedWeeds[:, i] = weeds[:, order[i]]
        return sortedWeeds

    def orderWeedsIntelligent(self, weeds, alpha = 0.7):
        n = weeds.shape[1]
        weights = np.zeros(n)
        for i in range(n):
            weights[i] = alpha*weeds[1, :] + (1-alpha)*weeds[0, :]
        order = np.argsort(weeds)
        sortedWeeds = np.zeros_like(weeds)
        for i in range(n):
            sortedWeeds[:, i] = weeds[:, order[i]]
        return sortedWeeds



    def update(self, time, weed = np.empty([])):
        """ Update weed list based on velocity 
        and if any new weeds are detected"""
        if not weed.size == 1:
            self.weeds = np.concatenate((self.weeds, weed), axis=1)
        # Update weed positions
        v = np.array([[0],
                    [self.velocity]])
        if not self.weeds.size == 0:
            self.weeds = self.weeds + v*time*0.1
    
    def getTrajectory(self):
        #print(self.total_trajectory)
        self.velocity_traj = np.zeros(self.total_trajectory.shape[1])
        self.velocity_traj += self.velocity
        return self.total_trajectory, self.velocity_traj, self.total_time

    def getWeedsSprayed(self):
        return self.weeds_sprayed