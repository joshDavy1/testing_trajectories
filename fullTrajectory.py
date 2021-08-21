import numpy as np
from polynomialTrajectory import PolynomialTrajectory
from linearTrajectory import LinearTrajectory

class fullTrajectory:
    def __init__(self, robot_def, start_state_xy, weed_state_xy, spraying_time, t0, t_spray=None):
        # Round t0
        #t0 = np.round(t0, 4)
        # Check start state in range
        if not robot_def.check_limits_position(start_state_xy['position']):
            self.result = {}
            self.result['failed'] = True
            self.result['trajectory'] = None
            self.result['time'] = None
            self.result['final_state_xy'] = None
            return
        # Initial Trajectory
        total_time = []   #Everything here is done relative to t=0, t0 added at end
        total_trajectory =  np.empty((3,1))
        # Poly Trajectory
        if t_spray == None:
            polyTrajectory, t_spray, failed = self.minimumPolynoialTrajectoryTime(robot_def, start_state_xy, weed_state_xy, 0)
        else:
            polyTrajectory = PolynomialTrajectory(robot_def, start_state_xy, weed_state_xy, 0, t_spray-t0, dt=0.1)
            failed =  not np.all(polyTrajectory.check_limits())
            t_spray = t_spray-t0
        if failed:
            self.result = {}
            self.result['failed'] = True
            self.result['trajectory'] = None
            self.result['time'] = None
            self.result['final_state_xy'] = None
            return
        # Add to trajectory
        traj, t = polyTrajectory.generate_trajectory()
        total_trajectory = traj
        #print(t+t0)
        total_time = np.append(total_time, t)
        # Linear Trajectory 
        tf = np.round(t_spray + spraying_time, 1)
        linTrajectory = LinearTrajectory(robot_def, weed_state_xy, t_spray, tf)
        # Add to trajectory
        traj, t = linTrajectory.generate_trajectory()
        total_trajectory = np.append(total_trajectory, traj, axis=1)
        total_time = np.append(total_time, t)
        # Final State
        final_state_xy = {}
        final_state_xy["position"] = robot_def.forward_kinematics(total_trajectory[:, -1])
        final_state_xy["velocity"] = weed_state_xy["velocity"]
        final_state_xy["accleration"] = weed_state_xy["accleration"]
        # Result
        self.result = {}
        self.result['failed'] = not np.all(linTrajectory.check_limits())
        self.result['trajectory'] = total_trajectory  # Add offset
        self.result['time'] = total_time + t0
        self.result['final_state_xy'] = final_state_xy
        #print(self.result['time'])
        print(t0, t0+t_spray, t0+tf)
        #input()

    
    def minimumPolynoialTrajectoryTime(self, robot_def, start_state_xy, goal_state_xy, dt=0.1, max_tries=50):
        tf = 0.5
        tries = 0
        failed = True
        while failed and tries < max_tries:
            tries += 1
            x = PolynomialTrajectory(robot_def, start_state_xy, goal_state_xy, 0, tf, dt=0.1)
            if  np.all(x.check_limits()):
                return x, tf, False
            else:
                tf += 0.1
        return None, None, True

    def getResult(self):
        return self.result