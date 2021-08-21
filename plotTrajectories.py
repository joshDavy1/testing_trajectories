from roboticstoolbox import robot
import matplotlib.pyplot as plt
import numpy as np

def plotTrajectories(robot_def, total_trajectory, total_time):
        """ Plots arm trajectory """
        n = total_trajectory.shape[1]
        
        # Plot Joint Space
        plt.figure()
        plt.subplot(1, 3, 1)
        plt.title("Position")
        plt.plot(total_time, total_trajectory[0])
        plt.plot(total_time, total_trajectory[1])
        plt.plot(total_time, total_trajectory[2])
        plt.subplot(1, 3, 2)
        plt.title("Velocity")
        plt.plot(total_time[0:-1], np.diff(total_trajectory[0])/0.1)
        plt.plot(total_time[0:-1], np.diff(total_trajectory[1])/0.1)
        plt.plot(total_time[0:-1], np.diff(total_trajectory[2])/0.1)
        plt.ylim((-3, 3))
        plt.subplot(1, 3, 3)
        plt.title("Accleration")
        plt.plot(total_time[0:-2], np.diff(total_trajectory[0],2)/(0.1**2))
        plt.plot(total_time[0:-2], np.diff(total_trajectory[1],2)/(0.1**2))
        plt.plot(total_time[0:-2], np.diff(total_trajectory[2],2)/(0.1**2))
        plt.ylim((-5, 5))
        plt.suptitle("Joint Space")
        plt.show(block=False)

        # Convert to Cartesian
        cartesian_trajectory = np.empty((2, n))
        for i in range(n):
            cartesian_trajectory[:,i] = robot_def.forward_kinematics(total_trajectory[:,i]).T

        # Plot Cartesian
        plt.figure()
        plt.subplot(1, 3, 1)
        plt.title("Position")
        plt.plot(total_time, cartesian_trajectory[0])
        plt.plot(total_time, cartesian_trajectory[1])
        plt.subplot(1, 3, 2)
        plt.title("Velocity")
        plt.ylim(-5, 5)
        plt.plot(total_time[0:-1], np.diff(cartesian_trajectory[0])/0.1)
        plt.plot(total_time[0:-1], np.diff(cartesian_trajectory[1])/0.1)
        plt.subplot(1, 3, 3)
        plt.title("Accleration")
        plt.ylim(-5, 5)
        plt.plot(total_time[0:-2], np.diff(cartesian_trajectory[0],2)/(0.1**2))
        plt.plot(total_time[0:-2], np.diff(cartesian_trajectory[1],2)/(0.1**2))
        plt.suptitle("Cartesian Space")
        plt.show(block=False)

        # Plot XY positions of the arm
        plt.figure()
        plt.title("XY Plot")
        plt.axis('equal')
        plt.plot(cartesian_trajectory[0], cartesian_trajectory[1])        
        plt.show(block=False)
