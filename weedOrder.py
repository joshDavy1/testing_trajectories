import numpy as np
import matplotlib.pyplot as plt
from weedGeneration import *


def orderWeeds(weeds):
        # Sort by y axis
        order = np.argsort(weeds[1, :])
        sortedWeeds = np.zeros_like(weeds)
        for i in range(len(order)):
            sortedWeeds[:, i] = weeds[:, order[i]]
        return sortedWeeds

def orderWeeds2(weeds, alpha = 0.8):
    n = weeds.shape[1]
    weights = np.zeros(n)
    for i in range(n):
        weights[i] = alpha*weeds[1, i] + (1-alpha)*weeds[0, i]
    order = np.argsort(weights)
    print(order)
    sortedWeeds = np.zeros_like(weeds)
    for i in range(n):
        sortedWeeds[:, i] = weeds[:, order[i]]
    return sortedWeeds


def orderWeeds3(weeds, alpha=0.95):
    min_index = np.argmin(weeds[1, :])
    current_weed = weeds[:, min_index]
    weeds = np.delete(weeds, min_index, axis=1)
    new_weeds = np.atleast_2d(current_weed).T
    while weeds.shape[1] > 0:
        # Find Closest Weed to current weed
        min_distance = np.inf
        closest_weed_index  = None
        for i in range(weeds.shape[1]):
            weed = weeds[:, i]
            sqr_dist = (1-alpha)*(current_weed[0]-weed[0])**2 + alpha*(current_weed[1]-weed[1])**2
            if sqr_dist < min_distance:
                min_distance = sqr_dist
                closest_weed_index = i
        print(closest_weed_index)
        closest_weed = weeds[:, closest_weed_index]
        print(closest_weed)
        # Add closest weed to array
        new_weeds = np.append(new_weeds, np.atleast_2d(closest_weed).T, axis=1)
        # Remove from original array
        weeds = np.delete(weeds, closest_weed_index, axis=1)
        # Make this weed the current weed
        current_weed = closest_weed
    print(new_weeds)
    return new_weeds


def plot_weed_order(weeds):
    plt.title("Spray Order")
    plt.plot(weeds[0, :], weeds[1, :], 'x')
    n = weeds.shape[1]
    for i in range(n):
        x = weeds[0, i]
        y = weeds[1, i]
        if i > 0:
            x1 = weeds[0, i-1]
            y1 = weeds[1, i-1]
            plt.plot((x, x1), (y,y1),)
        plt.text(x+0.03, y+0.03, str(i))
    plt.axis('scaled')
    plt.show(block=False)
    input()


weeds  = generate_weed_locations(-3, 3, 0, 10, 0.5)
weeds = orderWeeds3(weeds)
plot_weed_order(weeds)