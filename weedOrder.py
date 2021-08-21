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

def clusterWeeds(weeds):
    pass



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
weeds = orderWeeds2(weeds)
plot_weed_order(weeds)