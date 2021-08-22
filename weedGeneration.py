import numpy as np
import matplotlib.pyplot as plt

def generate_weed_locations(x0, x1, y0, y1, density, seed=1):
    np.random.seed(seed)
    number_of_weeds =  int(np.round(np.abs(x1-x0)*np.abs(y1-y0)*density))
    weed_locations = np.random.rand(2, number_of_weeds)
    weed_locations[0, :] = x0+weed_locations[0, :]*(x1-x0)
    weed_locations[1, :] = y0+weed_locations[1, :]*(y1-y0)
    return weed_locations

def generate_realistic_weed_locations(x0, x1, y0, y1, density, avg_cluster_size = 3, seed=1):
    np.random.seed(seed)
    number_of_weeds =  int(np.round(np.abs(x1-x0)*np.abs(y1-y0)*density))
    weeds_placed = 0
    weed_locations = np.array([[3],[4]])
    while weeds_placed < number_of_weeds:
        cluster_size = int(np.random.normal(avg_cluster_size, 1))
        centre_x = np.random.randint(x0, x1)
        centre_y = np.random.randint(y0, y1)
        for i in range(cluster_size):
            x = np.random.normal(centre_x, 0.2)
            y = np.random.normal(centre_y, 0.2)
            weed_locations = np.append(weed_locations, np.array([[x], [y]]), axis=1)
            weeds_placed += 1
    return weed_locations

if __name__ == "__main__":
    weed_locations = generate_weed_locations(0, 3, 0, 3, 3)
    plt.title("Uniformly Distributed Weeds")
    plt.plot(weed_locations[0, :], weed_locations[1, :], 'bx')
    plt.show(block=False)
    plt.figure()
    weed_locations = generate_realistic_weed_locations(0, 3, 0, 3, 3)
    plt.title("Realistically Distributed Weeds")
    plt.plot(weed_locations[0, :], weed_locations[1, :], 'rx')
    plt.show(block=False)
    input()
