import numpy as np

def weightedAverage(positions):
    positions = np.flip(positions)
    N = len(positions)
    weights = np.zeros_like(positions, dtype=np.float16)
    sum = 0
    for n in range(N):
        weights[n] = (N-n)/N
        sum += weights[n]
    return np.sum(positions*weights/sum)
