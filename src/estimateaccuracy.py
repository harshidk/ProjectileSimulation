import sim
import numpy as np
import matplotlib.pyplot as plt
import readmap as map
import random

x_bnds = (0, 3.685794)
y_bnds = (0, 8.069326)
launch_height = 0.5

pos = map.getNumpyArrayPositions()
states = map.getOptimalStates()

random_pos = np.array([random.uniform(0, 3.685794), random.uniform(0, 8.069326)])

min = 100000
index = 0
for i in range(len(pos)):
    dist = np.linalg.norm(pos[i] - random_pos)
    if dist < min:
        min = dist
        index = i

poses, vel = sim.simulateShotDragLift(states[index][0][0], [random_pos[0], random_pos[1], sim.calculateRequiredHeading(sim.TARGET_POSE, [random_pos[0], random_pos[1], 0])], states[index][0][1], launch_height)
sim.meshcatVisualizeShot(poses, sim.TARGET_POSE)
# sim.graphSimulatedShot(poses, sim.TARGET_POSE)