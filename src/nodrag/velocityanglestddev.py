import numpy as np
from .. import sim
import matplotlib.pyplot as plt

P_x = 6
P_y = 6
launch_height = 0.5
epilson = 0.05
vel_st_dev = 0.5
angle_st_dev = 1.5

v_min = sim.calculateMinimumVelocity(sim.TARGET_POSE, [P_x, P_y], launch_height) + epilson
v_samples = (sim.MAX_LAUNCH_SPEED - v_min)/sim.DELTA_V
# s = np.random.normal(v_min, vel_st_dev, 1000)

velocities = []
angles = []
headings = []
distances = []
percents = []

for i in range(int(v_samples)):
    vel = v_min + sim.DELTA_V*i
    heading, angle, dist, sqrt_term, y_d = sim.calculateOptimalTrajectoriesNoDrag(sim.TARGET_POSE, [P_x, P_y, launch_height], vel)
    velocities.append(vel)
    angles.append(angle)
    headings.append(heading)

print(len(velocities))

for i in range(len(velocities)):
    print("step: " + str(i))
    s = np.random.normal(velocities[i], vel_st_dev, 1000)
    a = np.random.normal(angles[i], angle_st_dev, 1000)
    distances = []
    for j in range(len(s)):
        poses, vel = sim.simulateShotNoDrag(s[j], [P_x, P_y, headings[i]], a[j], launch_height)
        dist, x, y = sim.findClosestPositionToTarget(poses, np.array(sim.TARGET_POSE))
        distances.append(dist)
    count = 0
    for d in distances:
        if d > sim.TOLERANCE:
            count+=1
    percents.append(count/len(distances) * 100)

min = 100
min_index = 0
for i in range(len(percents)):
    if percents[i] < min:
        min = percents[i]
        min_index = i

print("Optimal velocity: " + str(velocities[min_index]))

plt.scatter(np.array(velocities), np.array(percents))
plt.xlabel("Velocities")
plt.ylabel("Percent Missed")
plt.show()