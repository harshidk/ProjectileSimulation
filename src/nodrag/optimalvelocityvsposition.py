import numpy as np
from .. import sim
import matplotlib.pyplot as plt

launch_height = 0.5
epilson = 0.05
vel_st_dev = 0.5
angle_st_dev = 1.5

SAMPLES = 1000
NUM_POSITIONS = 50

x_positions = []
y_positions = []
optimal_velocities = []

for k in range(NUM_POSITIONS):
    P_x = sim.TARGET_POSE[0] * k / NUM_POSITIONS
    P_y = 2 * sim.TARGET_POSE[1] * k / NUM_POSITIONS
    x_positions.append(P_x)
    y_positions.append(P_y)

    v_min = sim.calculateMinimumVelocity(sim.TARGET_POSE, [P_x, P_y], launch_height) + epilson
    v_samples = (sim.MAX_LAUNCH_SPEED - v_min)/sim.DELTA_V
    velocities = []
    angles = []
    headings = []
    distances = []
    percents = []

    for f in range(int(v_samples)):
        vel = v_min + sim.DELTA_V*f
        heading, angle, dist, sqrt_term, y_d = sim.calculateOptimalTrajectoriesNoDrag(sim.TARGET_POSE, [P_x, P_y, launch_height], vel)
        velocities.append(vel)
        angles.append(angle)
        headings.append(heading)

    for i in range(len(velocities)):
        s = np.random.normal(velocities[i], vel_st_dev, SAMPLES)
        a = np.random.normal(angles[i], angle_st_dev, SAMPLES)
        distances = []
        for j in range(len(s)):
            print("         Sample Steps: {" + str(j)+", "+str(i)+", "+str(k)+"}" + " / {" + str(SAMPLES) + ", " +str(len(velocities)) + ", "+str(NUM_POSITIONS)+"}")
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
    for q in range(len(percents)):
        if percents[q] < min:
            min = percents[q]
            min_index = q

    optimal_velocity = velocities[min_index]
    optimal_velocities.append(optimal_velocity)

fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
ax.plot_surface(np.array(x_positions), np.array(y_positions), np.array(optimal_velocities))
ax.set_xlabel('X Position (m)')
ax.set_ylabel('Y Position (m)')
ax.set_zlabel('Optimal Velocity (m/s)')
plt.show()