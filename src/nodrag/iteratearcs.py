from .. import sim
import matplotlib.pyplot as plt
import numpy as np

P_x = 6
P_y = 6
launch_height = 0.5
epilson = 0.05

angles = []
velocities = []
headings = []
v_min = sim.calculateMinimumVelocity(sim.TARGET_POSE, [P_x, P_y], launch_height) + epilson
v_samples = (sim.MAX_LAUNCH_SPEED - v_min)/sim.DELTA_V
for i in range(int(v_samples)):
    vel = v_min + sim.DELTA_V * i
    heading, angle, dist, sqrt_term, y_d = sim.calculateOptimalTrajectoriesNoDrag(sim.TARGET_POSE, [P_x, P_y, launch_height], vel)
    velocities.append(vel)
    angles.append(angle)
    headings.append(heading)

max_heights = []
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
ax.scatter(P_x, P_y, launch_height, s=50, color='g', label='Start') # pyright: ignore[reportArgumentType]
ax.scatter(sim.TARGET_POSE[0], sim.TARGET_POSE[1], sim.TARGET_POSE[2], s=50, color='r', label='Target') # pyright: ignore[reportArgumentType]
for i in range(int(v_samples)):
    poses, vels = sim.simulateShotNoDrag(velocities[i], [P_x, P_y, headings[i]], angles[i], launch_height)
    poses_np = np.array(poses)
    ax.plot(poses_np[:,0], poses_np[:,1], poses_np[:,2])
    max_h = sim.getMaxHeight(poses)
    max_heights.append(max_h)

print(int(v_samples))
print(np.array(max_heights).max())

ax.set_xlabel('X Position (m)')
ax.set_ylabel('Y Position (m)')
ax.set_zlabel('Z Position (m)')
ax.legend()
plt.show()