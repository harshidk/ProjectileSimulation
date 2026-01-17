import sim
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

launch_height = 0.5
std_dev_vel = 0.5
std_dev_angle = 1.5

angle_bnds = (45, 75)
vel_bnds = (1, 20)
x_bnds = (0, 3.685794)
y_bnds = (0, 8.069326)

VEL_SAMPLES = 10
ANGLE_SAMPLES = 10
STDDEV_SAMPLES = 30

def findBestShotNoSOTM(P_x, P_y):
    vels = []
    angs = []
    dists = []
    heading = sim.calculateRequiredHeading(sim.TARGET_POSE, [P_x, P_y, 0])

    for i in range(VEL_SAMPLES):
        velocities = float(((vel_bnds[1]- vel_bnds[0])/VEL_SAMPLES)*i + vel_bnds[0])
        for j in range(ANGLE_SAMPLES):
            angle = float(((angle_bnds[1] - angle_bnds[0])/ANGLE_SAMPLES)*j + angle_bnds[0])
            headings = sim.calculateRequiredHeading(sim.TARGET_POSE, [P_x, P_y, 0])
            poses, vel = sim.simulateShotDrag(velocities, [P_x, P_y, headings], angle, launch_height)
            dist, x, y = sim.findClosestZToTarget(poses, np.array(sim.TARGET_POSE))
            if dist < sim.TOLERANCE:
                vels.append(velocities)
                angs.append(angle)
                dists.append(dist)
            print(str(j + ANGLE_SAMPLES*i + 1) + "/" + str(VEL_SAMPLES * ANGLE_SAMPLES))

    percents = []

    for k in range(len(vels)):
        s = np.random.normal(vels[k], std_dev_vel, STDDEV_SAMPLES)
        a = np.random.normal(angs[k], std_dev_angle, STDDEV_SAMPLES)
        count = 0
        for l in range(len(s)):
            poses, fake = sim.simulateShotDrag(s[l], [P_x, P_y, heading], a[l], launch_height)
            more_dist, x, y = sim.findClosestZToTarget(poses, np.array(sim.TARGET_POSE))
            if(more_dist < sim.TOLERANCE):
                count+=1
        percents.append((count / STDDEV_SAMPLES) * 100)

    max = 0
    max_index = 0
    for q in range(len(percents)):
        if percents[q] > max:
            max = percents[q]
            max_index = q

    best_shot = [vels[max_index], angs[max_index]]
    return best_shot, heading, percents[max_index]

shot, heading, prob = findBestShotNoSOTM(2, 2)
ps, vs = sim.simulateShotDrag(shot[0], [2, 2, heading], shot[1], launch_height)
print(prob)
sim.meshcatVisualizeShot(np.array(ps), sim.TARGET_POSE)

"""
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
sc = ax.scatter(vels, angs, dists, c=colors, cmap='gist_rainbow', marker='o') # type: ignore
fig.colorbar(sc, shrink=0.5, aspect=5, label='Percents')
ax.set_xlabel('Velocity')
ax.set_ylabel('Angle')
ax.set_zlabel('Distance')
plt.show()
"""