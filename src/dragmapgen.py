import sim
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from tqdm import tqdm
import json

launch_height = 0.5
std_dev_vel = 0.5
std_dev_angle = 1.5

angle_bnds = (45, 75)
vel_bnds = (1, 20)
x_bnds = (0, 3.685794)
y_bnds = (0, 8.069326)

VEL_SAMPLES = 20
ANGLE_SAMPLES = 20
STDDEV_SAMPLES = 30
POSITION_SAMPLES = 60

MAX_HEIGHT = 3.6576

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
            if dist < sim.TOLERANCE and sim.getMaxHeight(poses) < MAX_HEIGHT:
                vels.append(velocities)
                angs.append(angle)
                dists.append(dist)

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

    if len(percents) != 0:
        max = 0
        max_index = 0
        for q in range(len(percents)):
            if percents[q] > max:
                max = percents[q]
                max_index = q

        best_shot = [vels[max_index], angs[max_index]]
        return best_shot, heading, percents[max_index]
    else:
        # print("No valid shots found at position: " + str(P_x) + ", " + str(P_y))
        return [0,0], heading, 100.0

def findBestShotAverageVelocityNoSOTM(P_x, P_y):
    vels = []
    angs = []
    dists = []
    avg_velocities = []
    heading = sim.calculateRequiredHeading(sim.TARGET_POSE, [P_x, P_y, 0])

    for i in range(VEL_SAMPLES):
        velocities = float(((vel_bnds[1]- vel_bnds[0])/VEL_SAMPLES)*i + vel_bnds[0])
        for j in range(ANGLE_SAMPLES):
            angle = float(((angle_bnds[1] - angle_bnds[0])/ANGLE_SAMPLES)*j + angle_bnds[0])
            headings = sim.calculateRequiredHeading(sim.TARGET_POSE, [P_x, P_y, 0])
            poses, e = sim.simulateShotDrag(velocities, [P_x, P_y, headings], angle, launch_height)
            avg_vel = sim.getAveragePathVelocity(e)
            dist, x, y = sim.findClosestZToTarget(poses, np.array(sim.TARGET_POSE))
            if dist < sim.TOLERANCE and sim.getMaxHeight(poses) < MAX_HEIGHT:
                vels.append(velocities)
                angs.append(angle)
                dists.append(dist)
                avg_velocities.append(avg_vel)
    if len(vels) != 0:
        m = 0
        max_index = 0
        for k in range(len(vels)):
            if avg_velocities[k] > m:
                m = avg_velocities[k]
                max_index = k
        return [vels[max_index], angs[max_index]], heading, m
    else:
        return [0,0], heading, 100.0

def convertDataToDictionary(positions, shots, headings, probs):
    dict = {}
    for i in range(len(positions)):
        dict[f"{positions[i][0]},{positions[i][1]}"] = [shots[i], headings[i], probs[i]]
    return dict

P_x = 0
P_y = 0

positions = [] # [[P_x, P_y], [P_x, P_y]]
shots = [] # [[vel, ang], [vel, ang]]
headings = [] #[heading, heading]
probs = [] # [percent, percent]

for o in tqdm(range(POSITION_SAMPLES)):
    P_x = ((x_bnds[1] - x_bnds[0])/POSITION_SAMPLES)*o + x_bnds[0]
    for e in tqdm(range(POSITION_SAMPLES)):
        P_y = ((y_bnds[1] - y_bnds[0])/POSITION_SAMPLES)*e + y_bnds[0]
        p = (P_x, P_y)
        positions.append(p)
        shot, heading, prob = findBestShotAverageVelocityNoSOTM(P_x, P_y)
        shots.append(shot)
        headings.append(heading)
        probs.append(prob)

dictionary = convertDataToDictionary(positions, shots, headings, probs)

with open("data.json", "w") as json_file:
    json.dump(dictionary, json_file, indent=4)


#ps, vs = sim.simulateShotDrag(shot[0], [2, 2, heading], shot[1], launch_height)
#print(prob)
#sim.meshcatVisualizeShot(np.array(ps), sim.TARGET_POSE)

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