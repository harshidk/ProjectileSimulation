import numpy as np
import matplotlib.pyplot as plt
import time

import meshcat
import meshcat.geometry as g
import meshcat.transformations as tf

G = 9.81
RHO = 1.204
CD = 0.47
R = 0.15
CSA = np.pi * R**2
M = 0.203 # to 0.227
DT = 0.01
MIN_LAUNCH_SPEED = 1
MAX_LAUNCH_SPEED = 20
DELTA_V = 0.25
BACKSPIN_SPEED = 3 # rad/s counterclockwise
Cl = 0.5

OFFSET_X = 0.30
OFFSET_Y = 0.0
TARGET_POSE = (4.6255 + OFFSET_X, 4.034536 + OFFSET_Y, 1.4336494314+0.4) # (x, y, z) x is the long side of the field, z is the height
LAUNCH_SPEED = 12
TOLERANCE = 0.3


HUB_PATH = "assets/hub.stl"

""""
launch_angle_z = 0 # radians
launch_pose = np.array([0, 0, 1]) # x, y, theta/rotation
launch_vel = 15

"""

def advanceNextStateNoDrag(v, p, dt):
    a = np.array([0, 0, -G])
    v_new = v + a * dt
    p_new = p + v_new * dt
    return v_new, p_new

def advanceNextStateDrag(v, p, dt):
    v_dir = v / np.linalg.norm(v)
    drag_mag = 0.5 * RHO * CSA * CD * (np.linalg.norm(v)**2)
    drag = -(drag_mag * v_dir) / M

    a_g = np.array([0, 0, -G])
    a = a_g + drag

    v_new = v + a * dt
    p_new = p + v * dt
    return v_new, p_new

def advanceNextStateDragLift(v, p, dt):
    v_dir = v / np.linalg.norm(v)
    drag_mag = 0.5 * RHO * CSA * CD * (np.linalg.norm(v)**2)
    drag = -(drag_mag * v_dir) / M

    drag_dir = -v_dir
    i = [1, 0, 0]
    lift_mag = Cl * (4/3)*(4 * np.pi ** 2 * R ** 3 * BACKSPIN_SPEED * RHO * np.linalg.norm(v))
    lift_dir = np.cross(v_dir, i)/np.linalg.norm(np.cross(v_dir, i))

    lift = lift_mag * lift_dir
    print(lift_dir)
    print(v_dir)
    print(drag_dir)

    a_g = np.array([0, 0, -G])
    a = a_g + drag + lift

    v_new = v + a * dt
    p_new = p + v * dt
    return v_new, p_new

def simulateShotNoDrag(launch_vel, pose_2d, launch_angle_z, launch_height):
    angle_rad_z = np.radians(launch_angle_z)
    pose_2d_x = pose_2d[0]
    pose_2d_y = pose_2d[1]
    pos_2d_theta = np.radians(pose_2d[2])
    v_z = launch_vel * np.sin(angle_rad_z)
    v_x = launch_vel * np.cos(angle_rad_z) * np.cos(pos_2d_theta)
    v_y = launch_vel * np.cos(angle_rad_z) * np.sin(pos_2d_theta)
    v = np.array([v_x, v_y, v_z])
    
    pos = []
    vel = []

    pos.append(np.array([pose_2d_x, pose_2d_y, launch_height]))
    vel.append(v)

    p = np.array([pose_2d_x, pose_2d_y, launch_height])

    while p[2] > 0:
        v, p = advanceNextStateNoDrag(v, p, DT)
        pos.append(p)
        vel.append(v)
    return pos, vel

def simulateShotDrag(launch_vel, pose_2d, launch_angle_z, launch_height):
    angle_rad_z = np.radians(launch_angle_z)
    pose_2d_x = pose_2d[0]
    pose_2d_y = pose_2d[1]
    pos_2d_theta = np.radians(pose_2d[2])
    v_z = launch_vel * np.sin(angle_rad_z)
    v_x = launch_vel * np.cos(angle_rad_z) * np.cos(pos_2d_theta)
    v_y = launch_vel * np.cos(angle_rad_z) * np.sin(pos_2d_theta)
    v = np.array([v_x, v_y, v_z])
    
    pos = []
    vel = []

    pos.append(np.array([pose_2d_x, pose_2d_y, launch_height]))
    vel.append(v)

    p = np.array([pose_2d_x, pose_2d_y, launch_height])

    while p[2] > 0:
        v, p = advanceNextStateDrag(v, p, DT)
        pos.append(p)
        vel.append(v)
    return pos, vel

def simulateShotDragLift(launch_vel, pose_2d, launch_angle_z, launch_height):
    angle_rad_z = np.radians(launch_angle_z)
    pose_2d_x = pose_2d[0]
    pose_2d_y = pose_2d[1]
    pos_2d_theta = np.radians(pose_2d[2])
    v_z = launch_vel * np.sin(angle_rad_z)
    v_x = launch_vel * np.cos(angle_rad_z) * np.cos(pos_2d_theta)
    v_y = launch_vel * np.cos(angle_rad_z) * np.sin(pos_2d_theta)
    v = np.array([v_x, v_y, v_z])
    
    pos = []
    vel = []

    pos.append(np.array([pose_2d_x, pose_2d_y, launch_height]))
    vel.append(v)

    p = np.array([pose_2d_x, pose_2d_y, launch_height])

    while p[2] > 0:
        v, p = advanceNextStateDragLift(v, p, DT)
        pos.append(p)
        vel.append(v)
    return pos, vel

def graphVectorVelocity(pos, vel_vector, mag):
    pos = np.array(pos)
    vel = np.array(vel_vector)
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.quiver(pos[0], pos[1], pos[2], vel[0], vel[1], vel[2], length=mag, color='b')
    ax.set_xlabel('X Position (m)')
    ax.set_ylabel('Y Position (m)')
    ax.set_zlabel('Z Position (m)')

    ax.set_xlim([0, 2])
    ax.set_ylim([0, 2])
    ax.set_zlim([0, 2])
    plt.show()

def graphSimulatedShot(pos, target, closest_point=None):
    pos = np.array(pos)
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.scatter(pos[0][0], pos[0][1], pos[0][2], s=50, color='g', label='Start')
    ax.scatter(target[0], target[1], target[2], s=50, color='r', label='Target')
    ax.plot(pos[:,0], pos[:,1], pos[:,2], label='Projectile Path')
    if closest_point is not None:
        ax.scatter(closest_point[0], closest_point[1], closest_point[2], color='g', marker='x', s=50, label='Closest Point')
    ax.set_xlabel('X Position (m)')
    ax.set_ylabel('Y Position (m)')
    ax.set_zlabel('Z Position (m)')
    ax.legend()
    plt.show()

def calculateOptimalTrajectoriesNoDrag(target, position, launch_vel):
    pos_2d = np.array([position[0], position[1]])
    target_2d = np.array([target[0], target[1]])
    dist = np.linalg.norm(target_2d - pos_2d, axis=0)
    heading = np.degrees(np.arctan2(target[1] - position[1], target[0] - position[0]))
    y_d = target[2] - position[2]

    x = dist
    v = launch_vel
    g = G

    discriminant = v**4 - g * (g * x**2 + 2 * y_d * v**2)
    if discriminant < 0:
        raise ValueError("Target out of range for given launch speed and height")

    sqrt_term = np.sqrt(discriminant)
    tan_theta1 = (v**2 + sqrt_term) / (g * x)
    tan_theta2 = (v**2 - sqrt_term) / (g * x)

    theta1 = np.degrees(np.arctan(tan_theta1))
    theta2 = np.degrees(np.arctan(tan_theta2))

    chosen_theta = max(theta1, theta2)
    return heading, chosen_theta, dist, sqrt_term, y_d

def calculateRequiredHeading(target, position):
    heading = np.degrees(np.arctan2(target[1] - position[1], target[0] - position[0]))
    return heading

def findClosestPositionToTarget(pos, target_pos=np.array(TARGET_POSE)):
    dists = np.linalg.norm(np.array(pos) - target_pos, axis=1)
    min_idx = np.argmin(dists)
    min_dist = dists[min_idx]
    closest_point = pos[min_idx]
    return min_dist, closest_point, min_idx

def findClosestZToTarget(pos, target_pos=np.array(TARGET_POSE)):
    full_pos = pos
    pos = np.array_split(pos, 2)[1]
    z = target_pos[2]
    target_2d = np.array([target_pos[0], target_pos[1]])

    max = 0
    for j in full_pos:
        if j[2] > max:
            max = j[2]
            
    if max > (z+0.3):
        min_idx = 0
        min = 100000
        for p in range(len(pos)):
            if abs(pos[p][2] - z) < min:
                min = abs(pos[p][2] - z)
                min_idx = p

        dist = np.linalg.norm(np.array([pos[min_idx][0], pos[min_idx][1]]) - target_2d)
        return dist, min_idx, min
        
    else:
        return 5.0, 0.0, 0.0

def calculateMinimumVelocity(target, position, launch_height):
    pos_2d = np.array([position[0], position[1]])
    target_2d = np.array([target[0], target[1]])
    dist = np.linalg.norm(target_2d - pos_2d, axis=0)
    y_d = target[2] - launch_height
    g = G
    x = dist
    discriminant = g*y_d + (g * (np.sqrt(x**2 + y_d**2)))
    if discriminant < 0:
        raise ValueError("No valid launch velocity for given target and height")
    
    v_min = np.sqrt(discriminant)
    return v_min

def meshcatVisualizeShot(pos, target):
    vis = meshcat.Visualizer().open()
    sphere = g.Sphere(0.05)
    target_sphere = g.Sphere(0.1)

    vis["hub"].set_object(g.StlMeshGeometry.from_file(HUB_PATH),  g.MeshLambertMaterial(color = 0x373b40,transparency= False))
    vis["hub"].set_transform(tf.translation_matrix([3.6 + target[0] - OFFSET_X, target[1] - OFFSET_Y, 0]))

    vis["target"].set_object(target_sphere, g.MeshLambertMaterial(color=0xff0000))
    vis["target"].set_transform(tf.translation_matrix(target))

    vis["projectile"].set_object(sphere, g.MeshLambertMaterial(color=0x00ff00))
    vis["projectile"].set_transform(tf.translation_matrix(pos[0]))

    formatted_pose = np.zeros((3, len(pos)), dtype=np.float32)
    for p in range(len(pos)):
        formatted_pose[0, p] = pos[p][0]
        formatted_pose[1, p] = pos[p][1]
        formatted_pose[2, p] = pos[p][2]
    
    line_vertices = np.array(formatted_pose, dtype=np.float32)
    vis['line'].set_object(g.Line(g.PointsGeometry(line_vertices)))

    time.sleep(100)

def meshcatVisualizeMultipleShots(positions, optimal_states, target):
    vis = meshcat.Visualizer().open()
    target_sphere = g.Sphere(0.1)

    vis["hub"].set_object(g.StlMeshGeometry.from_file(HUB_PATH),  g.MeshLambertMaterial(color = 0x373b40,transparency= False))
    vis["hub"].set_transform(tf.translation_matrix([3.6 + target[0] - OFFSET_X, target[1] - OFFSET_Y, 0]))
    vis["target"].set_object(target_sphere, g.MeshLambertMaterial(color=0xff0000))
    vis["target"].set_transform(tf.translation_matrix(target))

    for i in range(len(positions)):
        sphere = g.Sphere(0.05)
        pos_x = float(positions[i][0])
        pos_y = float(positions[i][1])
        vis[f"projectile_{i}"].set_object(sphere, g.MeshLambertMaterial(color=0x00ff00))
        vis[f"projectile_{i}"].set_transform(tf.translation_matrix([pos_x, pos_y, 0.5]))

        p, v = simulateShotDrag(float(optimal_states[i][0][0]), [pos_x, pos_y, float(optimal_states[i][1])], float(optimal_states[i][0][1]), 0.5)
        formatted_pose = np.zeros((3, len(p)), dtype=np.float32)
        for f in range(len(p)):
            formatted_pose[0, f] = p[f][0]
            formatted_pose[1, f] = p[f][1]
            formatted_pose[2, f] = p[f][2]

        line_vertices = np.array(formatted_pose, dtype=np.float32)
        vis[f'line_{i}'].set_object(g.Line(g.PointsGeometry(line_vertices)))

    time.sleep(100)

def meshcatVisualizeHub(target):
    vis = meshcat.Visualizer().open()
    target_sphere = g.Sphere(0.1)

    vis["hub"].set_object(g.StlMeshGeometry.from_file(HUB_PATH),  g.MeshLambertMaterial(color = 0x373b40,transparency= False))
    vis["hub"].set_transform(tf.translation_matrix([3.6 + target[0] - OFFSET_X, target[1] - OFFSET_Y, 0]))

    vis["target"].set_object(target_sphere, g.MeshLambertMaterial(color=0xff0000))
    vis["target"].set_transform(tf.translation_matrix(target))

    time.sleep(100)

def printNumpyArray(arr):
    with np.printoptions(precision=3, suppress=True):
        print(arr)

def getMaxHeight(pos):
    pos = np.array(pos)
    max = 0
    for p in pos:
        if p[2] > max:
            max = p[2]
    return max

def getAveragePathVelocity(vel):
    vel = np.array(vel)
    total = 0
    for v in vel:
        total += np.linalg.norm(v)
    return total / len(vel)
# meshcatVisualizeHub(TARGET_POSE)
P_x = 6
P_y = 6
launch_height = 0.5

v_min = calculateMinimumVelocity(TARGET_POSE, [P_x, P_y, 0], launch_height) + 0.05

heading, angle, dist, sqrt_term, y_d = calculateOptimalTrajectoriesNoDrag(TARGET_POSE, [P_x, P_y, launch_height], v_min)
pos, vel = simulateShotDrag(v_min, [P_x, P_y, heading], angle, launch_height)

print(findClosestPositionToTarget(pos))

graphSimulatedShot(pos, TARGET_POSE)


"""
THIS IS AN EXAMPLE OF MESHCAT VISUALIZATION

heading, angle, dist, sqrt_term, y_d = calculateOptimalTrajectoriesNoDrag(TARGET_POSE, [P_x, P_y, launch_height], v_min)
pos, vel = simulateShotNoDrag(v_min, [P_x, P_y, heading], angle, launch_height)

# pos_arr = np.array(pos)
# min_dist, closest_point, min_idx = findClosestPositionToTarget(pos_arr, np.array(TARGET_POSE))
# print(getMaxHeight(pos))
# print(dist)
# meshcatVisualizeShot(pos, TARGET_POSE)
"""