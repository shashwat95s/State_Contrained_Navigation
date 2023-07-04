#!/usr/bin/python3
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from math import sqrt, cos, sin, exp


    

def closest_gap(pos, xc, yc, threshold):
    xc = xc[~np.isnan(xc)]
    yc = yc[~np.isnan(yc)]
    dis = []
    for i in range(len(xc) - 1):
        dis.append(np.sqrt((xc[i] - xc[i+1])**2 + (yc[i] - yc[i+1])**2))
    
    gap_indices = []
    midpoints = []
    for i in range(len(dis)):
        if dis[i] > threshold:
            gap_indices.append(i)
    
    for i in range(len(gap_indices)):
        xa = xc[gap_indices[i]]
        ya = yc[gap_indices[i]]
        xb = xc[gap_indices[i]+1]
        yb = yc[gap_indices[i]+1]
        mDis = 999
        if np.linalg.norm(pos - np.array([xa, ya])) < np.linalg.norm(pos - np.array([xb, yb])):
            ang = np.arctan2(ya - pos[1], xa - pos[0])
            for j in range(len(xc)):
                tang = np.arctan2(yc[j] - pos[1], xc[j] - pos[0])
                if np.mod(tang - ang, 2 * np.pi) <= 3 * np.pi / 4 and np.mod(tang - ang, 2 * np.pi) > 0:
                    dist = np.linalg.norm(np.array([xa, ya]) - np.array([xc[j], yc[j]]))
                    if dist < mDis:
                        mDis = dist
                        mx = xc[j]
                        my = yc[j]
            midpoints.append([(xa + mx) / 2, (ya + my) / 2])
        else:
            ang = np.arctan2(yb - pos[1], xb - pos[0])
            for j in range(len(xc)):
                tang = np.arctan2(yc[j] - pos[1], xc[j] - pos[0])
                if np.mod(tang - ang, 2 * np.pi) >= 5 * np.pi / 4 and np.mod(tang - ang, 2 * np.pi) > 0:
                    dist = np.linalg.norm(np.array([xb, yb]) - np.array([xc[j], yc[j]]))
                    if dist < mDis:
                        mDis = dist
                        mx = xc[j]
                        my = yc[j]
            midpoints.append([(xb + mx) / 2, (yb + my) / 2])
    
    if np.sqrt((xc[-1] - xc[0])**2 + (yc[-1] - yc[0])**2) > threshold:
        xa = xc[-1]
        ya = yc[-1]
        xb = xc[0]
        yb = yc[0]
        mDis = 999
        if np.linalg.norm(pos - np.array([xa, ya])) < np.linalg.norm(pos - np.array([xb, yb])):
            ang = np.arctan2(ya - pos[1], xa - pos[0])
            for j in range(len(xc)):
                tang = np.arctan2(yc[j] - pos[1], xc[j] - pos[0])
                if np.mod(tang - ang, 2 * np.pi) <= 3 * np.pi / 4 and np.mod(tang - ang, 2 * np.pi) > 0:
                    dist = np.linalg.norm(np.array([xa, ya]) - np.array([xc[j], yc[j]]))
                    if dist < mDis:
                        mDis = dist
                        mx = xc[j]
                        my = yc[j]
            midpoints.append([(xa + mx) / 2, (ya + my) / 2])
        else:
            ang = np.arctan2(yb - pos[1], xb - pos[0])
            for j in range(len(xc)):
                tang = np.arctan2(yc[j] - pos[1], xc[j] - pos[0])
                if np.mod(tang - ang, 2 * np.pi) >= 5 * np.pi / 4 and np.mod(tang - ang, 2 * np.pi) > 0:
                    dist = np.linalg.norm(np.array([xb, yb]) - np.array([xc[j], yc[j]]))
                    if dist < mDis:
                        mDis = dist
                        mx = xc[j]
                        my = yc[j]
            midpoints.append([(xb + mx) / 2, (yb + my) / 2])
    
    return midpoints




def chooseGoalByOrientation(goals, startPosition, desiredOrientation):
    # Calculate the orientation of each goal point relative to the start position
    goals = np.array(goals)
    orientations = np.arctan2(goals[:,1]-startPosition[1], goals[:,0]-startPosition[0])
    
    # Compute the angular difference between the orientations and desired orientation
    angle = (orientations - desiredOrientation)
    angle = angle - 2*np.pi*(angle > np.pi)
    angularDifference = np.abs(angle)
    
    # Find the goal point with the minimum angular difference
    idx = np.argmin(angularDifference)
    
    # Select the closest goal point
    selectedGoal = goals[idx, :]
    
    return selectedGoal



def findNearestObstacle(obstacles, target):
    # Initialize variables
    minDistance = np.inf
    nearestObstacle = None

    # Iterate through each obstacle
    for i in range(obstacles.shape[0]):
        obstacle = obstacles[i]

        # Calculate distance between obstacle and target point
        distance = np.linalg.norm(obstacle - target)

        # Update minimum distance and nearest obstacle if necessary
        if distance < minDistance:
            minDistance = distance
            nearestObstacle = obstacle

    return nearestObstacle, minDistance


def newSP(curr, goal, K, xc, yc):
    l = 0
    dist = np.linalg.norm(goal - curr)
    angle = np.arctan2(goal[1] - curr[1], goal[0] - curr[0])

    gx = curr[0]
    gy = curr[1]
    r2 = 0.1

    inc = 0.01

    while np.linalg.norm(curr - np.array([gx, gy])) <= r2:
        l = l + inc
        gx = l * np.cos(angle) + curr[0]
        gy = l * np.sin(angle) + curr[1]
        _, minDistance = findNearestObstacle(np.column_stack((xc, yc)), np.array([gx, gy]))
        r1 = minDistance
        r2 = r1 / np.sqrt(1 + K ** 2)

    l = l - inc

    if l > dist:
        l = dist
        dir = motionDir(np.array([gx, gy]))
        sp = np.array([goal[0], goal[1], dir])
    else:
        sp = np.array([gx, gy, angle])

    return sp




def reached(A, B):
    if np.sqrt((A[1] - B[1]) ** 2 + (A[0] - B[0]) ** 2) < 0.05 and ((np.abs(A[2] - B[2]) % (2 * np.pi)) < 0.0173 * 1 or (np.abs(A[2] - B[2]) % (2 * np.pi)) > 2 * np.pi - 0.0173 * 1):
        t = 1
    else:
        t = 0
    return t



def getLidarCoordinates(ranges, angles, robotPosition):
    # Convert robot position to radians
    robotAngle = robotPosition[2]
    
    # Calculate the coordinates of each reading
    x = ranges * np.cos(angles + robotAngle) + robotPosition[0]
    y = ranges * np.sin(angles + robotAngle) + robotPosition[1]
    
    return x, y


# # Formatting the plots
# plt.rcParams['lines.linewidth'] = 2
# plt.rcParams['axes.linewidth'] = 2
# plt.rcParams['axes.labelsize'] = 18
# plt.rcParams['xtick.labelsize'] = 18
# plt.rcParams['ytick.labelsize'] = 18
# plt.rcParams['font.family'] = 'arial'
# plt.rcParams['axes.grid'] = True
# plt.rcParams['text.usetex'] = True
# plt.rcParams['axes.title'] = r'\textbf{latex}'

# Basic grid setup
map_size = (200, 200)
map = np.zeros(map_size)
map[0:2, :] = 1
map[-2:, 60:] = 1
map[:, 0:2] = 1
map[60:, -2:] = 1
map[1:16, 95:106] = 1
map[45:61, 95:106] = 1
map[60:63, 60:] = 1
map[60:, 60:63] = 1

lidar_range = [0, 100]


class RangeSensor:
    def __init__(self):
        self.range = lidar_range

    def __call__(self, pose, occupancy_map):
        x, y, th = pose
        ranges = []
        angles = np.linspace(-np.pi / 2, np.pi / 2, 181)
        for angle in angles:
            x_end = x + self.range[1] * np.cos(th + angle)
            y_end = y + self.range[1] * np.sin(th + angle)
            ray = np.linspace(0, 1, int(np.sqrt((x_end - x) ** 2 + (y_end - y) ** 2) * 100))
            x_ray = np.interp(ray, [0, 1], [x, x_end])
            y_ray = np.interp(ray, [0, 1], [y, y_end])
            indices = np.round(x_ray).astype(int), np.round(y_ray).astype(int)
            if np.any(indices[0] < 0) or np.any(indices[1] < 0) or \
                    np.any(indices[0] >= occupancy_map.shape[0]) or np.any(indices[1] >= occupancy_map.shape[1]):
                ranges.append(np.nan)
            else:
                ranges.append(np.nanmin(occupancy_map[indices]))
        return np.array(ranges), angles


class BinaryOccupancyMap:
    def __init__(self, size):
        self.size = size
        self.occupancy_map = np.zeros(size)

    def set_occupancy(self, occ):
        self.occupancy_map = occ

    def show(self):
        plt.imshow(self.occupancy_map, cmap='binary')
        plt.show()


lidar = RangeSensor()
occupancy_map = BinaryOccupancyMap(map_size)
occupancy_map.set_occupancy(map)
# occupancy_map.show()

samp_T = 0.01
x_start = [180, 170, -np.pi]
X = np.zeros((3, 1))
X[:, 0] = x_start
t_euler = [0]

Path = []
V = [0]
W = [0]
R = [0]

K = 1
K1 = 0.1
Ks = 2
fl = 0

goal=[]
curr=[]
for i in range(40):
    ranges, angles = lidar(X[:, -1], occupancy_map.occupancy_map)
    xr, yr = getLidarCoordinates(ranges, angles,X[:, -1])
    xr, yr = xr[~np.isnan(xr)], yr[~np.isnan(yr)]

    a = closest_gap(X[:2, -1], xr, yr, 15)
    if X[0, -1] < 60:
        in_dir = -np.pi / 2
    else:
        in_dir = -np.pi

    selected_goal = chooseGoalByOrientation(a, X[:, -1], in_dir)
    
    curr=X[:, -1]
    goal = newSP([curr[0],curr[1]], selected_goal, K, xr, yr)


    plt.figure(1)
    plt.scatter(xr, yr)
    plt.plot(selected_goal[0], selected_goal[1], 'o', color='green', linewidth=1.5)
    plt.plot(X[0, -1], X[1, -1], 'o', color='red', linewidth=1.5)
    plt.show()
    Path.append([goal[0], goal[1]])

    if np.mod(X[2, -1] - goal[2], 2 * np.pi) > np.pi:
        sg = -1
        K = -1
    else:
        sg = 1
        K = 1

    while True:
        x_start = X[:, -1]
        r2 = np.sqrt((x_start[0] - goal[0]) ** 2 + (x_start[1] - goal[1]) ** 2)
        r1 = r2 * np.sqrt(1 + K ** 2)
        r3 = r2 * r1 * np.sqrt(K ** 2 + 4) * np.exp(-K * (x_start[2] - goal[2])) / (r2 * sg * K)
        if r3 < 0.1:
            break
        else:
            X[2, -1] = X[2, -1] + sg * 2 * np.pi

    j = 0
    sumS = 0
    while not reached(X[:, -1], goal):
        j += 1
        if j > 10:
            break

        x0 = (-goal[0] + X[0, -1])
        y0 = (-goal[1] + X[1, -1])
        th_d = goal[2]
        alpha = X[2, -1] - goal[2]

        x = np.cos(th_d) * x0 + np.sin(th_d) * y0
        y = np.cos(th_d) * y0 - np.sin(th_d) * x0

        z1 = alpha
        z2 = x * np.cos(alpha) + y * np.sin(alpha)
        z3 = x * np.sin(alpha) - y * np.cos(alpha)
        S = z2 - K * z3
        sumS += np.sign(S) * samp_T

        if fl == 0:
            surf = np.array([S, S])
            fl = 1
        else:
            surf = np.append(surf, S)

        if np.abs(S) < 0.005:
            omega = (-K1 * z1)
            v = (-K1 * z1 * (z3 + K * z2))
        else:
            omega = 0
            v = (-Ks * np.sign(S))

        omega = -K1 * z1
        v = (-K1 * z1 * (z3 + K * z2)) - Ks * 20 * np.sign(S) * np.sqrt(np.abs(S)) - 0.1 * sumS
        V.append(v)
        W.append(omega)
        R.append(v / omega)

        X = np.append(X, [X[:, -1] + [v * np.cos(X[2, -1]), v * np.sin(X[2, -1]), omega] * samp_T], axis=1)
        t_euler.append(t_euler[-1] + samp_T)

plot(X[0, :], X[1, :])
plt.plot(X[0, -1], X[1, -1], 'o', color='red', linewidth=1.5)
plt.plot(X[0, 0], X[1, 0], 'o', color='green', linewidth=1.5)

for i in range(len(Path) - 1):
    plt.plot(Path[i][0], Path[i][1], 'o', color='black', linewidth=1.5)

plt.xlabel('Position X')
plt.ylabel('Position Y')
plt.title('Position')
plt.legend(['path', 'last', 'start', 'obstacle', '', '', 'waypoint'])
plt.show()

plt.plot(t_euler, X[2, :])
plt.axhline(y=goal[2], linestyle='--', color='black', linewidth=1.5)
plt.xlabel('Time')
plt.ylabel('Orientation (theta)')
plt.title('Orientation')
plt.legend(['theta', 'goal_theta'])
plt.show()

plt.plot(t_euler, V)
plt.plot(t_euler, W)
plt.plot(t_euler, R)
plt.xlabel('Time')
plt.ylabel('Velocity')
plt.title('Velocity')
plt.legend(['V', 'W', 'R'])
plt.show()
