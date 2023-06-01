"""

Mobile robot motion planning sample with Dynamic Window Approach

author: Atsushi Sakai (@Atsushi_twi), Göktuğ Karakaşlı

"""

import math
from enum import Enum

import matplotlib.pyplot as plt
import numpy as np

import mujoco
import prm

# parameter
prm.N_SAMPLE = 500  # number of sample_points
prm.N_KNN = 10  # number of edge from one sampled point
prm.MAX_EDGE_LEN = 30.0  # [m] Maximum edge length

show_animation = True


def dwa_control(x, config, goal, ob):
    """
    Dynamic Window Approach control
    """
    dw = calc_dynamic_window(x, config)

    u, trajectory = calc_control_and_trajectory(x, dw, config, goal, ob)

    return u, trajectory


class RobotType(Enum):
    circle = 0
    rectangle = 1


class Config:
    """
    simulation parameter class
    """

    def __init__(self):
        # robot parameter
        self.max_speed = 0.8  # [m/s]
        self.min_speed = -0.8  # [m/s]
        self.max_yaw_rate = 30.0 * math.pi / 180.0  # [rad/s]
        self.max_accel = 0.3  # [m/ss]
        self.max_delta_yaw_rate = 20.0 * math.pi / 180.0  # [rad/ss]
        self.v_resolution = 0.05  # [m/s]
        self.yaw_rate_resolution = 0.1 * math.pi / 180.0  # [rad/s]
        self.dt = 0.1  # [s] Time tick for motion prediction
        self.predict_time = 3.0  # [s]
        self.to_goal_cost_gain = 0.3
        self.speed_cost_gain = 1.5
        self.obstacle_cost_gain = 1.0
        self.robot_stuck_flag_cons = self.v_resolution  # constant to prevent robot stucked
        self.robot_type = RobotType.circle

        # if robot_type == RobotType.circle
        # Also used to check if goal is reached in both types
        self.robot_radius = 0.4  # [m] for collision check
        self.success_distance = self.robot_radius
        # if robot_type == RobotType.rectangle
        self.robot_width = 0.8  # [m] for collision check
        self.robot_length = 1.8  # [m] for collision check
        # obstacles [x(m) y(m), ....]
        self.ob = np.array([[10,10]
                            ])

    @property
    def robot_type(self):
        return self._robot_type

    @robot_type.setter
    def robot_type(self, value):
        if not isinstance(value, RobotType):
            raise TypeError("robot_type must be an instance of RobotType")
        self._robot_type = value


config = Config()


def motion(x, u, dt):
    """
    motion model
    """

    x[2] += u[1] * dt
    x[0] += u[0] * math.cos(x[2]) * dt
    x[1] += u[0] * math.sin(x[2]) * dt
    x[3] = u[0]
    x[4] = u[1]

    return x


def calc_dynamic_window(x, config):
    """
    calculation dynamic window based on current state x
    """

    # Dynamic window from robot specification
    Vs = [config.min_speed, config.max_speed,
          -config.max_yaw_rate, config.max_yaw_rate]

    # Dynamic window from motion model
    Vd = [x[3] - config.max_accel * config.dt,
          x[3] + config.max_accel * config.dt,
          x[4] - config.max_delta_yaw_rate * config.dt,
          x[4] + config.max_delta_yaw_rate * config.dt]

    #  [v_min, v_max, yaw_rate_min, yaw_rate_max]
    dw = [max(Vs[0], Vd[0]), min(Vs[1], Vd[1]),
          max(Vs[2], Vd[2]), min(Vs[3], Vd[3])]

    return dw


def predict_trajectory(x_init, v, y, config):
    """
    predict trajectory with an input
    """

    x = np.array(x_init)
    trajectory = np.array(x)
    time = 0
    while time <= config.predict_time:
        x = motion(x, [v, y], config.dt)
        trajectory = np.vstack((trajectory, x))
        time += config.dt

    return trajectory


def calc_control_and_trajectory(x, dw, config, goal, ob):
    """
    calculation final input with dynamic window
    """

    x_init = x[:]
    min_cost = float("inf")
    best_u = [0.0, 0.0]
    best_trajectory = np.array([x])

    # evaluate all trajectory with sampled input in dynamic window
    for v in np.arange(dw[0], dw[1], config.v_resolution):
        for y in np.arange(dw[2], dw[3], config.yaw_rate_resolution):

            trajectory = predict_trajectory(x_init, v, y, config)
            # calc cost
            to_goal_cost = config.to_goal_cost_gain * calc_to_goal_cost(trajectory, goal)
            ob_cost = config.obstacle_cost_gain * calc_obstacle_cost(trajectory, ob, config)
            speed_cost = config.speed_cost_gain * (config.max_speed - trajectory[-1, 3])

            # if ob_cost >= to_goal_cost:
            #     speed_cost = config.speed_cost_gain * (config.max_speed - abs(trajectory[-1, 3]))
            # else:
            #     speed_cost = config.speed_cost_gain * (config.max_speed - trajectory[-1, 3])
            

            final_cost = to_goal_cost + speed_cost + ob_cost

            # search minimum trajectory
            if min_cost >= final_cost:
                min_cost = final_cost
                best_u = [v, y]
                best_trajectory = trajectory
                if abs(best_u[0]) < config.robot_stuck_flag_cons \
                        and abs(x[3]) < config.robot_stuck_flag_cons:
                    # to ensure the robot do not get stuck in
                    # best v=0 m/s (in front of an obstacle) and
                    # best omega=0 rad/s (heading to the goal with
                    # angle difference of 0)
                    best_u[1] = -config.max_delta_yaw_rate

    return best_u, best_trajectory


def calc_obstacle_cost(trajectory, ob, config):
    """
    calc obstacle cost inf: collision
    """
    ox = ob[:, 0]
    oy = ob[:, 1]
    dx = trajectory[:, 0] - ox[:, None]
    dy = trajectory[:, 1] - oy[:, None]
    r = np.hypot(dx, dy)

    if config.robot_type == RobotType.rectangle:
        yaw = trajectory[:, 2]
        rot = np.array([[np.cos(yaw), -np.sin(yaw)], [np.sin(yaw), np.cos(yaw)]])
        rot = np.transpose(rot, [2, 0, 1])
        local_ob = ob[:, None] - trajectory[:, 0:2]
        local_ob = local_ob.reshape(-1, local_ob.shape[-1])
        local_ob = np.array([local_ob @ x for x in rot])
        local_ob = local_ob.reshape(-1, local_ob.shape[-1])
        upper_check = local_ob[:, 0] <= config.robot_length / 2
        right_check = local_ob[:, 1] <= config.robot_width / 2
        bottom_check = local_ob[:, 0] >= -config.robot_length / 2
        left_check = local_ob[:, 1] >= -config.robot_width / 2
        if (np.logical_and(np.logical_and(upper_check, right_check),
                           np.logical_and(bottom_check, left_check))).any():
            return float("Inf")
    elif config.robot_type == RobotType.circle:
        if np.array(r <= config.robot_radius).any():
            return float("Inf")

    min_r = np.min(r)
    return 1.0 / min_r  # OK


def calc_to_goal_cost(trajectory, goal):
    """
        calc to goal cost with angle difference
    """

    dx = goal[0] - trajectory[-1, 0]
    dy = goal[1] - trajectory[-1, 1]
    error_angle = math.atan2(dy, dx)
    cost_angle = error_angle - trajectory[-1, 2]
    cost = abs(math.atan2(math.sin(cost_angle), math.cos(cost_angle)))

    return cost


def plot_arrow(x, y, yaw, length=0.5, width=0.1):  # pragma: no cover
    plt.arrow(x, y, length * math.cos(yaw), length * math.sin(yaw),
              head_length=width, head_width=width)
    plt.plot(x, y)


def plot_robot(x, y, yaw, config):  # pragma: no cover
    if config.robot_type == RobotType.rectangle:
        outline = np.array([[-config.robot_length / 2, config.robot_length / 2,
                             (config.robot_length / 2), -config.robot_length / 2,
                             -config.robot_length / 2],
                            [config.robot_width / 2, config.robot_width / 2,
                             - config.robot_width / 2, -config.robot_width / 2,
                             config.robot_width / 2]])
        Rot1 = np.array([[math.cos(yaw), math.sin(yaw)],
                         [-math.sin(yaw), math.cos(yaw)]])
        outline = (outline.T.dot(Rot1)).T
        outline[0, :] += x
        outline[1, :] += y
        plt.plot(np.array(outline[0, :]).flatten(),
                 np.array(outline[1, :]).flatten(), "-k")
    elif config.robot_type == RobotType.circle:
        circle = plt.Circle((x, y), config.robot_radius, color="b")
        plt.gcf().gca().add_artist(circle)
        out_x, out_y = (np.array([x, y]) +
                        np.array([np.cos(yaw), np.sin(yaw)]) * config.robot_radius)
        plt.plot([x, out_x], [y, out_y], "-k")



"""" Transpalet Spesific Functions """

def critical_action(x,y,critical_points,modified_prm_data, s_prm_data):
    pos = [x,y]; choosen = np.array([]); s_pos = xy_to_s(x,y,modified_prm_data,s_prm_data)
    for c_p in critical_points:
        dist = np.hypot(c_p[1,0]-pos[0],c_p[1,1]-pos[1])
        if dist <= 2.5:
            choosen = c_p
            break

    # print("choosen size=",choosen.size)
    if choosen.size == 0:
        return np.array([])
    
    s_choosen = []
    for i in choosen:
        temp = xy_to_s(i[0],i[1],modified_prm_data,s_prm_data); temp.tolist()
        s_choosen.append(temp)

    if abs(s_choosen[0]-s_choosen[2]) >= 1.5:

        index_choosen = 0; possible_goal = []
        for i in choosen:
            dist_choosen = np.hypot(i[0]-pos[0],i[1]-pos[1])
           
            if dist_choosen >= 1.0 and s_choosen[index_choosen] > s_pos:
                possible_goal.append((i,s_choosen[index_choosen]))

            index_choosen = index_choosen+1

        # print("possible_goal=",possible_goal)
        if not possible_goal:
            return np.array([])
        
        min_s = np.Inf
        for i in possible_goal:
            if i[1] < min_s:
                goal = i[0]
                min_s = i[1]
        
        return goal
    
    else:
        return np.array([])

def find_dwa_goal(follow_dist, x, y, modified_prm_data, s_prm_data):
    s = xy_to_s(x, y, modified_prm_data, s_prm_data)
    goal_dwa = s_to_xy(s + follow_dist, modified_prm_data, s_prm_data)

    return goal_dwa

def s_to_xy(s, modified_prm_data, s_prm_data):
    difference = abs(s_prm_data - s)
    closest_index = np.argmin(difference)

    xy_prm = modified_prm_data[closest_index]

    return xy_prm

def xy_to_s(x, y, modified_prm_data, s_prm_data):

    distances = np.hypot((modified_prm_data[:, 0] - x),(modified_prm_data[:, 1] - y))
    closest_index = np.argmin(distances)

    s = s_prm_data[closest_index]

    return s

def modify_prm_road(road_prm, step):
    road_prm = np.flip(road_prm, axis=0)

    modified_prm_data = np.empty((0, 2))
    for i in range(len(road_prm)):
        if i == 0:
            continue

        if i == len(road_prm)-1:
            road_temp = lin_data(road_prm[i-1],road_prm[i],step)
            modified_prm_data = np.vstack((modified_prm_data, road_temp))
            break

        road_temp = lin_data(road_prm[i-1],road_prm[i],step)
        road_temp = road_temp[:-1]
        modified_prm_data = np.vstack((modified_prm_data, road_temp))

    s_prm_data = np.cumsum([0] + [np.hypot((modified_prm_data[i][0] - modified_prm_data[i-1][0]),
                                            (modified_prm_data[i][1] - modified_prm_data[i-1][1])) for i in range(1, len(modified_prm_data))])

    return s_prm_data, modified_prm_data

def remove_duplicates(arr):
    """
    Given a NumPy ndarray, remove any duplicates and return the modified ndarray.
    """
    unique_arr = np.array(list(set(tuple(row) for row in arr)))
    return unique_arr

def lin_data(start, end, step):
    
    start = np.array(start)
    end = np.array(end)

    direction = end - start
    distance = np.linalg.norm(direction)
    data = np.linspace(0, distance, num=int(distance/step))
    points = start + (direction / distance) * data.reshape(-1, 1)
    return points

# gx=-3.0, gy=6.0
def main(gx=-4.0, gy=3.4, robot_type=RobotType.circle):
    print(__file__ + " start!!")
    # initial state [x(m), y(m), yaw(rad), v(m/s), omega(rad/s)]
    x = np.array([7.0, 0.0, math.pi*-0.5, 0.0, 0.0])

    model = mujoco.MjModel.from_xml_path('scene2.xml')

    wall_number = 12
    wall_information = []
    for wall_index in range(1, wall_number+1):
        wall = model.geom('wall_'+str(wall_index))
        wall_bottom_left_x = (wall.pos[0]-wall.size[0])
        wall_bottom_left_y = (wall.pos[1]-wall.size[1])
        wall_top_right_x = (wall.pos[0]+wall.size[0])
        wall_top_right_y = (wall.pos[1]+wall.size[1])

        wall_corners = [[wall_bottom_left_x,wall_bottom_left_y], [wall_top_right_x,wall_top_right_y]]
        wall_information.append(wall_corners)

    robot_size = config.robot_radius
    for i in range(wall_number):
        wall_bottom_left = wall_information[i][0]
        wall_top_right = wall_information[i][1]
        wall_bottom_right = [wall_top_right[0],wall_bottom_left[1]]
        wall_top_left = [wall_bottom_left[0],wall_top_right[1]]

        obs_temp = np.concatenate((lin_data(wall_top_left, wall_top_right, robot_size), lin_data(wall_top_right, wall_bottom_right, robot_size),
                    lin_data(wall_bottom_right, wall_bottom_left, robot_size), lin_data(wall_bottom_left, wall_top_left, robot_size)), axis=0)

        if i == 0:
            obs = obs_temp
        else:
            obs = np.vstack((obs, obs_temp))

    obs = remove_duplicates(obs)
    ox = obs[:,0]
    oy = obs[:,1]

    road_prm_x = []; i = 0
    while not road_prm_x and i <= 4:
        road_prm_x, road_prm_y = prm.prm_planning(x[0], x[1], gx, gy, ox, oy, robot_size*1.5, rng=None)
        i = i+1

    road_prm_x = np.array(road_prm_x); road_prm_y = np.array(road_prm_y)
    road_prm_x = road_prm_x.reshape((-1,1)); road_prm_y = road_prm_y.reshape((-1,1))
    road_prm = np.concatenate((road_prm_x,road_prm_y), axis=1)

    s_prm_data, modified_prm_data = modify_prm_road(road_prm, 0.1)

    critical_points = np.array([((-2.14-1.5,3.3),(-2.14,3.3),(-2.14+1.5,3.3)),
                                ((2.05-1.5,10.8),(2.05,10.8),(2.05+1.5,10.8)),
                                ((-13.2,2.2-1.5),(-13.2,2.2),(-13.2,2.2+1.5)),
                                ((13.2,2.2-1.5),(13.2,2.2),(13.2,2.2+1.5))])
    # input [forward speed, yaw_rate]

    config.robot_type = robot_type
    trajectory = np.array(x)
    ob = obs
    while True:

        critic_goal = critical_action(x[0],x[1],critical_points,modified_prm_data, s_prm_data)
        if critic_goal.size == 0:
            goal = find_dwa_goal(2.5, x[0], x[1], modified_prm_data, s_prm_data)
        else:
            goal = critic_goal

        u, predicted_trajectory = dwa_control(x, config, goal, ob)
        x = motion(x, u, config.dt)  # simulate robot
        print("u = ", u)
        trajectory = np.vstack((trajectory, x))  # store state history

        if show_animation:
            plt.cla()
            # for stopping simulation with the esc key.
            plt.gcf().canvas.mpl_connect(
                'key_release_event',
                lambda event: [exit(0) if event.key == 'escape' else None])
            plt.plot(modified_prm_data[:,0], modified_prm_data[:,1], 'c--')
            plt.plot(predicted_trajectory[:, 0], predicted_trajectory[:, 1], "-g")
            plt.plot(x[0], x[1], "xr")
            plt.plot(goal[0], goal[1], "xb")
            plt.plot(ob[:, 0], ob[:, 1], "ok")
            plot_robot(x[0], x[1], x[2], config)
            plot_arrow(x[0], x[1], x[2])
            plt.axis("equal")
            plt.grid(True)
            plt.pause(0.0001)

        # check reaching goal
        dist_to_goal = math.hypot(x[0] - goal[0], x[1] - goal[1])
        if dist_to_goal <= config.robot_radius:
            print("Goal!!")
            break

    print("Done")
    if show_animation:
        plt.plot(trajectory[:, 0], trajectory[:, 1], "-r")
        plt.pause(0.0001)
        plt.show()


if __name__ == '__main__':
    main(robot_type=RobotType.rectangle)
    # main(robot_type=RobotType.circle)