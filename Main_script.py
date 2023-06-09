import xml.etree.ElementTree as ET
from dm_control.suite import common

import mujoco
import mujoco_viewer
import numpy as np
import math
import dwa
import prm
from scipy.spatial.transform import Rotation

from simple_pid import PID

# Setting the positions and the rotations of the both transpalet and pallet
transpalet_pos = [17.0, 0.0]; transpalet_rot = math.pi*0.5
palet_pos = [-9.0, 8.0]; palet_rot = math.pi*0.8
desired_palet_pos = [-15.0, 9.0]; desired_palet_rot = math.pi*0.5

transpalet_path = "/myworks/term_project_folder/model.xml"
palet_path = "/myworks/term_project_folder/palet.xml"

transpalet_xml_string = common.read_model(transpalet_path)
palet_xml_string = common.read_model(palet_path)

transpalet_root = ET.fromstring(transpalet_xml_string)
palet_root = ET.fromstring(palet_xml_string)

transpalet_body = transpalet_root.find(".//worldbody/body[@name='atlas_transpalet_body']")
palet_body = palet_root.find(".//worldbody/body[@name='palet_body']")

transpalet_body.set("pos", f"{transpalet_pos[0]} {transpalet_pos[1]} 0"); transpalet_body.set("euler", f"0 0 {transpalet_rot}")
palet_body.set("pos", f"{palet_pos[0]} {palet_pos[1]} 0.075"); palet_body.set("euler", f"0 0 {palet_rot}")

with open(transpalet_path, 'w') as f:
    f.write(ET.tostring(transpalet_root, encoding='unicode'))

with open(palet_path, 'w') as f:
    f.write(ET.tostring(palet_root, encoding='unicode'))



def palet_take_or_put_calculations(palet_index, palet_normal, palet_pos, palet_rot, mid_point_pos, mid_point_rotation_angles):
        veh_angle = model_angle_correction(mid_point_rotation_angles[0])

        if palet_index == 0:
            palet_normal= -palet_normal
            veh_angle = mid_point_rotation_angles[0]

        goal_vec = np.array([palet_pos[0]-mid_point_pos[0], palet_pos[1]-mid_point_pos[1]])

        d_palet = np.dot(goal_vec, palet_normal)
        angle_diff_palet = palet_rot - veh_angle
        

        pid1 = PID(Kp=0.8, Ki=0.0, Kd=0.02, setpoint=0, output_limits= (-1,1))
        pid2 = PID(Kp=1.5, Ki=0.0, Kd=0.02, setpoint=0, output_limits= (-1,1))

        steering_angle_control = pid1(d_palet)+pid2(angle_diff_palet)
        drive_speed_control = -0.15

        return drive_speed_control, steering_angle_control

def deciding_control_inputs(lat_drive_vel, long_drive_vel):
    mag_drive_vel = np.hypot(lat_drive_vel, long_drive_vel)

    if long_drive_vel >= 0:
        drive_speed_control = -mag_drive_vel/0.8
    else:
        drive_speed_control = mag_drive_vel/0.8

    steering_drive_angle = math.atan2(long_drive_vel, lat_drive_vel)
    if steering_drive_angle < 0:
        steering_drive_angle = steering_drive_angle + math.pi

    steering_angle_control= steering_drive_angle/(math.pi/2) - 1

    return drive_speed_control, steering_angle_control

def global_path_planning(x, y, gx, gy, ox, oy, robot_size):
    road_prm_x = []; i = 0
    while not road_prm_x and i <= 4:
        road_prm_x, road_prm_y = prm.prm_planning(x, y, gx, gy, ox, oy, robot_size*1.5, rng=None)
        i = i+1

    road_prm_x = np.array(road_prm_x); road_prm_y = np.array(road_prm_y)
    road_prm_x = road_prm_x.reshape((-1,1)); road_prm_y = road_prm_y.reshape((-1,1))
    road_prm = np.concatenate((road_prm_x,road_prm_y), axis=1)

    s_prm_data, modified_prm_data = modify_prm_road(road_prm, 0.1)

    return s_prm_data, modified_prm_data

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

'''
                                        Mujoco Spesific Functions
'''
def check_integer_multp(num1, num2):
    value = num1/num2
    value = round(value)
    error = abs(num1 - value*num2)
    if error <= 1e-6:
        return True
    else:
        return False
    
def model_angle_correction(angle_model):
    if angle_model >= 0:
        angle_dwa = angle_model - math.pi
    else:
        angle_dwa = angle_model + math.pi

    return angle_dwa

# robot parameters
config = dwa.Config()
config.max_speed = 0.8  # [m/s]
config.min_speed = -0.8  # [m/s]
config.max_yaw_rate = 30.0 * math.pi / 180.0  # [rad/s]
config.max_accel = 0.3  # [m/ss]
config.max_delta_yaw_rate = 20.0 * math.pi / 180.0  # [rad/ss]
config.v_resolution = 0.05  # [m/s]
config.yaw_rate_resolution = 0.1 * math.pi / 180.0  # [rad/s]
config.dt = 0.1  # [s] Time tick for motion prediction
config.predict_time = 3.0  # [s]
config.to_goal_cost_gain = 0.25 # 0.25
config.speed_cost_gain = 1.5
config.obstacle_cost_gain = 0.7 #0.7
config.robot_stuck_flag_cons = config.v_resolution  # constant to prevent robot stucked
config.robot_type = dwa.RobotType.circle
# if robot_type == RobotType.circle
# Also used to check if goal is reached in both types
config.robot_radius = 0.4  # [m] for collision check
# if robot_type == RobotType.rectangle
config.robot_width = 0.8  # [m] for collision check
config.robot_length = 1.8   # [m] for collision check
# obstacles [x(m) y(m), ....]
config.ob = np.array([[10,10]])
prm.N_SAMPLE = 500  # number of sample_points
prm.N_KNN = 10  # number of edge from one sampled point
prm.MAX_EDGE_LEN = 30.0  # [m] Maximum edge length


model = mujoco.MjModel.from_xml_path('scene2.xml')
data = mujoco.MjData(model)

model.opt.timestep = 0.005

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

dynamic_object_actuators = [data.actuator("dynamic_object_1"), data.actuator("dynamic_object_2"), data.actuator("dynamic_object_3")]
dynamic_object_positions = [data.site("dynamic_object_1_link"), data.site("dynamic_object_2_link"), data.site("dynamic_object_3_link")]
dynamic_object_speed = 0.2

def update_dynamic_objects(step_counter):
    for dynamic_object_index in range(0, len(dynamic_object_actuators)):
        dynamic_object_actuators[dynamic_object_index].ctrl = abs(math.sin(dynamic_object_speed * math.pi * step_counter / 1000)) * 100
        # print(dynamic_object_positions[dynamic_object_index].xpos)

def update_dynamic_obs(obs, dynamic_object_positions):
    ob = obs
    for d_o in dynamic_object_positions:
        d_o = np.array(d_o.xpos[:2])
        ob = np.vstack((ob, d_o))

    return ob

step_counter = 0

mid_point = data.site("mid_point")
drive_speed_actuator = data.actuator("drive_speed")
steering_angle_actuator = data.actuator("steering_angle")
fork_height_actuator = data.actuator("fork_height")

u = [0.0, 0.0]

reach_palet = True; take_palet = False; move_palet = False; put_palet = False; move_back = False; first_action = True; end_action = False 

viewer = mujoco_viewer.MujocoViewer(model, data)
while True:
    time = data.time
    mid_point_pos = mid_point.xpos

    mid_point_rotation_matrix = np.array(mid_point.xmat).reshape(3,3)
    mid_point_rotation =  Rotation.from_matrix(mid_point_rotation_matrix) 
    mid_point_rotation_angles = mid_point_rotation.as_euler("zyx")

    if check_integer_multp(time, config.dt):

        ob = update_dynamic_obs(obs, dynamic_object_positions)

        update_dynamic_objects(step_counter)
        step_counter = step_counter + 1

        if reach_palet:

            if first_action:
                s_prm_data, modified_prm_data = global_path_planning(transpalet_pos[0], transpalet_pos[1], palet_pos[0], palet_pos[1], ox, oy, robot_size)
                first_action = False

            goal = find_dwa_goal(4.0, mid_point_pos[0], mid_point_pos[1], modified_prm_data, s_prm_data)

            # initial state [x(m), y(m), yaw(rad), v(m/s), omega(rad/s)]
            x = np.array([mid_point_pos[0], mid_point_pos[1], model_angle_correction(mid_point_rotation_angles[0]), u[0], u[1]])
            u, _ = dwa.dwa_control(x, config, goal, ob)

            lat_drive_vel = u[1]*1.275
            long_drive_vel = u[0]

            drive_speed_actuator.ctrl ,steering_angle_actuator.ctrl = deciding_control_inputs(lat_drive_vel, long_drive_vel)

            dist_to_main_goal = np.hypot((palet_pos[0]-mid_point_pos[0]),(palet_pos[1]-mid_point_pos[1]))
            s_dist_to_main_goal = xy_to_s(palet_pos[0],palet_pos[1],modified_prm_data,s_prm_data) - xy_to_s(mid_point_pos[0],mid_point_pos[1],modified_prm_data,s_prm_data)

            if end_action:
                u = [0.0, 0.0]
                reach_palet = False
                take_palet = True
                first_action = True
                end_action = False

            if dist_to_main_goal <= 5 and s_dist_to_main_goal <= 7 and reach_palet:
                drive_speed_actuator.ctrl = 0
                steering_angle_actuator.ctrl = 0
                end_action = True

        if take_palet:

            if first_action:
                palet_direction = np.array([np.cos(palet_rot), np.sin(palet_rot)])
                palet_normal = np.array([-np.sin(palet_rot), np.cos(palet_rot)])
                palet_enters = np.array([(palet_pos + 3*palet_direction), (palet_pos - 3*palet_direction)])
                palet_enter_dists = [np.hypot((palet_enters[0,0]-mid_point_pos[0]),(palet_enters[0,1]-mid_point_pos[1])),
                                    np.hypot((palet_enters[1,0]-mid_point_pos[0]),(palet_enters[1,1]-mid_point_pos[1]))]

                palet_index = np.argmin(palet_enter_dists)
                first_action = False

            drive_speed_actuator.ctrl ,steering_angle_actuator.ctrl = palet_take_or_put_calculations(palet_index, palet_normal, palet_pos, palet_rot, mid_point_pos, mid_point_rotation_angles)
            
            dist_to_main_goal = np.hypot((palet_pos[0]-mid_point_pos[0]),(palet_pos[1]-mid_point_pos[1]))

            if end_action:

                take_palet = False
                move_palet = True
                first_action = True
                end_action = False

            if dist_to_main_goal <= 0.3 and take_palet:
                drive_speed_actuator.ctrl = 0
                steering_angle_actuator.ctrl = 0
                fork_height_actuator.ctrl = 1.0
                end_action = True

        if move_palet:

            if first_action:
                s_prm_data, modified_prm_data = global_path_planning(mid_point_pos[0], mid_point_pos[1], desired_palet_pos[0], desired_palet_pos[1], ox, oy, robot_size)
                first_action = False

            goal = find_dwa_goal(4.0, mid_point_pos[0], mid_point_pos[1], modified_prm_data, s_prm_data)

            # initial state [x(m), y(m), yaw(rad), v(m/s), omega(rad/s)]
            x = np.array([mid_point_pos[0], mid_point_pos[1], model_angle_correction(mid_point_rotation_angles[0]), u[0], u[1]])
            u, _ = dwa.dwa_control(x, config, goal, ob)

            lat_drive_vel = u[1]*1.275
            long_drive_vel = u[0]

            drive_speed_actuator.ctrl ,steering_angle_actuator.ctrl = deciding_control_inputs(lat_drive_vel, long_drive_vel)

            dist_to_main_goal = np.hypot((desired_palet_pos[0]-mid_point_pos[0]),(desired_palet_pos[1]-mid_point_pos[1]))
            s_dist_to_main_goal = xy_to_s(desired_palet_pos[0],desired_palet_pos[1],modified_prm_data,s_prm_data) - xy_to_s(mid_point_pos[0],mid_point_pos[1],modified_prm_data,s_prm_data)

            if end_action:
                u = [0.0, 0.0]
                move_palet = False
                put_palet = True
                first_action = True
                end_action = False

            if dist_to_main_goal <= 5 and s_dist_to_main_goal <= 7 and move_palet:
                drive_speed_actuator.ctrl = 0
                steering_angle_actuator.ctrl = 0
                end_action = True
            
        if put_palet:
            if first_action:
                palet_direction = np.array([np.cos(desired_palet_rot), np.sin(desired_palet_rot)])
                palet_normal = np.array([-np.sin(desired_palet_rot), np.cos(desired_palet_rot)])
                palet_enters = np.array([(desired_palet_pos + 3*palet_direction), (desired_palet_pos - 3*palet_direction)])
                palet_enter_dists = [np.hypot((palet_enters[0,0]-mid_point_pos[0]),(palet_enters[0,1]-mid_point_pos[1])),
                                    np.hypot((palet_enters[1,0]-mid_point_pos[0]),(palet_enters[1,1]-mid_point_pos[1]))]

                palet_index = np.argmin(palet_enter_dists)
                first_action = False

            drive_speed_actuator.ctrl ,steering_angle_actuator.ctrl = palet_take_or_put_calculations(palet_index, palet_normal, desired_palet_pos, desired_palet_rot, mid_point_pos, mid_point_rotation_angles)
            
            dist_to_main_goal = np.hypot((desired_palet_pos[0]-mid_point_pos[0]),(desired_palet_pos[1]-mid_point_pos[1]))

            if end_action:
                put_palet = False
                move_back = True
                first_action = True
                end_action = False

            if dist_to_main_goal <= 0.3 and put_palet:
                drive_speed_actuator.ctrl = 0
                steering_angle_actuator.ctrl = 0
                fork_height_actuator.ctrl = 0.0
                end_action = True

        if move_back:
            drive_speed_actuator.ctrl = 0.1
            steering_angle_actuator.ctrl = 0

            back_point = palet_enters[palet_index]
            dist_to_main_goal = np.hypot((back_point[0]-mid_point_pos[0]),(back_point[1]-mid_point_pos[1]))

            if dist_to_main_goal <= 0.3:
                drive_speed_actuator.ctrl = 0
                steering_angle_actuator.ctrl = 0

                move_back = False


    if viewer.is_alive:

        viewer.render()  
        
    else:
        break

    mujoco.mj_step(model, data)

viewer.close()
