import asyncio
import json
from sys import argv
from time import time, sleep
import math
import numpy as np

from attractive_field import AttractiveField as af
from repulsive_field import RepulsiveField as rf
from tangential_field import TangentialField as tf
from random_field import RandomField as ranf
from wheel_speed import WheelSpeed
from a_star import AStar
from rrt_search import RRTSearch
from rt_rrt_star import RTRRTStar
from pathfinder import Pathfinder
from path_container import PathContainer
from position_container import PositionContainer
from kalman import Kalman
from extended_kalman import ExtendedKalman

def main(host, port):
    loop = asyncio.get_event_loop()
    reader, writer = loop.run_until_complete(
            asyncio.open_connection(host, port))
    print(reader.readline())

    def do(command):
        print('>>>', command)
        writer.write(command.strip().encode())
        res = loop.run_until_complete(reader.readline()).decode().strip()
        print('<<<', res)
        print()
        return res

    def follow_goal(goal, delta_t, lag_multiplier, speed_multiplier, y_adj):
        robot_dic = {}
        while (not 'orientation' in robot_dic):
            res = do('where robot')
            robot_dic = json.loads(res)
        robot_position = robot_dic['center']
        robot_position = [robot_position[0], y_adj - robot_position[1]]
        robot_direction = robot_dic['orientation']
        robot_direction = [robot_direction[0], -1 * robot_direction[1]]
        robot_angle = (math.atan2(robot_direction[1], robot_direction[0]) + 2 * math.pi) % (2 * math.pi)

        others_dic = {}
        while (not goal in others_dic):
            res = do('where others')
            others_dic = json.loads(res)
        goal_position = others_dic[goal]['center']
        goal_position = [goal_position[0], y_adj - goal_position[1]]
        corner1 = others_dic[goal]['corners'][0]
        corner2 = others_dic[goal]['corners'][1]
        corner3 = others_dic[goal]['corners'][2]
        tag_radius = round(math.sqrt(((corner3[0] - corner1[0]) / 2) ** 2 + ((corner3[1] - corner1[1]) / 2) ** 2) * 1.5)
        wheel_distance = math.sqrt((corner2[0] - corner1[0]) ** 2 + (corner2[1] - corner1[1]) ** 2)

        max_force = 5
        goal_radius = tag_radius * 2
        goal_field = af(goal_radius, 10, max_force, True)
        ws = WheelSpeed(float(max_force))
        not_in_radius = True

        kalman_robot = ExtendedKalman(delta_t, lag_multiplier, robot_position, robot_angle, wheel_distance)
        kalman_goal = Kalman(delta_t, lag_multiplier, goal_position, pos_only=True)

        while(not_in_radius):
            start_time = time()
            res = do('where robot')
            robot_dic = json.loads(res)
            res = do('where others')
            others_dic = json.loads(res)
            res = do('speed')
            speed_dic = json.loads(res)
            if ('orientation' in robot_dic and goal in others_dic):

                robot_direction = robot_dic['orientation']
                robot_direction = [robot_direction[0], -1 * robot_direction[1]]
                robot_angle = math.atan2(robot_direction[1], robot_direction[0])
                robot_angle = robot_angle % (2 * math.pi)
                robot_position = robot_dic['center']
                robot_position = [robot_position[0], y_adj - robot_position[1]]
                speed = [speed_dic['speed_b'] * speed_multiplier, speed_dic['speed_a'] * speed_multiplier]
                robot_position, robot_angle = kalman_robot.get_position(robot_position, speed, robot_angle)
                robot_direction = [math.cos(robot_angle), math.sin(robot_angle)]

                goal_position = others_dic[goal]['center']
                goal_position = [goal_position[0], y_adj - goal_position[1]]
                goal_position = kalman_goal.get_position(goal_position)

                force = [0, 0]
                force = np.add(force, goal_field.get_vector(robot_position, goal_position))
                speed = ws.get_wheel_speed(force)
                ws.adjust_speed_for_rotation(speed, robot_direction, force)
                do('speed ' + str(speed[0]) + ' '+ str(speed[1]))
                distance_to_goal = math.sqrt((goal_position[0] - robot_position[0]) ** 2 + (goal_position[1] - robot_position[1]) ** 2)
                not_in_radius = distance_to_goal > goal_radius
                elapsed_time = time() - start_time
                time_diff = delta_t - elapsed_time
                if (time_diff > 0):
                    sleep(time_diff)
        do('speed 0 0')


    def follow_waypoints(goal, delta_t, lag_multiplier, speed_multiplier, field_dim, y_adj):
        robot_dic = {}
        while (not 'orientation' in robot_dic):
            res = do('where robot')
            robot_dic = json.loads(res)
        robot_position = robot_dic['center']
        robot_position = [robot_position[0], y_adj - robot_position[1]]
        robot_direction = robot_dic['orientation']
        robot_direction = [robot_direction[0], -1 * robot_direction[1]]
        robot_angle = (math.atan2(robot_direction[1], robot_direction[0]) + 2 * math.pi) % (2 * math.pi)

        others_dic = {}
        while (not goal in others_dic):
            res = do('where others')
            others_dic = json.loads(res)
        goal_position = others_dic[goal]['center']
        goal_position = [goal_position[0], y_adj - goal_position[1]]
        corner1 = others_dic[goal]['corners'][0]
        corner2 = others_dic[goal]['corners'][1]
        corner3 = others_dic[goal]['corners'][2]
        tag_radius = round(math.sqrt(((corner3[0] - corner1[0]) / 2) ** 2 + ((corner3[1] - corner1[1]) / 2) ** 2) * 1.5)
        wheel_distance = math.sqrt((corner2[0] - corner1[0]) ** 2 + (corner2[1] - corner1[1]) ** 2)
        del others_dic[goal]
        del others_dic['time']
        obstacle_pos = []
        for obstacle_key in others_dic:
            center = others_dic[obstacle_key]['center']
            obstacle_pos.append([round(center[0]), round(center[1])])

        max_force = 5
        goal_radius = tag_radius * 2
        goal_field = af(goal_radius, 10, max_force, True)
        ws = WheelSpeed(float(max_force))
        not_in_radius = True

        kalman_robot = ExtendedKalman(delta_t, lag_multiplier, robot_position, robot_angle, wheel_distance)
        kalman_goal = Kalman(delta_t, lag_multiplier, goal_position, pos_only=True)
        
        time_limit = 0.05
        robot_radius = tag_radius
        searcher = RTRRTStar(field_dim, time_limit, tag_radius, robot_radius, 50)
        path_container = PathContainer()
        pos_container = PositionContainer(robot_position, obstacle_pos, goal_position)
        pathfinder = Pathfinder(searcher, path_container, pos_container, tag_radius)
        pathfinder.start()

        while(not_in_radius):
            path = path_container.get_path()
            if path == None:
                continue
            current_waypoint = 1
            while (not path_container.is_changed()) and (not_in_radius):
                start_time = time()
                distance_to_waypoint = math.sqrt((path[current_waypoint][0] - robot_position[0]) ** 2 + (path[current_waypoint][1] - robot_position[1]) ** 2)
                if (distance_to_waypoint < tag_radius * 2):
                    current_waypoint += 1
                res = do('where robot')
                robot_dic = json.loads(res)
                res = do('where others')
                others_dic = json.loads(res)
                res = do('speed')
                speed_dic = json.loads(res)
                if ('orientation' in robot_dic and goal in others_dic):

                    robot_direction = robot_dic['orientation']
                    robot_direction = [robot_direction[0], -1 * robot_direction[1]]
                    robot_angle = math.atan2(robot_direction[1], robot_direction[0])
                    robot_angle = robot_angle % (2 * math.pi)
                    robot_position = robot_dic['center']
                    robot_position = [robot_position[0], y_adj - robot_position[1]]
                    speed = [speed_dic['speed_b'] * speed_multiplier, speed_dic['speed_a'] * speed_multiplier]
                    robot_position = kalman_robot.get_position(robot_position, speed, robot_angle)
                    pos_container.set_robot_position(robot_position)

                    goal_position = others_dic[goal]['center']
                    goal_position = [goal_position[0], y_adj - goal_position[1]]
                    goal_position = kalman_goal.get_position(goal_position)
                    pos_container.set_goal_position(goal_position)

                    force = [0, 0]
                    force = np.add(force, goal_field.get_vector(robot_position, path[current_waypoint]))
                    speed = ws.get_wheel_speed(force)
                    ws.adjust_speed_for_rotation(speed, robot_direction, force)
                    do('speed ' + str(speed[0]) + ' '+ str(speed[1]))
                    distance_to_goal = math.sqrt((goal_position[0] - robot_position[0]) ** 2 + (goal_position[1] - robot_position[1]) ** 2)
                    not_in_radius = distance_to_goal > goal_radius
                    elapsed_time = time() - start_time
                    time_diff = delta_t - elapsed_time
                    if (time_diff > 0):
                        sleep(time_diff)
        do('speed 0 0')
        pathfinder.join()


    do('param kp 25')
    do('param ki .5')
    do('param kd .5')

    field_dim = [1920, 1080]
    goal = '30'

    delta_t = .205
    lag_multiplier = 4
    speed_multiplier = 16
    follow_waypoints(goal, delta_t, lag_multiplier, speed_multiplier, field_dim, field_dim[1])
    # follow_goal(goal, delta_t, lag_multiplier, speed_multiplier, field_dim[1])

    writer.close()

if __name__ == '__main__':
    main(*argv[1:])
