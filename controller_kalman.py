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
from kalman import Kalman

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

    def follow_waypoint(waypoint_position, goal, tag_radius, kalman_filter, delta_t):
        max_force = 5
        waypoint_radius = tag_radius * 2
        res = do('where others')
        others_dic = json.loads(res)
        del others_dic[goal]
        del others_dic['time']
        field_dic = {}
        waypoint = 'waypoint'
        field_dic[waypoint] = af(waypoint_radius, 10, max_force, True)
        #for obstacle_key in others_dic:
            #field_dic[obstacle_key] = rf(tag_radius, 100, max_force, True)
        ws = WheelSpeed(float(max_force))

        not_in_radius = True

        while(not_in_radius):
            start_time = time()
            res = do('where robot')
            robot_dic = json.loads(res)
            res = do('where others')
            others_dic = json.loads(res)
            res = do('speed')
            speed_dic = json.loads(res)
            if ('orientation' in robot_dic):
                robot_direction = robot_dic['orientation']
                robot_position = robot_dic['center']
                print()
                print('observed: ', robot_position)
                average_speed = (speed_dic['speed_a'] + speed_dic['speed_b']) / 2
                robot_direction_unit = np.array(robot_direction) / np.linalg.norm(np.array(robot_direction))
                speed = average_speed * robot_direction_unit
                robot_position = kalman_filter.get_position(robot_position, speed)
                print('predicted: ', robot_position)
                force = [0, 0]
                force = np.add(force, field_dic[waypoint].get_vector(robot_position, waypoint_position))
                for tag_key in others_dic:
                    if (tag_key != 'time' and tag_key in field_dic):
                        tag_position = others_dic[tag_key]['center']
                        field = field_dic[tag_key]
                        force = np.add(force, field.get_vector(robot_position, tag_position))
                speed = ws.get_wheel_speed(force)
                ws.adjust_speed_for_rotation(speed, robot_direction, force)
                do('speed ' + str(speed[0]) + ' '+ str(speed[1]))
                distance_to_goal = math.sqrt((waypoint_position[0] - robot_position[0]) ** 2 + (waypoint_position[1] - robot_position[1]) ** 2)
                not_in_radius = distance_to_goal > waypoint_radius
                elapsed_time = time() - start_time
                time_diff = delta_t - elapsed_time
                if (time_diff > 0):
                    sleep(time_diff)

    def solve_maze(search_strategy, field_dim, goal, unit_length):
        res = do('where others')
        others_dic = json.loads(res)
        corner1 = others_dic[goal]['corners'][0]
        corner3 = others_dic[goal]['corners'][2]
        tag_radius = round(math.sqrt(((corner3[0] - corner1[0]) / 2) ** 2 + ((corner3[1] - corner1[1]) / 2) ** 2) * 1.5)
        robot_radius = tag_radius

        robot_dic = {}
        while (not 'orientation' in robot_dic):
            res = do('where robot')
            robot_dic = json.loads(res)
        robot_position = robot_dic['center']
        robot_position = [round(robot_position[0]), round(robot_position[1])]

        others_dic = {}
        while (not goal in others_dic):
            res = do('where others')
            others_dic = json.loads(res)
        goal_position = others_dic[goal]['center']
        goal_position = [round(goal_position[0]), round(goal_position[1])]
        del others_dic[goal]
        del others_dic['time']
        obstacle_pos = []
        for obstacle_key in others_dic:
            center = others_dic[obstacle_key]['center']
            obstacle_pos.append([round(center[0]), round(center[1])])
        searcher = search_strategy(field_dim, tag_radius, robot_radius)
        waypoints = searcher.get_path(robot_position, goal_position, obstacle_pos, unit_length)

        delta_t = .205
        lag_multiplier = 4
        kalman_filter = Kalman(delta_t, lag_multiplier, robot_position)

        for waypoint in waypoints:
            follow_waypoint(waypoint, goal, tag_radius, kalman_filter, delta_t)

        do('speed 0 0')

    do('param kp 25')
    do('param ki .5')
    do('param kd .5')

    field_dim = [1920, 1080]
    goal = '6'
    unit_length = 50
    search_strategy = AStar

    solve_maze(search_strategy, field_dim, goal, unit_length)

    writer.close()

if __name__ == '__main__':
    main(*argv[1:])
