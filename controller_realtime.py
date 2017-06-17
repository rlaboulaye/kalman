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
from rrt_star import RRTStar
from rt_rrt_star import RTRRTStar

from pathfinder import Pathfinder
from path_container import PathContainer
from position_container import PositionContainer

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

    def follow_waypoint(waypoint_position, goal, tag_radius):
        max_force = 2
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
            res = do('where robot')
            robot_dic = json.loads(res)
            res = do('where others')
            others_dic = json.loads(res)
            if ('orientation' in robot_dic):
                robot_direction = robot_dic['orientation']
                robot_position = robot_dic['center']
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
        #do('speed 0 0')

    def approach_waypoint(waypoint_position, goal, tag_radius, path_container, pos_container):
        max_force = 2
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

        while not_in_radius:
            if path_container.is_changed():
                return
            res = do('where robot')
            robot_dic = json.loads(res)
            res = do('where others')
            others_dic = json.loads(res)
            if goal in others_dic:
                goal_position = others_dic[goal]['center']
                goal_position = [round(goal_position[0]), round(goal_position[1])]
                print('SETTING GOAL POSITION')
                pos_container.set_goal_position(goal_position)
                print('FINISHED SETTING GOAL POSITION')
            speed_set = False
            if ('orientation' in robot_dic):
                robot_direction = robot_dic['orientation']
                robot_position = robot_dic['center']
                print('SET ROBOT POSITION')
                pos_container.set_robot_position(robot_position)
                print('FINISHED SETTING ROBOT POSITION')
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

    def solve_maze(search_strategy, field_dim, goal, unit_length):
        res = do('where others')
        others_dic = json.loads(res)
        corner1 = others_dic[goal]['corners'][0]
        corner3 = others_dic[goal]['corners'][2]
        tag_radius = round(math.sqrt(((corner3[0] - corner1[0]) / 2) ** 2 + ((corner3[1] - corner1[1]) / 2) ** 2) * 1.2)
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


        if search_strategy == RTRRTStar:
            time_limit = 0

            searcher = search_strategy(field_dim, time_limit, tag_radius, robot_radius, 50)
            path_container = PathContainer()
            pos_container = PositionContainer(robot_position, obstacle_pos, goal_position)
            pathfinder = Pathfinder(searcher, path_container, pos_container, tag_radius)
            pathfinder.start()

            res = do('where others')
            others_dic = json.loads(res)
            corner1 = others_dic[goal]['corners'][0]
            corner3 = others_dic[goal]['corners'][2]
            tag_radius = round(math.sqrt(((corner3[0] - corner1[0]) / 2) ** 2 + ((corner3[1] - corner1[1]) / 2) ** 2) * 1.2)
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

#            path = searcher.get_path(robot_position, obstacle_pos, goal_position, 10)
#            approach_waypoint(path[1], goal, tag_radius)
            not_at_goal = True
            while not_at_goal:
                path = path_container.get_path()
                if path == None:
                    continue
                for waypoint in path:
                    approach_waypoint(waypoint, goal, tag_radius, path_container, pos_container)
                    if path_container.is_changed():
                        break
                distance_to_goal = math.sqrt((goal_position[0] - robot_position[0]) ** 2 + (goal_position[1] - robot_position[1]) ** 2)
                not_at_goal = distance_to_goal > tag_radius * 2

        else:
            searcher = search_strategy(field_dim, tag_radius, robot_radius)
            waypoints = searcher.get_path(robot_position, goal_position, obstacle_pos, unit_length)
            for waypoint in waypoints:
                follow_waypoint(waypoint, goal, tag_radius)

        do('speed 0 0')
        pathfinder.join()

    do('param kp 25')
    do('param ki .5')
    do('param kd .5')

    field_dim = [1920, 1080]
    goal = '42'
    unit_length = 30
    # search_strategy = RRTSearch
    search_strategy = RTRRTStar
    # search_strategy = RRTStar

    solve_maze(search_strategy, field_dim, goal, unit_length)

    writer.close()

if __name__ == '__main__':
    main(*argv[1:])
