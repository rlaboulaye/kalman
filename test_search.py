from rt_rrt_star import RTRRTStar

field_dim = [1920, 1080]
time_limit = 0.1
tag_radius = 50
robot_radius = 50
robot_initial_pos = [50 , 50]

searcher = RTRRTStar(field_dim, time_limit, tag_radius, robot_radius, robot_initial_pos)

#robot_pos = [50 , 50]
obstacle_pos = []
obstacle_pos.append([100  , 100])
obstacle_pos.append([150  , 789])
obstacle_pos.append([346  , 924])
obstacle_pos.append([1750 , 245])
obstacle_pos.append([1349 , 632])
obstacle_pos.append([890  , 422])
obstacle_pos.append([789  , 983])
obstacle_pos.append([1589 , 238])
obstacle_pos.append([1783 , 920])
obstacle_pos.append([1102 , 131])
goal_pos = [1870 , 1030]

path = searcher.get_path(robot_initial_pos, obstacle_pos, goal_pos)
print(path)
