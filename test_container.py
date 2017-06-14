from rrt_node import RRTNode
from rrt_container import RRTContainer

cont = RRTContainer([1024, 1024], 30)
a = RRTNode([255,10])
cont.add_node(a)
b = RRTNode([260,10])
cont.add_node(b)
c = RRTNode([257,10])
print(cont.get_nearest_node(c).pos)
