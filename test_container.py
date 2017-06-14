from rrt_node import RRTNode
from rrt_container import RRTContainer

cont = RRTContainer([1024, 1024], 30)
a = RRTNode([100,100])
cont.add_node(a)
b = RRTNode([200,200])
cont.add_node(b)
c = RRTNode([1000,1000])
print(cont.get_nearest_node(c))
