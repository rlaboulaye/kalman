from rrt_node import RRTNode
from rrt_container import RRTContainer

cont = RRTContainer([2000, 1500], 30)
a = RRTNode([125,93.75])
cont.add_node(a)
b = RRTNode([120,100])
cont.add_node(b)
c = RRTNode([119,120])
print(cont.get_neighbors(c))
