uint64 id

uint8 CUBOID = 0
uint8 CYLINDER = 1
# cuboid or cylinder
uint8 type

uint8 RED = 0
uint8 BLUE = 1
uint8 YELLOW = 2
uint8 GREEN = 3

uint8 color # one of the four colors defined above

float64 width 
float64 depth
float64 height

geometry_msgs/PoseStamped pose