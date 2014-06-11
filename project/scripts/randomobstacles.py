#!/usr/bin/env python
'''
A Script to generate a world file to be included that has obstacles positioned
randomly.

Instructions:
The robot environment is to be 10 meters by 10 meters. The environment
shall contain five objects that are randomly placed in the environment when the
simulation is started. In other words, the position of the objects in the
environment is to change every time the simulation is started. The five objects
are to be of the following sizes: 50mm (h) x 50mm (w) x 50mm (d); 100mm x 100mm
x 100mm; 275mm x 50mm x 100mm; 80mm x 250mm x 100mm; 300mm x 300mm x 300mm.
Your robot should be able to detect all objects in the environment, but it is
not to be programmed for these specific object sizes.

Usage:
    ./randomobstacles output_file
'''
import random
import sys
import roslib; roslib.load_manifest('project')
import rospy

# All units are in mm
ROBOT_ORIGIN = [0,0]
#ROBOT_DIM = [455,381] # From Pioneer 3-DX spec sheet
ROBOT_DIM = [511,400] # From Pioneer 3-DX stage model
ENV_DIM = [10000, 10000] # -5000, 5000

OBJECTS_DIM = [ # Note that the ordering is different form the instructions
        [50, 50, 50],
        [100, 100, 100],
        [50, 100, 275],
        [250, 100, 80],
        [300, 300, 300]
        ]

def rand_satisfy(rng, constr=None):
    """Generate a random number within the given range but outside the
    constraint area."""

    while True:
        num = random.randint(*rng)
        if not constr:
            return num
        elif num < constr[0] or num > constr[1]:
            return num

def get_random_position(obj_dim):
    """Generates a random position. The constraints are that it is within the
    environment, and it's not colliding with the robot."""
    padding = 100

    # For simplicity, we will set a bounding box equal to the largest side of
    # the robot. Objects won't be created in this box.
    max_robot_dim = max(ROBOT_DIM)

    # Shift the range to go from 0 to 10000 instead of -5000 to 5000 for easier
    # generation of random numbers. We will then subtract 5000 from the result
    constr_x = [ENV_DIM[0]/2 + ROBOT_ORIGIN[0] - max_robot_dim/2 - obj_dim[0] - padding,
                ENV_DIM[0]/2 + ROBOT_ORIGIN[0] + max_robot_dim/2 + obj_dim[0] + padding]

    constr_y = [ENV_DIM[1]/2 + ROBOT_ORIGIN[1] - max_robot_dim/2 - obj_dim[1] - padding,
                ENV_DIM[1]/2 + ROBOT_ORIGIN[1] + max_robot_dim/2 + obj_dim[1] + padding]

    #print constr_x, constr_y
    # We will also adjust the range from which the random number by the size of
    # the object/obstacle
    rng_x = [0 + obj_dim[0], ENV_DIM[0] - obj_dim[0]]
    rng_y = [0 + obj_dim[1], ENV_DIM[1] - obj_dim[1]]

    x_pos = rand_satisfy(rng_x)
    if x_pos < constr_x[0] or x_pos > constr_x[1]:
        constr_y = None
    y_pos = rand_satisfy(rng_y, constr_y)
    # We finally subtract the offset
    x_pos = x_pos - ENV_DIM[0]/2
    y_pos = y_pos - ENV_DIM[1]/2

    return [x_pos, y_pos]


#class Object():
#    """Class to hold object position and dimensions"""
#    def __init__(self, dim, pos):
#        self.dim = dim
#        self.pos = pos

#    def collides(self, obj)
#        """Check if two objects collide"""


def gen_obstacles():
    """Generates obstacles and makes sure they don't collide with each other"""
    obj_pos = []
    for obj in OBJECTS_DIM:
        pos = get_random_position(obj)
        obj_pos.append(pos)

    return obj_pos

def gen_world_file(file_path, obj_pos):
    assert len(obj_pos) == len(OBJECTS_DIM)
    object_def = '''define object model
(
  shape "square"
  color "blue"
  ranger_return 1
)
'''
    object_templ = '''object
(
  size [%f %f %f]
  pose [%f %f 0 0]
)
'''
    # Make a list of arguments for the template
    args = [size+pose for size, pose in zip(OBJECTS_DIM,obj_pos)]

    # Convert to meters
    for v in args:
        v[:] = [i/1000.0 for i in v]

    # Start with object def
    objects = [object_templ % tuple(v) for v in args]

    output = object_def + "\n".join(objects)

    with open(file_path, 'w') as fp:
        fp.write(output)

def main(args):
    """Handles user input and calls appropriate functions"""
    if len(args) != 2 or args[1] == '-h':
        print(__doc__)
        sys.exit(0)
    else:
        obj_pos = gen_obstacles()
        gen_world_file(sys.argv[1], obj_pos)

if __name__ == '__main__':
    rospy.init_node("randomobs")
    main(rospy.myargv(argv=sys.argv))
