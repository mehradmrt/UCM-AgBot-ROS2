# Poses1 includes all the midpoints and rotations for all detected targets.
# e.g Poses2 includes same midpoints but with new rotations for all detected targets (could be
# 45 degrees CC rotation around the normal vector of the targets calculated in Poses1)

std_msgs/Header header
geometry_msgs/PoseArray[] Poses1
geometry_msgs/PoseArray[] Poses2
geometry_msgs/PoseArray[] Poses3
geometry_msgs/PoseArray[] Poses4
geometry_msgs/PoseArray[] Poses5

