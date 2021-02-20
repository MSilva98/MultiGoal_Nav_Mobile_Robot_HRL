"""collect_distances controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot, Supervisor, Motor, DistanceSensor
import time

all_dists = [4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30]

# create the Robot instance.
# Use supervisor to set robot position
supervisor = Supervisor()
robot_node = supervisor.getFromDef("epuck")
translation_field = robot_node.getField("translation")
initial_pose = translation_field.getSFVec3f()   # ROBOT IS 4 cm away from WALL

# get the time step of the current world.
timestep = 32

# Enable front sharp sensor
front_sensor = supervisor.getDistanceSensor("sharps0")
front_sensor.enable(timestep)

# Main loop:
# - perform simulation steps until Webots is stopping the controller
distances = open("distances.txt", "w+")
j = 0
while supervisor.step(timestep) != -1:
    print("Measure ", j)
    val = front_sensor.getValue()
    s = "\nDistance: " + str(all_dists[j]) + " Front Sharp Val: " + str(val)
    distances.write(s)
    j += 1
    if j >= len(all_dists):
        break

    atual_pose = initial_pose.copy()
    atual_pose[0] -= (all_dists[j]-4)/100  # Convert distance to cm
    translation_field.setSFVec3f(atual_pose)
    time.sleep(1)   

# Enter here exit cleanup code.
distances.close()