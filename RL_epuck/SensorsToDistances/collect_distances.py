"""collect_distances controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot, Supervisor, Motor, DistanceSensor
import time

all_dists = [4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30]
# Front sensor is 3 cm closer to wall than robot center
front_sensor_offset = -3
robot_pose = [x-front_sensor_offset for x in all_dists]
print(robot_pose)

# create the Robot instance.
# Use supervisor to set robot position
supervisor = Supervisor()
robot_node = supervisor.getFromDef("epuck")
translation_field = robot_node.getField("translation")
# Tested in QLearning_giant_corridor.wbt
# Robot with translation (0,0,4.93) and with rotation (0,1,0,3.14)
initial_pose = translation_field.getSFVec3f()   # ROBOT IS 4.3 cm away from WALL

# get the time step of the current world.
timestep = 32

# Enable front sharp sensor
front_sensor = supervisor.getDistanceSensor("sharps0")
front_sensor.enable(timestep)

# initialize devices
ds = []
dsNames = [
    'sharps0',  # front
    'sharps1',  # front left
    'sharps2',  # left
    'sharps3',  # front right
    'sharps4',  # right
    'sharps5'   # rear
]
# enable sharp distance sensors
for i in range(len(dsNames)):
    ds.append(supervisor.getDistanceSensor(dsNames[i]))
    ds[i].enable(timestep)


# Main loop:
# - perform simulation steps until Webots is stopping the controller
distances = open("distancesToSensor_all.txt", "w+")
# distances = open("distancesToSensor_validation.txt", "w+")
j = 0
i = 0
val = 0
while supervisor.step(timestep) != -1:
    # # COLLECT 10 DATA SAMPLES FROM ALL SENSORS IN SAME POSIITON
    # distances.write("\n")
    # for i in range(len(ds)):
    #     distances.write("\n"+str(ds[i].getValue()))
    # j += 1
    # if j >= 10:
    #     break

    # COLLECT DATA FROM FRONT SENSOR ONLY
    if i >= 0:
        print("Measure " + str(j) + " distance: " + str(all_dists[i]) + " robot_pose: " + str(robot_pose[i]))
        val += front_sensor.getValue()
        j += 1
    else:
        j = 30
    if j >= 30: # get mean of 20 values for each distance
        if i >= 0:
            s = "\nDistance: " + str(all_dists[i]) + " Front Sharp Val: " + str(val/30)
            distances.write(s)
        val = 0
        j = 0
        i += 1
        if i >= len(all_dists):
            break
        actual_pose = initial_pose.copy()
        actual_pose[2] -= i/100  # move 1 cm back
        translation_field.setSFVec3f(actual_pose)
    atual_pose = initial_pose.copy()
    atual_pose[0] -= (all_dists[j]-4)/100  # Convert distance to cm
    translation_field.setSFVec3f(atual_pose)
    time.sleep(1)   

# Enter here exit cleanup code.
distances.close()