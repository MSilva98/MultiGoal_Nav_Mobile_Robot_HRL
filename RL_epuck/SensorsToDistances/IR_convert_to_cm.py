import numpy as np
import matplotlib.pyplot as plt
import math

# https://www.limulo.net/website/coding/physical-computing/sharp-linearization.html

# Compute this values - ver resolução dos sensores
t_distances_cm = [4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30]
# Values obtained using webots epuck with this sensor (Sharp GP2Y0A41SK0F) placed in the front of the robot
# Sensor as an offset of 3 cm from center of robot which was considered
voltageToSensor = [
    2.72363985807244550,
    2.33064299447017800,
    2.01163620550354770,
    1.76279208674806800,
    1.54953647664867520,
    1.39826862073564180,
    1.25167019679126960,
    1.14608103795107770,
    1.04034567972324490,
    0.98043362108076050,
    0.91966183668049580,
    0.86982640519434720,
    0.81980100391153960,
    0.77553951331585660,
    0.73035334911931140,
    0.69521961172835530,
    0.66013465735442570,
    0.63492896404666500,
    0.60803073169123510,
    0.58168295165894060,
    0.55526297463720600,
    0.52975206365775220,
    0.50760314465567970,
    0.48602688395622545,
    0.46411634445201017,
    0.44163426907554390,
    0.42053634666121487
]

# New read for the exact same distances
validation_data = [
    2.69430313587892250,
    2.33082703084202900,
    2.01208034853745400,
    1.76168061171254190,
    1.55153053849958990,
    1.39931617594750410,
    1.25077356097593720,
    1.14607385161515430,
    1.04155141308313580,
    0.98018342266805000,
    0.91930724568072060,
    0.87072634342374070,
    0.81913392271222060,
    0.77539116849275550,
    0.73031292213313730,
    0.69548232163001140,
    0.65987095159382600,
    0.63344378166039280,
    0.60933098229784580,
    0.58254544738418050,
    0.55613973148506320,
    0.53031000755741450,
    0.50743924732996970,
    0.48630393908369324,
    0.46372312716770170,
    0.44178654550102275,
    0.41960235198206236
]

# Voltage curve
# fig1, plt1 = plt.subplots()
# plt1.plot(t_distances_cm, voltageToSensor, 'k')
# plt1.set(xlabel='Distance (cm)', ylabel='Voltage (V)', title='Estimation data')   
# plt.show()

# Inversion of distances
# For sensor values between max and 0.55
k = 0.7
# For lower values than 0.55 and higher than 0.48
# k = 1.2
# For lower values than 0.48
# k = 1.4

inv_d = [1/x for x in t_distances_cm]
inv_d_k = [1/(x+k) for x in t_distances_cm]
# fig2, plt2 = plt.subplots()
# plt2.plot(inv_d, voltageToSensor, 'r*-')
# plt2.plot(inv_d_k, voltageToSensor, 'b*-')
# plt2.set(xlabel='Inverse Distance (1/cm)', ylabel='Voltage (V)', title='Inverted Distances, Constant: ' + str(k))
# plt2.legend(['Without Corrective Constant', 'With Corrective Constant'], loc='lower right')
# plt.show()

# Linearization
coef = np.polyfit(inv_d_k, voltageToSensor, 1)
# x = np.linspace(0.02, 0.255, 100)
# y = np.polyval(coef, x)
# fig3,plt3 = plt.subplots()
# plt3.plot(x, y, 'k:')
# plt3.plot(inv_d_k, voltageToSensor, 'rx-')
# plt3.set(xlabel='Inverse Distance (1/cm)', ylabel='Voltage (V)', title='Polynomial Validation')
# plt.show()


print("Equation: " + str(coef[0]) + "/(voltage- " + str(coef[1]) + ")-" + str(k))
def getDistance(voltage):
    d = coef[0]/(voltage-coef[1])-k
    return d if d <= 30 else 30

# https://cyberbotics.com/doc/guide/distancesensor-sensors#sharp-gp2y0a41sk0f
# Model of sensor - Sharp GP2Y0A41SK0F
# Range from 4 to 30 cm
# Equations
# Convert meters to voltage: y(x) = 0.5131*x^(-0.5735)-0.6143
# Convert voltage to meters: y(x) = 0.1594*x^(-0.8533)-0.02916
def getDistanceWebots(voltage):
    d = ((0.1594*pow(voltage,-0.8533))-0.02916)*100 # *100 -> m to cm
    return d if d <= 30 else 30
    
# Validation of equation
m_distances = [getDistance(x) for x in voltageToSensor]
m_distances_val = [getDistance(x) for x in validation_data]
# fig4, plt4 = plt.subplots()
# plt4.plot(t_distances_cm, m_distances, "g")
# plt4.plot(t_distances_cm, m_distances_val, "bx")
# plt4.plot(t_distances_cm, t_distances_cm, "k:")
# plt4.set(xlabel='Real Distances of sensor (cm)', ylabel='Estimated distances (cm)', title='Validation')
# plt4.legend(["Estimated", "Estimated Val", "Webots", "Webots Val", "X=Y"], loc="upper left")

# Comparison to Webots equation (Both very similar, mine is a little better)
m_distancesWebots = [getDistanceWebots(x) for x in voltageToSensor]
fig, plt5 = plt.subplots()
plt5.plot(t_distances_cm, m_distances, "r")
# plt5.plot(t_distances_cm, m_distancesWebots, "b")
plt5.plot(t_distances_cm, t_distances_cm, "k:")
plt5.set(xlabel='Real Distances of sensor (cm)', ylabel='Estimated distances (cm)', title='Estimated vs Webots')
# plt5.legend(["Estimated", "Webots", "X=Y"], loc="upper left")
plt.tight_layout()
plt.show()

for x in voltageToSensor:
    print(getDistance(x), x)
print("\n")
for x in validation_data:
    print(getDistance(x))

# Values from 6 sensors, aligned at 0 degrees, 5 cm to the left from center
# Each measures split by a 0 value, sensors in order from sharp0 to sharp5
# 10 reads in same spot
six_sensors = [
    [0.30943340616967363, 1.134588184556104, 1.7697675298605458, 0.5238705733109741, 0.7690031875769673, 0.3099241672096078],
    [0.3084871536809765, 1.1225077933865348, 1.76007408274599, 0.5254312522950505, 0.7758388242447617, 0.30991336011781023],
    [0.3102334698597219, 1.1329294072058884, 1.762005461994565, 0.5251324350199261, 0.7668885371657969, 0.30945357302230414],
    [0.3094906215081552, 1.1257347609004256, 1.7506112120767445, 0.5233177098056421, 0.7733262442501286, 0.3109677615437699],
    [0.30932460394742345, 1.1348421074586255, 1.7591285378452912, 0.5263142807871717, 0.7759358880370124, 0.3124979160964847],
    [0.3079693055110142, 1.1215584012107969, 1.767331740105161, 0.5164259015002256, 0.7717161828374463, 0.30904237479941477],
    [0.3101562493794013, 1.1239583123157042, 1.762183590003747, 0.5237268815785255, 0.7744436208520675, 0.30966588064642264],
    [0.30939207254987927, 1.132861590047858, 1.750396324130342, 0.5265902496614587, 0.7694565980619167, 0.3102692268453508],
    [0.3093951864019828, 1.135037143950305, 1.7591208963264042, 0.5270736450579886, 0.7786640273460418, 0.3104332469835587],
    [0.3123814794370194, 1.1329113699406557, 1.7610863863443003, 0.5210380961323234, 0.7702866251349426, 0.3065092105264945]
]

# print("\nVALUES FROM All Sensors")
# names = ["F", "FL", "L", "FR", "R", "B"]
# for x in six_sensors:
#     print("\n")
#     for v in x:
#         print(names[x.index(v)] + ":\t" + str(getDistance(v)))
