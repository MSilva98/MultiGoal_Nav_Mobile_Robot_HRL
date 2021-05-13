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
fig1, plt1 = plt.subplots()
plt1.plot(t_distances_cm, voltageToSensor, 'k')
plt1.set(xlabel='Distance (cm)', ylabel='Voltage (V)', title='Raw Voltage to Distance')   

# Inversion of distances
# Different constants used to get the most linear plot possible
# For sensor values between max and 0.55
k = 0.7
# For lower values than 0.55 and higher than 0.48
k = 1.2
# For lower values than 0.48
k = 1.4

inv_d = [1/x for x in t_distances_cm]
inv_d_k = [1/(x+k) for x in t_distances_cm]
fig2, plt2 = plt.subplots()
plt2.plot(inv_d, voltageToSensor, 'r*-')
plt2.plot(inv_d_k, voltageToSensor, 'b*-')
plt2.set(xlabel='Inverse Distance (1/cm)', ylabel='Voltage (V)', title='Inverted Distances with Constant: ' + str(k))
plt2.legend(['Without Corrective Constant', 'With Corrective Constant'], loc='lower right')

# Linearization
coef = np.polyfit(inv_d_k, voltageToSensor, 1)
x = np.linspace(0.02, 0.255, 100)
y = np.polyval(coef, x)
fig3,plt3 = plt.subplots()
plt3.plot(x, y, 'k:')
plt3.plot(inv_d_k, voltageToSensor, 'rx-')
plt3.set(xlabel='Inverse Distance (1/cm)', ylabel='Voltage (V)', title='Polynomial Validation with Constant: ' + str(k))

print("Equation: " + str(coef[0]) + "/(voltage- " + str(coef[1]) + ")-" + str(k))
def getDistance(voltage):
    return coef[0]/(voltage-coef[1])-k

# https://cyberbotics.com/doc/guide/distancesensor-sensors#sharp-gp2y0a41sk0f
# Model of sensor - Sharp GP2Y0A41SK0F
# Range from 4 to 30 cm
# Equations
# Convert meters to voltage: y(x) = 0.5131*x^(-0.5735)-0.6143
# Convert voltage to meters: y(x) = 0.1594*x^(-0.8533)-0.02916
def getDistanceWebots(voltage):
    return ((0.1594*pow(voltage,-0.8533))-0.02916)*100 # *100 -> m to cm
    
# Validation of equation
m_distances = [getDistance(x) for x in voltageToSensor]
m_distances_val = [getDistance(x) for x in validation_data]
fig4, plt4 = plt.subplots()
plt4.plot(t_distances_cm, m_distances, "g")
plt4.plot(t_distances_cm, m_distances_val, "bx")
plt4.plot(t_distances_cm, t_distances_cm, "k:")
plt4.set(xlabel='Real Distances of sensor (cm)', ylabel='Estimated distances (cm)', title='Equation Validation with Constant: ' + str(k))
plt4.legend(["Estimated", "Estimated Val", "Webots", "Webots Val", "X=Y"], loc="upper left")

# Comparison to Webots equation (Both very similar, mine is a little better)
m_distancesWebots = [getDistanceWebots(x) for x in voltageToSensor]
fig, plt5 = plt.subplots()
plt5.plot(t_distances_cm, m_distances, "r")
plt5.plot(t_distances_cm, m_distancesWebots, "b")
plt5.plot(t_distances_cm, t_distances_cm, "k:")
plt5.set(xlabel='Real Distances of sensor (cm)', ylabel='Estimated distances (cm)', title='Estimated equation with constant ' + str(k) + ' vs Webots equations')
plt5.legend(["Estimated", "Webots", "X=Y"], loc="upper left")


# PLOT all 3 equations simultaneously
def sensorVoltageToDistance(voltage):
    if voltage <= 0.48:
        d = (15.065187821049603/(voltage+0.04822905725919005)-1.4)
    elif voltage > 0.48 and voltage <= 0.53:
        d = (14.483674005107527/(voltage+0.02681880954262667)-1.2)
    else:
        d = (13.045778715415159/(voltage-0.028295530064741125)-0.7)
    
    return d if d <= 30 else 30

def eq_k14(voltage):
    return (15.065187821049603/(voltage+0.04822905725919005)-1.4)

def eq_k12(voltage):
    return (14.483674005107527/(voltage+0.02681880954262667)-1.2)

def eq_k07(voltage):
    return (13.045778715415159/(voltage-0.028295530064741125)-0.7)

all_vals = [sensorVoltageToDistance(x) for x in validation_data]
eqk14 = [eq_k14(x) for x in validation_data]
eqk12 = [eq_k12(x) for x in validation_data]
eqk07 = [eq_k07(x) for x in validation_data]

fig, plt6 = plt.subplots()
plt6.plot(t_distances_cm, eqk07, "g", label='Equation K=0.7')
plt6.plot(t_distances_cm, eqk12, "b", label='Equation K=1.2')
plt6.plot(t_distances_cm, eqk14, "brown", label='Equation K=1.4')
plt6.plot(t_distances_cm, all_vals, "r", label='3 Equations')
# plt6.plot(t_distances_cm, m_distancesWebots, "b", label='Webots Equation')
plt6.plot(t_distances_cm, t_distances_cm, "k:", label='X=Y')
plt6.set(xlabel='Real Distances of sensor (cm)', ylabel='Estimated distances (cm)', title='System of 3 Equations vs Each one')
plt6.legend()
plt.tight_layout()
plt.show()