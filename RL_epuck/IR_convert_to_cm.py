import numpy
import matplotlib.pyplot as plt

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
x = numpy.linspace(0.02, 0.255, 100)

k3 = 1.34
inv_d = [1/x for x in t_distances_cm]
inv_d_k = [1/(x+k3) for x in t_distances_cm]
coef_3cm = numpy.polyfit(inv_d_k, voltageToSensor, 1)
y = numpy.polyval(coef_3cm, x)
# fig1, plt1 = plt.subplots()
# plt1.plot(t_distances_cm, voltageToSensor, 'r*-')
# plt1.set(xlabel='Distance (cm)', ylabel='Analog Output Voltage (V)')   
# fig2, plt2 = plt.subplots()
# plt2.plot(inv_d, voltageToSensor, 'r*-')
# plt2.plot(inv_d_k, voltageToSensor, 'b*-')
# plt2.set(xlabel='Inverse Distance (1/cm)', ylabel='Analog Output Voltage (V)')
# plt2.legend(['Without Corrective Constant', 'With Corrective Constant'], loc='lower right')
# fig3,plt3 = plt.subplots()
# plt3.plot(x, y, 'k:')
# plt3.plot(inv_d_k, voltageToSensor, 'rs')
# plt3.set(xlabel='Inverse Distance (1/cm)', ylabel='Analog Output Voltage (V)')

print("Equation: " + str(round(coef_3cm[0],4)) + "/(voltage- " + str(round(coef_3cm[1],4)) + ")-" + str(k3))
def getDistance(voltage):
    d = round((14.8903/(voltage+0.0418)-1.34),4)
    # Noise may give values higher than max distance possible (30 cm)
    # Added 3 cm to compensate for sensor placement relative to robot center
    return d+3 if d <= 30 else 33 

# https://cyberbotics.com/doc/guide/distancesensor-sensors#sharp-gp2y0a41sk0f
# Model of sensor - Sharp GP2Y0A41SK0F
# Range from 4 to 30 cm
# Equations
# Convert meters to voltage: y(x) = 0.5131*x^(-0.5735)-0.6143
# Convert voltage to meters: y(x) = 0.1594*x^(-0.8533)-0.02916
# IMPORTANT NOTE: it seems that the webots equations has the offset taken in consideration
def getDistance2(voltage):
    d = ((0.1594*pow(voltage,-0.8533))-0.02916)*100 # *100 -> m to cm
    # Added 3 cm to compensate for sensor placement relative to robot center
    return d+3 if d <= 30 else 33 

m_distances = [getDistance(x) for x in voltageToSensor]
m_distances2 = [getDistance2(x) for x in voltageToSensor]

fig4, plt4 = plt.subplots()
plt4.plot(t_distances_cm, m_distances, "gs")
# plt4.plot(t_distances_cm, m_distances2, "rs")
plt4.plot(t_distances_cm, t_distances_cm, "k:")
plt4.set(xlabel='Real Distances of sensor (cm)', ylabel='Estimated distances (cm)')
plt4.legend(["Obtained Equation", "Webots Equation", "X=Y"], loc="upper left")
plt.tight_layout()
plt.show()

for x in voltageToSensor:
    print(getDistance(x))
