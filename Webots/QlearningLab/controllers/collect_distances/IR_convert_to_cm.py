import numpy
import matplotlib.pyplot as plt

# https://www.limulo.net/website/coding/physical-computing/sharp-linearization.html

# Compute this values - ver resolução dos sensores
distances_cm = [4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30]
sensorVals = [
2.439349559160350,
2.083649437781640,
1.814999273823080,
1.598285660931710,
1.433913928959740,
1.286998150676660,
1.167077954375970,
1.064086898639210,
0.991861623091553,
0.933862077186127,
0.881399820086957,
0.833240398092112,
0.784840139920606,
0.740669339648862,
0.702886217109464,
0.667850969034901,
0.640069782775082,
0.613253385661716,
0.587550874084748,
0.562364644576316,
0.536143384811568,
0.512957670178804,
0.491273105282327,
0.469167609973223,
0.447494245576402,
0.424859519573155,
0.412892232238276
]
k = 0.99

fig1, plt1 = plt.subplots()
plt1.plot(distances_cm, sensorVals, 'r*-')
plt1.set(xlabel='Distance (cm)', ylabel='Analog Output Voltage (V)')

inv_d = [1/x for x in distances_cm]
inv_d_k = [1/(x+k) for x in distances_cm]   
fig2, plt2 = plt.subplots()
plt2.plot(inv_d, sensorVals, 'r*-')
plt2.plot(inv_d_k, sensorVals, 'b*-')
plt2.set(xlabel='Inverse Distance (1/cm)', ylabel='Analog Output Voltage (V)')
plt2.legend(['Without Corrective Constant', 'With Corrective Constant'], loc='lower right')

coef = numpy.polyfit(inv_d_k, sensorVals, 1)
x = numpy.linspace(0.02, 0.255, 100)
y = numpy.polyval(coef, x)
fig3,plt3 = plt.subplots()
plt3.plot(x, y, 'k:')
plt3.plot(inv_d_k, sensorVals, 'rs')
plt3.set(xlabel='Inverse Distance (1/cm)', ylabel='Analog Output Voltage (V)')
plt.tight_layout()
plt.show()

def getDistance(voltage):
    return coef[0]/(voltage-coef[1])-k


# https://cyberbotics.com/doc/guide/distancesensor-sensors#sharp-gp2y0a41sk0f
# Model of sensor - Sharp GP2Y0A41SK0F
# Range from 4 to 30 cm
# Equations
# Convert meters to voltage: y(x) = 0.5131*x^(-0.5735)-0.6143
# Convert voltage to meters: y(x) = 0.1594*x^(-0.8533)-0.02916

def getDistance2(voltage):
    return ((0.1594*pow(voltage,-0.8533))-0.02916)*100 # *100 -> m to cm

print(getDistance(2.4))  # 4.20
print(getDistance2(2.4)) # 4.64