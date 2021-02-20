import numpy
import matplotlib.pyplot as plt

# https://www.limulo.net/website/coding/physical-computing/sharp-linearization.html

# https://cyberbotics.com/doc/guide/distancesensor-sensors#sharp-gp2y0a41sk0f
# Model of sensor - Sharp GP2Y0A41SK0F
# Range from 4 to 30 cm
# Equations
# Convert meters to voltage: y(x) = 0.5131*x^(-0.5735)-0.6143
# Convert voltage to meters: y(x) = 0.1594*x^(-0.8533)-0.02916

# Compute this values - ver resolução dos sensores
distances_cm = [4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30]
sensorVals = [
    2.4543934803964813,
    2.0857225891635913,
    1.8050147563855587,
    1.5718546325370197,
    1.4253904095199885,
    1.285132256686784,
    1.1689770544533742,
    1.0602254495057448,
    0.9934011137483789,
    0.9376087713837975,
    0.8834950504985026,
    0.826813622161961,
    0.7871347864244898,
    0.741650699781455,
    0.7015223510690278,
    0.6691575341299844,
    0.638382342777867,
    0.6088014963499528,
    0.5866359623014504,
    0.5615903804413932,
    0.5370533382585617,
    0.511131616769578,
    0.48861761046287017,
    0.4651073610135785,
    0.4461150800975016,
    0.422786245667686,
    0.41206637170230653
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


def getDistance2(voltage):
    return ((0.1594*pow(voltage,-0.8533))-0.02916)*100 # *100 -> m to cm

print(getDistance(2.4))  # 4.5
print(getDistance2(2.4)) # 4.64