import numpy
import matplotlib.pyplot as plt

# https://www.limulo.net/website/coding/physical-computing/sharp-linearization.html

# Compute this values - ver resolução dos sensores
t_distances_cm = [4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30]
# Values obtained using webots epuck with this sensor (Sharp GP2Y0A41SK0F) builtin in front of the robot
# Sensor as an offset of 3 cm from center of robot which was considered
sensorVals = [
    1.61352880785326600,
    1.43377018100631240,
    1.28705183710230700,
    1.17208331490884030,
    1.06387317358384600,
    0.99351022657205420,
    0.93488823336375750,
    0.88302174461569990,
    0.83173435567693990,
    0.78589335140912010,
    0.74083657930346870,
    0.70311633190787070,
    0.66837510498309240,
    0.64077721358365160,
    0.61473111588227130,
    0.58808622227290810,
    0.56206625531085770,
    0.53714963364933900,
    0.51341705321055220,
    0.49112689838530000,
    0.46868489140718056,
    0.44700516500458876,
    0.42490559905063060,
    0.41258393139430300,
    0.40252204575921186,
    0.39216870053219870,
    0.38288633824110810
]

sensorVals_noOffset = [
    2.39418651313791870,
    2.08499891155868950,
    1.82151906561484630,
    1.60345355968724480,
    1.43395317052195170,
    1.28519967015518820,
    1.17079386352255500,
    1.06669906967101320,
    0.99406087250499160,
    0.93456774626409970,
    0.88199186393038040,
    0.83168659236860980,
    0.78571402742139020,
    0.74144755412342070,
    0.70395258965980030,
    0.66826162069001060,
    0.64005919637329020,
    0.61527584031175660,
    0.58839730719839720,
    0.56206446079499710,
    0.53558164060345250,
    0.51298794867821100,
    0.49079160346075700,
    0.46941847161771590,
    0.44733699993517920,
    0.42501566738362400,
    0.41293250763279193
]
k = 5
inv_d = [1/x for x in t_distances_cm]
inv_d_k = [1/(x+k) for x in t_distances_cm]
coef = numpy.polyfit(inv_d_k, sensorVals, 1)
x = numpy.linspace(0.02, 0.255, 100)
y = numpy.polyval(coef, x)

fig1, plt1 = plt.subplots()
plt1.plot(t_distances_cm, sensorVals, 'r*-')
plt1.set(xlabel='Distance (cm)', ylabel='Analog Output Voltage (V)')   
fig2, plt2 = plt.subplots()
plt2.plot(inv_d, sensorVals, 'r*-')
plt2.plot(inv_d_k, sensorVals, 'b*-')
plt2.set(xlabel='Inverse Distance (1/cm)', ylabel='Analog Output Voltage (V)')
plt2.legend(['Without Corrective Constant', 'With Corrective Constant'], loc='lower right')
fig3,plt3 = plt.subplots()
plt3.plot(x, y, 'k:')
plt3.plot(inv_d_k, sensorVals, 'rs')
plt3.set(xlabel='Inverse Distance (1/cm)', ylabel='Analog Output Voltage (V)')


k2 = 2.5
inv_d = [1/x for x in t_distances_cm]
inv_d_k = [1/(x+k2) for x in t_distances_cm]
coef_noOff = numpy.polyfit(inv_d_k, sensorVals_noOffset, 1)
y = numpy.polyval(coef_noOff, x)
fig5, plt5 = plt.subplots()
plt5.plot(t_distances_cm, sensorVals_noOffset, 'r*-')
plt5.set(xlabel='Distance (cm)', ylabel='Analog Output Voltage (V)', title='No Offset')
fig6, plt6 = plt.subplots()
plt6.plot(inv_d, sensorVals_noOffset, 'r*-')
plt6.plot(inv_d_k, sensorVals_noOffset, 'b*-')
plt6.set(xlabel='Inverse Distance (1/cm)', ylabel='Analog Output Voltage (V)', title='No Offset')
plt6.legend(['Without Corrective Constant', 'With Corrective Constant'], loc='lower right')
fig7,plt7 = plt.subplots()
plt7.plot(x, y, 'k:')
plt7.plot(inv_d_k, sensorVals_noOffset, 'rs')
plt7.set(xlabel='Inverse Distance (1/cm)', ylabel='Analog Output Voltage (V)', title='No Offset')


def getDistance(voltage):
    return coef[0]/(voltage-coef[1])-k

def getDistance3(voltage):
    return coef_noOff[0]/(voltage-coef_noOff[1])-k2

# https://cyberbotics.com/doc/guide/distancesensor-sensors#sharp-gp2y0a41sk0f
# Model of sensor - Sharp GP2Y0A41SK0F
# Range from 4 to 30 cm
# Equations
# Convert meters to voltage: y(x) = 0.5131*x^(-0.5735)-0.6143
# Convert voltage to meters: y(x) = 0.1594*x^(-0.8533)-0.02916
# IMPORTANT NOTE: it seems that the webots equations has the offset taken in consideration

def getDistance2(voltage):
    return ((0.1594*pow(voltage,-0.8533))-0.02916)*100 # *100 -> m to cm


m_distances_cm = [getDistance(x) for x in sensorVals]
m_distances_noOff_cm = [getDistance2(x) for x in sensorVals_noOffset]
m_distances2_cm = [getDistance2(x) for x in sensorVals]
m_distances3_cm = [getDistance3(x) for x in sensorVals_noOffset]

fig4, plt4 = plt.subplots()
plt4.plot(t_distances_cm, m_distances_cm, "rs")
plt4.plot(t_distances_cm, m_distances_noOff_cm, "cs")
# plt4.plot(t_distances_cm, m_distances2_cm, "bs")
plt4.plot(t_distances_cm, m_distances3_cm, "gs")
plt4.plot(t_distances_cm, t_distances_cm, "k:")
plt4.set(xlabel='Real Distances of sensor (cm)', ylabel='Estimated distances (cm)')
plt4.legend(["Obtained Equation", "Webots Equation No Off", "Equation No Offset", "X=Y"], loc="upper left")
plt.tight_layout()
plt.show()
