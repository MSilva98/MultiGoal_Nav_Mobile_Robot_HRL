% Sharp IR distance sensor linearization
% https://www.limulo.net/website/coding/physical-computing/sharp-linearization.html
clear all
close all
%% SharpGP2Y0A41SK0F output distance characteristic
d_cm = [2 3 3.5 4 5 6  7 8 10 12 14 16 20 25 30 40];
AOVoltage = [2.23 3.05 3.0 2.73 2.35 2.03 1.76 1.57 1.28 1.03 0.92 0.81 0.65 0.53 0.43 0.28];
figure
plot(d_cm,AOVoltage,'r*-')
xlabel('Distance to reflective object (cm)'), ylabel('Analog voltage output (V)')
axis([0 40 0 3.2]), grid on

%% Inverse distance with and without corrective constant k = 0.42
inv_d_cm = 1./d_cm;
inv_d_cm_k = 1./(d_cm+0.42);
figure
plot(inv_d_cm,AOVoltage,'r*-', inv_d_cm_k,AOVoltage,'b*-')
xlabel('Inverse number of distance(1/cm)'), ylabel('Analog voltage output (V)')
axis([0 0.45 0 3.2]), grid on
legend('without corrective constant','with corrective constant')

%% Linearization curve (with corrective constant k = 0.42)
inv_d_cm = 1./(d_cm(3:end)+0.42);
AOVoltage = AOVoltage(3:end); 
coef = polyfit(inv_d_cm,AOVoltage,1);

x = linspace(0.024,0.255,100);
y = polyval(coef,x);
figure
plot(x,y,'k:',inv_d_cm,AOVoltage,'rs')
xlabel('Inverse number of distance(1/cm)'), ylabel('Analog voltage output (V)')

%% Compute distance from analog voltage
V_meas = 2.4;
d_cm_est = coef(1)/(V_meas-coef(2))-0.42

