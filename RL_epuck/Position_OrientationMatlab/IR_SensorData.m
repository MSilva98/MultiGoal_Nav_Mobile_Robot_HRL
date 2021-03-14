function [D_IR_sensor,IP_IR_sensor] = IR_SensorData(A,B,x,y,fi,IR_sensor_ori)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
%--------------------------------------------------------------------------
% Simulate the specified number of IR-sensors
NUM_IR_sensors = length(IR_sensor_ori);
D_IR_sensor = zeros(NUM_IR_sensors,1);  % Pre-allocation for speed
IP_IR_sensor = zeros(NUM_IR_sensors,2); % Pre-allocation for speed
for ii = 1:NUM_IR_sensors,

    % Angles range: from -pi to pi
    AUX_ori = fi+IR_sensor_ori(ii);
    if AUX_ori > pi, AUX_ori = AUX_ori-2*pi; end
    if AUX_ori < -pi, AUX_ori = AUX_ori+2*pi; end
    % Specifies points A and B corresponding to lines AB
    PA = [A; x y]; 
    if (AUX_ori >= -pi/4 && AUX_ori <= pi/4) || ...,
       (AUX_ori >= 3*pi/4 && AUX_ori <= -3*pi/4)     
       PB = [B; 10 tan(fi+IR_sensor_ori(ii))*(10-x)+y];
    else
        PB = [B; (10-y)/tan(fi+IR_sensor_ori(ii))+x 10];
    end
    DIM = size(PA,1);
    % Find the intersection points among ALL-lines
    [I,flag,IPoint] = LineLineIntersection(PA,PB);
    OUT_data = [I IPoint];
    % Extract the intersection points: robot vs maze
    ILog1 = I(:,2) == DIM;
    IP_rob_maz = IPoint(ILog1,:);
    D_rob_maz = sqrt((x-IP_rob_maz(:,1)).^2+(y-IP_rob_maz(:,2)).^2);
    %AUX_IP = [I(ILog) IP_rob_maz]
    % Eliminate intersection points outside line segments 
    IDel = [];
    for kk = 1:DIM-1,
        if abs(PA(kk,1)-PB(kk,1)) < 1e-10,    % X-alignment
            if ~( IP_rob_maz(kk,2) >= PA(kk,2) & IP_rob_maz(kk,2) <= PB(kk,2) ),
                IDel = [IDel; kk];  
            end
        elseif abs(PA(kk,2)-PB(kk,2)) < 1e-10 % Y-alignment
                if ~( IP_rob_maz(kk,1) >= PA(kk,1) & IP_rob_maz(kk,1) <= PB(kk,1)),
                    IDel = [IDel; kk]; 
                end
        else
            if ~( IP_rob_maz(kk,1) >= PA(kk,1) & IP_rob_maz(kk,1) <= PB(kk,1) & ...
                  IP_rob_maz(kk,2) >= PA(kk,2) & IP_rob_maz(kk,2) <= PB(kk,2) ),
                IDel = [IDel; kk]; 
            end
        end
    end
    IP_rob_maz(IDel,:) = [];
    D_rob_maz(IDel) = [];
    % Extract intersection points given the current robot's orientation    
    if AUX_ori >= -pi/4 && AUX_ori < pi/4,
       ILog2 = x <= IP_rob_maz(:,1);
    end
    if AUX_ori >= pi/4 && AUX_ori < 3*pi/4,
       ILog2 = y <= IP_rob_maz(:,2);
    end
    if abs(AUX_ori) >= 3*pi/4 && abs(AUX_ori) <= pi,
       ILog2 = x >= IP_rob_maz(:,1);
    end
    if AUX_ori >= -3*pi/4 && AUX_ori <= -pi/4,
       ILog2 = y >= IP_rob_maz(:,2);
    end  
    % Sensor distance
    D_rob_maz = D_rob_maz(ILog2);
    [D_IR_sensor(ii) I] = min(D_rob_maz);
    % Coordinates of the intersection point with the environment
    IP_rob_maz = IP_rob_maz(ILog2,:);
    IP_IR_sensor(ii,:) = IP_rob_maz(I,:);
    % Clear variables
    clear ILog1 ILog2
    
end
%--------------------------------------------------------------------------
end

