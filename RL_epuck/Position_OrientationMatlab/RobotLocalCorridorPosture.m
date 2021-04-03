function SOL_final = RobotLocalCorridorPosture(Dist_IR_sensor, IR_sensor_max_dist)
%--------------------------------------------------------------------------
% Determine the LOCAL robot's posture in a corridor
%--------------------------------------------------------------------------
% IR-sensor range
NUM_sensors = length(Dist_IR_sensor);
for k = 1:NUM_sensors
     if Dist_IR_sensor(k) > IR_sensor_max_dist
        Dist_IR_sensor(k) = IR_sensor_max_dist;         
     end
end
% Extract current distance measurements
d1 = Dist_IR_sensor(1);    d4 = Dist_IR_sensor(4); % Front/back sensors
d3 = Dist_IR_sensor(3);    d5 = Dist_IR_sensor(5); % Lateral sensors
% Calculate POSition and ORIentation in the corridor
if d1 ~= 0.3 & d4 ~= 0.3
     POS = 0.15*(d4-d1)/(d4+d1);    
     ORI = pi/2-acos((0.15-POS)./d1);
elseif d3 ~= 0.3 & d5 ~= 0.3
        POS = 0.15*(d3-d5)/(d3+d5);    
        ORI = acos((0.15-POS)/d5);
else
     disp('ERROR: distances (d1 or d4) and (d3 or d5) out of range should never happen!!!')
end
POS = abs(POS); ORI = abs(ORI); % Absolute values
% Define the 4 possible solutions!
POS_sol(1) =  POS;   ORI_sol(1) =  ORI;
POS_sol(2) =  POS;   ORI_sol(2) = -ORI;
POS_sol(3) = -POS;   ORI_sol(3) =  ORI;
POS_sol(4) = -POS;   ORI_sol(4) = -ORI;
% Find the solutions through coordinate matching
I_sol = 1:4;
P_R = [0 -Dist_IR_sensor(2)*sqrt(2)/2 -Dist_IR_sensor(3) 0 Dist_IR_sensor(5) Dist_IR_sensor(6)*sqrt(2)/2; 
            Dist_IR_sensor(1) Dist_IR_sensor(2)*sqrt(2)/2 0 -Dist_IR_sensor(4) 0  Dist_IR_sensor(6)*sqrt(2)/2; 
            1 1 1 1 1 1];
for k = 1:4,
        % Coordinate transformation
        T_R2L = [cos(ORI_sol(k)) -sin(ORI_sol(k)) POS_sol(k)
                        sin(ORI_sol(k))  cos(ORI_sol(k)) 0
                         0 0 1];
        % Reference frame new coordinates
        PL = [];
        for m = 1:NUM_sensors
             if Dist_IR_sensor(m) < 0.3;
                P_L = [PL T_R2L*P_R(:,m)];
             end
        end
        P_L_xx = abs(P_L(1,:));
        for m =1:length(P_L_xx)
             if abs(P_L_xx(m)-0.15) > 1e-8
                   I_sol(k) = [];
                   break
             end
        end
end
SOL_final = [POS_sol(I_sol); ORI_sol(I_sol)]'; 
end %----------------------------------------------------------------------

