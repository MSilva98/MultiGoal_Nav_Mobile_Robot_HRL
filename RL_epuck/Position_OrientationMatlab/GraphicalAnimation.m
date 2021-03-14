function GraphicalAnimation(A,B,POS_d,ORI_d,IPoint)
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here
%--------------------------------------------------------------------------
% Plot the maze layout
H1 = figure(1); clf, hold on, set(H1,'Color',[1 1 1])
for kk = 1:size(A,1),
    if kk <= 4,
       H2 = plot([A(kk,1) B(kk,1)],[A(kk,2) B(kk,2)],'k');
       set(H2,'LineWidth',4)   
    else
        H2 = plot([A(kk,1) B(kk,1)],[A(kk,2) B(kk,2)],'k');
        set(H2,'LineWidth',2)
    end
end
axis off
axis equal
%--------------------------------------------------------------------------
% Mobile robot Alphabot2: two-wheeled circular shape
% Zero-position
R_body = 0.055;
BODY_x = R_body*cos(0:pi/180:2*pi);
BODY_y = R_body*sin(0:pi/180:2*pi);
% Wheels
R_wheel = 0.021;
W_wheel = 0.019;
LWheel_x = [-R_wheel R_wheel R_wheel -R_wheel -R_wheel];
LWheel_y = [R_body R_body R_body+W_wheel R_body+W_wheel R_body];
RWheel_x = [-R_wheel -R_wheel R_wheel R_wheel -R_wheel];
RWheel_y = [-R_body -R_body-W_wheel -R_body-W_wheel -R_body -R_body];
% Transformations according the current robot's localization
LPos = [ cos(ORI_d) -sin(ORI_d) POS_d(1)
         sin(ORI_d) cos(ORI_d)  POS_d(2)
         0         0          1]*[LWheel_x;LWheel_y;ones(1,5) ];
RPos = [ cos(ORI_d) -sin(ORI_d) POS_d(1)
         sin(ORI_d) cos(ORI_d)  POS_d(2)
         0          0           1]*[RWheel_x;RWheel_y;ones(1,5) ];    
%--------------------------------------------------------------------------
% Plot the mobile robot Alphabot2
H3 = plot(BODY_x+POS_d(1),BODY_y+POS_d(2),'b',...   % Body part
          LPos(1,:),LPos(2,:),'b',...               % Left wheel
          RPos(1,:),RPos(2,:),'b');                 % Right wheel
set(H3,'LineWidth',2)
%--------------------------------------------------------------------------
% Plot the IR distance measure
for kk = 1:size(IPoint,1),    
    plot([POS_d(1) IPoint(kk,1)],[POS_d(2) IPoint(kk,2)],'r:')
end
%--------------------------------------------------------------------------
end

