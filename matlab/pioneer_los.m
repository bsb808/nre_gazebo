function [dist2end, linvel,angvel] = pioneer_los(x,y,th,xstart,ystart,xend,yend)

% pioneer_los controller
% Line of sight controller for pioneer
% Inputs:
%  x: current x position of robot
%  y: current y position of robot
%  th: current yaw angle of robot
%  xstart,ystart: starting point of the line
%  xend,yend: ending point (goal) of the line


% Control Parameters - tunable
look_ahead_distance = 1;
maxvel = 1;  % Maximum linear velocity 
Kang = 2;  % angular gain
Klin = 0.5;  % linear gain

% Slope of line
m = 0;
if ((xend-xstart)==0) % Vertical
    m = 10000;
else
    m = (yend-ystart)/(xend-xstart);
end

% Intersection between robot current location and line
xint = 0;
yint = 0;
if (m==0)
    yint = ystart;
    xint = x;
elseif (abs(m) > 1000)
    yint = y;
    xint = xstart;
else
    xint = (m/(m^2+1))*(y-ystart+m*xstart+x/m);
    yint = ystart+m*(xint-xstart);
end

% Distance to line
dist2line = sqrt((x-xint)^2+(y-yint)^2);
% Distances to end points
dist2end = sqrt((x-xend)^2+(y-yend)^2);
dist2start = sqrt((x-xstart)^2+(y-ystart)^2);

% Use Look Ahead Distance to pick a target point down the line
angleline = atan2(yend-ystart,xend-xstart);
xtgt = xint+look_ahead_distance*cos(angleline);
ytgt = yint+look_ahead_distance*sin(angleline);

% Check for target point beyond end point or before start point
linemag = sqrt((xend-xstart)^2+(yend-ystart)^2);
intmag_start = sqrt((xint-xstart)^2+(yint-ystart)^2);
intmag_end =  sqrt((xint-xend)^2+(yint-yend)^2);
%fprintf('linemag = %4.2f intmag_end=%4.2f, xint=%f, yint=%f\n',linemag,intmag_end,xint,yint);
if (intmag_start > (linemag))
    xtgt = xend;
    ytgt = yend;
end
if (intmag_end > (linemag))
    xtgt = xstart;
    ytgt = ystart;
end

% Angle to the target
angle_tgt = atan2(ytgt-y,xtgt-x);
hdgError = angle_tgt-th; 

% Clamp angle 
while (hdgError < -pi)
    hdgError = hdgError+2*pi;
end
while (hdgError > pi)
    hdgError = hdgError-2*pi;
end

% Apply gains and set outputs
angvel = Kang*hdgError;
linvel = min(maxvel, Klin*dist2end);

% Logic for sharp turns
if abs(hdgError) > 0.5
    linvel = 0;
end

% For debugging 
msg = sprintf('LOS: dist2goal: %5.1f, dist2line: %5.2f, angle_tgt %5.1f, th=%5.2f, hdgError=%5.2f', ...
     dist2end,dist2line,angle_tgt,th,hdgError);
%disp(msg);

 
