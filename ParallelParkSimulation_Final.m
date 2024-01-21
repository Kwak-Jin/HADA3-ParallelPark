close all;clc; clear all;
parapark = readmatrix("WP_GEN3\data\Parapark_Info.csv");
WPparapark= readmatrix("WP_GEN3\data\WP_Parapark.csv");
UNIT     = struct('D2R',pi/180, ...
                  'R2D',180/pi)                           ;
LATRAD2M =     ( 6358805.404762348 )                      ;   %Y in K-city
LONRAD2M =     ( 5083989.602644125 )                      ;   %X in K-city
% Where Vehicle stops
ERPPOS = [37.23931166666667, 126.77315166666668 ]';
% Where Vehicle parks
Park =parapark(3,1:2)';
% Distance between Origin and Destination
%Dist= sqrt(((ERPPOS(1)-Park(1))*UNIT.D2R*LATRAD2M)^2+((ERPPOS(2)-Park(2))*UNIT.D2R*LONRAD2M)^2);
Midpoint= (ERPPOS+Park)/2;
% Coordinate rotation angle

ParaAzimuth       =  parapark(3,3)*UNIT.D2R;
AngleP2E    =  azimuth('rh',Park(1),Park(2),ERPPOS(1),ERPPOS(2))*UNIT.D2R
% Distance Vertical/Horizontal with respect to I-frame
dLAT = ERPPOS(1)-Park(1);
dLON = ERPPOS(2)-Park(2);

% Distance between ERP and Parking
L    = sqrt((dLAT*LATRAD2M*UNIT.D2R)^2+(dLON*LONRAD2M*UNIT.D2R)^2)       ;

% Constants
S    = L*cos(ParaAzimuth-AngleP2E);
H    = L*sin(ParaAzimuth-AngleP2E);

%Rmin = 1.05/tan(25*UNIT.D2R);
syms Rmin
eqn= sqrt((S/2-S)^2+(H/2-(H-Rmin))^2)==Rmin;
Rmin = double(solve(eqn,Rmin));

% Origin of Radius of Curvature
ParaNormal = ParaAzimuth+ pi/2;
O1  = [ERPPOS(1)+(Rmin*cos(ParaNormal))*UNIT.R2D/LATRAD2M;ERPPOS(2)+(Rmin*sin(ParaNormal))*UNIT.R2D/LONRAD2M]
O2  = [Park(1)-(Rmin*cos(ParaNormal))*UNIT.R2D/LATRAD2M;Park(2)-(Rmin*sin(ParaNormal))*UNIT.R2D/LONRAD2M]
O12Mid      =  azimuth('rh',O1(1),O1(2),Midpoint(1),Midpoint(2))*UNIT.D2R
%O12ERP      =  azimuth('rh',O1(1),O1(2),ERPPOS(1),ERPPOS(2))*UNIT.D2R;

% Angle of Each Radial trajectory
alpha = acos((S^2-H^2)/(S^2+H^2))
theta1 = linspace(pi/2-O12Mid-alpha,pi/2-O12Mid,10);
xO1 = O1(2) + (Rmin * cos(theta1))*UNIT.R2D/LONRAD2M;
yO1 = O1(1) + (Rmin * sin(theta1))*UNIT.R2D/LATRAD2M;
theta2 = linspace(pi/2-ParaNormal+alpha,pi/2-ParaNormal,10);
xO2 = O2(2) + Rmin * cos(theta2)*UNIT.R2D/LONRAD2M;
yO2 = O2(1) + Rmin * sin(theta2)*UNIT.R2D/LATRAD2M;
RADCURV   = [yO1' xO1';
             yO2' xO2'];
%% Check plot

geobasemap('satellite'); hold on;
geoplot_with_index(O1,'bp','Park');
geoplot_with_index(O2,'bp','Park');
geoplot_with_index(Midpoint,'gp','Midpoint');
geoplot_with_index(Park,'cp','Park');
geoplot_with_index(ERPPOS,'kp','Park');
geoplot_with_index(RADCURV,'r*','Radius of Curvature');
legend('Origin 1','Origin 2','MidPoint','Parking Destination','ERP POS','Curvature1');
%%
clear all; clc; close all;
geobasemap('satellite'); hold on;
ParaTrajectory =readmatrix("ParaparkTrajectory.csv");
for i= 1:length(ParaTrajectory(:,1))
    geoplot(ParaTrajectory(i,1),ParaTrajectory(i,2),'rp');
    drawnow;
end
%%
close all;
%O1= [ 37.23929542 126.77319122];
%O2= [ 37.23931425 126.77310061];
X=[126.77321452085893, 126.77321734053764, 126.77321958443986, 126.77322118763674, 126.77322210373873, 126.77322230623777, 126.77322178927444, 126.77322056780743, 126.77321867718068, 126.77321617210076, 126.77321503724967, 126.77321212188883, 126.77320976874758, 126.77320804591565, 126.77320700324434, 126.77320667090406, 126.77320705851128, 126.77320815485032, 126.77320992819791, 126.773212327241]
Y=[37.23940982834975, 37.239412352709024, 37.23941522274768, 37.239418355419204, 37.239421660077646, 37.23942504110049, 37.239428400655555, 37.23943164153186, 37.239434669952466, 37.239437398287976, 37.239439243748, 37.239441697605805, 37.2394445111325, 37.23944760291679, 37.23945088349581, 37.23945425794382, 37.239457628618865, 37.23946089798822, 37.23946397145046, 37.2394667600729]
geobasemap('satellite');
hold on; 
geoplot(Y,X,'gp');
