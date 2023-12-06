%% Dist calculation
LATRAD2M =     ( 6358805.404762348 )                      ;   %Y in K-city
LONRAD2M =     ( 5083989.602644125 )                      ;   %X in K-city
UNIT     = struct('D2R',pi/180, ...
                  'R2D',180/pi)                           ;
H    = zeros(3,1);
H(1) = sqrt((WPparapark(1,1)-parapark(1,1))*UNIT.D2R*LATRAD2M)^2+(((WPparapark(1,2)-parapark(1,2))*UNIT.D2R*LONRAD2M)^2); 
H(2) = sqrt((WPparapark(3,1)-parapark(2,1))*UNIT.D2R*LATRAD2M)^2+(((WPparapark(3,2)-parapark(2,2))*UNIT.D2R*LONRAD2M)^2);
H(3) = sqrt((WPparapark(5,1)-parapark(3,1))*UNIT.D2R*LATRAD2M)^2+(((WPparapark(5,2)-parapark(3,2))*UNIT.D2R*LONRAD2M)^2);
S    = zeros(3,1);
S(1) = WPparapark(1,5);
S(2) = WPparapark(3,5);
S(3) = WPparapark(5,5);

%% Simulation in Matlab
UNIT     = struct('D2R',pi/180, ...
                  'R2D',180/pi)                           ;
%Arbitrary constant
S = 5               ;
H = 3               ;
%Radius of Curvature Minimum
Rmin = 1.05/tan(24*UNIT.D2R);
k = (S*(H-2*Rmin)+sqrt(4*Rmin^2*(S^2+H^2)-16*Rmin^3*H))/(S^2- 4*Rmin^2);
m = Rmin*(1-sqrt(1+k^2));
% 곡률 반경 중심 좌표
O_1 = [S,H-Rmin]; 
O_2 = [0,Rmin];

alpha= acos((S^2-H^2)/(S^2+H^2));
theta = linspace(pi/2,pi/2+alpha,100);
xO1 = O_1(1) + Rmin * cos(theta);
yO1 = O_1(2) + Rmin * sin(theta);
theta = linspace(-pi/2,-pi/2+alpha,100);
xO2 = O_2(1) + Rmin * cos(theta);
yO2 = O_2(2) + Rmin * sin(theta);
% Origin
syms x y
O1= [S H-Rmin];
eqn=(x-O1(1))^2+(y-O1(2))^2-Rmin^2==k*x-y+m;
% 곡선과 직선이 만나는 점 

%% plot
figure; hold on; grid on;
% Radius of Curvature points
plot(O_1(1),O_1(2),'rp');
plot(O_2(1),O_2(2),'rp');
% Radius 
plot(xO1,yO1,'r-',xO2,yO2,'r-');
plot(0,0,'bp');
plot(S,H,'bp');
plot(x,y,'b-');
xlim([0 10]);
ylim([0 10]);

%% Check
% 4.6
% 2.8
Parking1 =[-2.3  -1.4;
           -2.3   1.4;
            2.3   1.4;
            2.3  -1.4 ;
            -2.3 -1.4];
Parking1(:,1)= Parking1(:,1)+2;
Parking2 = Parking1;
Parking2(:,1) = Parking2(:,1)+4.6;

UNIT     = struct('D2R',pi/180, ...
                  'R2D',180/pi)                           ;
S = 7;
H = 2.8+0.7;
syms Rmin
eqn= sqrt((S/2-S)^2+(H/2-(H-Rmin))^2)==Rmin;
Rmin = double(solve(eqn,Rmin));
k = (S*(H-2*Rmin)+sqrt(4*Rmin^2*(S^2+H^2)-16*Rmin^3*H))/(S^2- 4*Rmin^2);
m = Rmin*(1-sqrt(1+k^2));

% 곡률 반경 중심 좌표
O_1 = [S,H-Rmin]; 
O_2 = [0,Rmin];

alpha= acos((S^2-H^2)/(S^2+H^2));
theta = linspace(pi/2,pi/2+alpha,100);
xO1 = O_1(1) + Rmin * cos(theta);
yO1 = O_1(2) + Rmin * sin(theta);
theta = linspace(-pi/2,-pi/2+alpha,100);
xO2 = O_2(1) + Rmin * cos(theta);
yO2 = O_2(2) + Rmin * sin(theta);

% 곡선과 직선이 만나는 점 
f1= linecirc(k,m,S,H-Rmin,Rmin);
f2= linecirc(k,m,0,Rmin,Rmin);
x= f2(1):0.01:f1(1);
y = k*x+m;
%% plot
figure; hold on; grid on;
% Radius of Curvature points
plot(Parking1(:,1),Parking1(:,2),'r*-');
plot(Parking2(:,1),Parking2(:,2),'r*-');
plot(O_1(1),O_1(2),'rp');
plot(O_2(1),O_2(2),'rp');
% Radius 
plot(xO1,yO1,'r-',xO2,yO2,'r-');
plot(0,0,'bp');
plot(S,H,'bp');
plot(x,y,'b-');
xlim([0 10]);
ylim([0 10]);

%% Parallel Parking matlab
%WP=readmatrix("C:/Users/82102/Desktop/하다/하다2기코드/코드 인수인계/HADA2/fileIn/waypoint/WP_TRACK.csv");
% parapark=readmatrix("C:/Users/82102/Desktop/하다/하다2기코드/코드 인수인계/HADA2/fileIn/waypoint/PARK_PARALLEL.csv");
close all;clc; clear all;
parapark = readmatrix("WP_GEN3\data\WP_Parapark_JK,HMW.csv");
WPparapark= readmatrix("WP_GEN3\data\WP_Parapark.csv");
figure;
geobasemap('satellite'); hold on;
% geoplot_with_index(parapark(:,1:2),'rp','parallel Parking');
% geoplot_with_index(WPparapark(:,1:2),'b*-','Waypoint for parapark');
UNIT     = struct('D2R',pi/180, ...
                  'R2D',180/pi)                           ;
LATRAD2M =     ( 6358805.404762348 )                      ;   %Y in K-city
LONRAD2M =     ( 5083989.602644125 )                      ;   %X in K-city
%Where Vehicle stops
Origin = WPparapark(2,1:2);
%Where Vehicle parks
Destination =parapark(1,1:2);
%Distance between Origin and Destination
Dist= sqrt(((Origin(1)-Destination(1))*UNIT.D2R*LATRAD2M)^2+((Origin(2)-Destination(2))*UNIT.D2R*LONRAD2M)^2);
Midpoint= (Origin+Destination)/2;

%Check=[Origin(1)-H*UNIT.R2D/LATRAD2M Origin(2)-S*UNIT.R2D/LONRAD2M];
syms R
rot= parapark(3,3)*UNIT.D2R;
S= Dist*cos(rot);
H= Dist*sin(rot);

%%%%If Tangent Line Exists, else comment this out
k = (S*(H-2*R)+sqrt(4*R^2*(S^2+H^2)-16*R^3*H))/(S^2- 4*R^2);
m = R*(1-sqrt(1+k^2));
%%%%
% From Origin
O_1 = [Origin(1),Origin(2)-R/LONRAD2M*UNIT.R2D];
% From Parking
O_2 = [Destination(1),Destination(2)+R/LONRAD2M*UNIT.R2D];
alpha= acos((S^2-H^2)/(S^2+H^2));
theta = linspace(parapark(3,3)+pi/2,parapark(3,3)+pi/2+alpha,100);
xO1 = O_1(2) + (R * cos(theta))*UNIT.R2D/LONRAD2M;
yO1 = O_1(1) + (R * sin(theta))*UNIT.R2D/LATRAD2M;
theta = linspace(parapark(3,3)-pi/2,parapark(3,3)-pi/2+alpha,100);
xO2 = O_2(2) + (R * cos(theta))*UNIT.R2D/LONRAD2M;
yO2 = O_2(1) + (R * sin(theta))*UNIT.R2D/LATRAD2M;
% plot
geoplot_with_index(Origin,'rp','Stop'); hold on;
geoplot_with_index(Destination,'bp','Parking');
geoplot_with_index(Midpoint,'gp','Middle Point');
geoplot(yO1,xO1,'r-');
geoplot(yO2,xO2,'r-');
