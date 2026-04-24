clear;close all;clc;

CM_elevator=2.4239233/2;%(rad^-1)
CM_rudder=0.0215093;%(rad^-1)
CM_aileron=0.2710920/2;%(rad^-1)
CM_flap=0.0079219/2;%(rad^-1)

% HTMeanCord=(1.66+0.833)/2;%[ft]
% VTMeanCord=1.3;%[ft]
wingMeanCord=(2.1+1.05)/2;%[ft]

% HTArea=10;%[ft^2]
% VTArea=VTMeanCord*2;%[ft^2]
wingArea=wingMeanCord*25;%[ft^2]

v=220.537;%[ft/s] 120knots
rho=14.28e-4;%air density [slugs/ft^3] @15000ft
q=rho*v^2/2;% dynamic pressure [slugs/s^2/ft]=[psf]

deflection_flaps=deg2rad(45);%[rad]
deflection_ailerons=deg2rad(15);%[rad]
deflection_rudder=deg2rad(20);%[rad]
deflection_elevators=deg2rad(5);%[rad]


flapHingeMoment=CM_flap*deflection_flaps*wingArea*wingMeanCord*q/2;%[lb*ft]
aileronHingeMoment=CM_aileron*deflection_ailerons*wingArea*wingMeanCord*q/2;%[lb*ft]
rudderHingeMoment=CM_rudder*deflection_rudder*wingArea*wingMeanCord*q;%[lb*ft]
elevatorHingeMoment=CM_elevator*deflection_elevators*wingArea*wingMeanCord*q/2;%[lb*ft]

flapHingeMoment=flapHingeMoment*1.3558%[Nm]
aileronHingeMoment=aileronHingeMoment*1.3558%[Nm]
rudderHingeMoment=rudderHingeMoment*1.3558%[Nm]
elevatorHingeMoment=elevatorHingeMoment*1.3558%[Nm]