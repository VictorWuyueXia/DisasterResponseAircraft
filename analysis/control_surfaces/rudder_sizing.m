sympref('FloatingPointOutput',true);
clear all;close all;clc;

% Elevator Design

%%
% 0.constants
f=1807;%[N] engine thrust
m=326.587;%[kg] max take-off mass
dcg=0.12;%[m] distance between the most aft and most forward cg
S=3.58;%[m^2] wing area
b=7.62;%[m] wingspan
Sv=0.24;%[m^2] vertical stablizer area
Vs=26.91;%[m/s] stall speed, 120knots
ClaV=5.73;%[rad^-1] Cl vs AoA slope for vertical stablizer
etaV=0.96;% vertical tail dynamic pressure ratio, qv/q_inf
Lf=3.65;%[m] fuselage length
Df=0.457;%[m] fuselage diameter
dsigma_dbeta=0;% vertical tail sidewash gradient
%                beta=atan2(vwind/u) wind velocity over ground velocity
%                sigma=crab angle, between sind velocity and aircraft nose direction
Cno=0;%part of yawing moment coeff, aircraft is symmetric about xz-plane
Cyo=0;% part of aerodynamic side force coeff


%%
% 1.required to land safely with a crosswind of 20*n knots
Vw=10.289*1.25;%[m/s] 20*1.25knots

cg2acV=1.411;%[m] most forward cg to verTail ac distance
bV=0.61;%[m] tail span

%%
% 2.The most unfavorable cg location and aircraft weight combination
% And the most unfavorable altitude for directional control
lv=cg2acV-dcg%[m]

% 3.Design based on max TO weight at sea-level

% 4.Suppose the approach speed is 1.1*stall speed
u=1.1*Vs%[m/s]

% 5.Determine the total airspeed
Vt=sqrt(u^2+Vw^2)%[m/s]

% 6.Projected side area 
% 2% is added as the estimated landing gear
Ss=1.02*(Lf*Df+Sv)%[m^2]

%%
% 7.Center of the projected area and its distance to cg
% Assumed rectangular fuselage and vTail side projection,
% so the center is at the midpoint

% mean vtail cord
cbarV=Sv/bV%[m]

% distance between the center of area and fuselage nose is
xca=( ...
    (Lf*Df)*Lf/2 ...
    +Sv*( ...
    (Lf-cbarV)+cbarV/2 ...
    ) ...
    ) ...
    /(Lf*Df+Sv)%[m]

% Assume the ac center of the tail is at the quarter chord
% so the distance between the cg and fuselage nose is 
xcg=Lf-lv-.75*cbarV%[m]

% distance beween the center of the projected area cg
dc=xca-xcg%[m] positive for ca behind cg

%%
% 8.Side force produced by the crosswind
% assume wind from right, so positive side slip angle

% For cylindrical shape, assume
% side drag coeff (go check Sadreay section 16.2.2.3)
Cdy=0.6;

% sea-level air 
rho=1.225;%[kg/m^3]

% Side force
Fw=rho/2*Vw^2*Ss*Cdy%[N]

%%
% 9.rudder span is selected to be 0.9
br_bv=0.9;

% 10.rudder cord is selected to be 0.4cbarV
Cr_Cv=0.4;

% 11.Sideslip angle
beta=atan2(Vw,u)%[rad]

% 12. Sideslip derivatives

%Choose Kf1 from Section 6.8.1
Kf1=0.75;
% Choose Kf2 from Table 12.6.2.3
Kf2=1.35;

Cn_beta=Kf1*ClaV*(1-dsigma_dbeta)*etaV*lv*Sv/b/S%[rad]
Cy_beta=-Kf2*ClaV*(1-dsigma_dbeta)*etaV*Sv/S%[rad]

%%
% 13.Rudder AoA effectiveness
%% 
% 

tau_r=0.6;

%%
% 14.Aircraft control derivatives
Cy_dr=ClaV*etaV*tau_r*br_bv*Sv/S%[rad^-1]
Cn_dr=-ClaV*lv*Sv/b/S*etaV*tau_r*br_bv%[rad^-1]

%%
% 15.Compute rudder deflection
syms dr sigma

% BookEqn1=0==1/2*1.225*70.95^2*365*60 ...
%     *(0.2*(0.294-sigma)-0.136*dr)+62936*3.766*cos(sigma)
eqn1=0==rho/2*Vt^2*S*b ...
    *(Cno+Cn_beta*(beta-sigma)+Cn_dr*dr) ...
    +Fw*dc*cos(sigma)

% BookEqn2=62936==1/2*1.225*70.95^2*365 ...
%     *(-.799*(.294-sigma)+.302*dr)
eqn2=rho/2*Vw^2*Ss*Cdy ...
    ==rho/2*Vt^2*S ...
    *(Cyo+Cy_beta*(beta-sigma)+Cy_dr*dr)

[dr,sigma]=vpasolve([eqn1,eqn2],[dr sigma],[0.5;0.2])

dr_deg=rad2deg(dr)