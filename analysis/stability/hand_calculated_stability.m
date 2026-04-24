%% Hand Calculated stability

clear;close;clc;

load('stabDerivs.mat')

%%
m=720;%[lbm]
volume=24.7;%[ft^3]
c=1.575;%[ft]
b=25;%[ft]
S=39.37;%[ft^2]
St=10;%[ft^2]
rho=14.28e-4;%[slugs/ft^3]
v=220.54;%[ft/s]
q=rho*v^2/2%[slugs/s^2/ft]=[psf]

Iyy=227.640*m/volume/32%[slugs*ft^2]
Izz=429.121*m/volume/32;%[slugs/ft^2]

lt=5.42
dcLt_dat = 5.729; % cl_alphat
dep_da = 0.25; % epsilon_a, slope of downwash angle vs AoA
tail_AC = 10.137;
wing_AC = 7.119;
lt_prime = tail_AC - wing_AC;
cLadot= 2*dcLt_dat*dep_da*lt_prime/c*St/S %1.5-3.0

%% Short Peroid Mode

CMm_base=0;%0.0848501;
CMm_alpha=-3.1350291;
CMm_q=-25.8161070;
CMm_elev=2.4239233;
% CMm_alphaDot=0;%-1.691;
cMadot = -2*dcLt_dat*dep_da*(lt/c)*(lt_prime/c)*(St/S)

Mm_alpha=CMm_alpha*q*c*S
Mm_q=CMm_q*q*c*S*(c/2/v);
Mm_elev=CMm_elev*q*c*S;
Mm_base=CMm_base*q*c*S;
% Mm_alphaDot=CMm_alphaDot*q*c*S;
Mm_alphaDot=cMadot*q*S*c*(c/2./v)

omega=(-Mm_alpha/Iyy)^0.5%[rad/sec]
xi=(-(Mm_q+Mm_alphaDot)/Iyy)/2/omega
period=1/(omega/2/pi)%[sec]


alpha0=[1,0]';%[alpha; alpha_dot]
tspan=[0,10];
[t,alpha]=ode45(@(t,x)SPMode(t,x,...
    Mm_alpha,Mm_q,Mm_elev,Mm_base,...
    Iyy,Mm_alphaDot),tspan,alpha0);

figure
plot(t,alpha(:,1),'LineWidth',2)
xlabel('time (s)');
ylabel('Impulse Response');
title('Short Period Mode');

%% Dutch Roll Mode

CMn_beta=table2array(stabDerivs('CMn','beta'));
CMn_r=table2array(stabDerivs('CMn','r'));
CMn_rudder=table2array(stabDerivs('CMn','rudder'));

Mn_beta=CMn_beta*q*b*S;
Mn_r=CMn_r*q*b*S*(b/2/v);
Mn_rudder=CMn_rudder*q*b*S;

omega=(Mn_beta/Izz)^0.5%[rad/sec]
xi=(-Mn_r/Izz)/2/omega
period=1/(omega/2/pi)%[sec]


beta0=[1,0]';%[alpha; alpha_dot]
tspan=[0,30];
[t,beta]=ode45(@(t,x)DRMode(t,x,Mn_beta,Mn_r,Mn_rudder,Izz),tspan,beta0);

figure
plot(t,beta(:,1),'LineWidth',2)
xlabel('time (s)');
ylabel('Impulse Response');
title('Ditch Roll Mode');

%%
function dx=DRMode(t,x,Mn_beta,Mn_r,Mn_rudder,Izz)
    beta=x(1);
    beta_dot=x(2);
    
    rudder=0;%+1*(t==0);
    
    beta_ddot=(Mn_r*beta_dot-Mn_beta*beta-Mn_rudder*rudder)/Izz;
    
    dx=[beta_dot;beta_ddot];
end
%% 
% 

function dx=SPMode(t,x,Mm_alpha,Mm_q,Mm_elev,Mm_base,Iyy,Mm_alphaDot)
    alpha=x(1);
    alpha_dot=x(2);
    
    de=0;%+1*(t==0);
    
    alpha_ddot=((Mm_q+Mm_alphaDot)*alpha_dot+Mm_alpha*alpha+Mm_elev*de+Mm_base)/Iyy;
    
    dx=[alpha_dot;alpha_ddot];
end
%% 
%