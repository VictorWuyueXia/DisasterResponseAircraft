function [Xdot] = RCAM_model(X,U)
%%
% State and Control Vector

x1=X(1);% u
x2=X(2);% v
x3=X(3);% w
x4=X(4);% p
x5=X(5);% q
x6=X(6);% r
x7=X(7);% phi
x8=X(8);% theta
x9=X(9);% psi

u1=U(1);% aileron
u2=U(2);% elevator
u3=U(3);% rudder
u4=U(4);% throttle1
u5=U(5);% throttle2


%%
% Constants
m=326.587;% mass [kg], assumed constant
cbar=0.48;% chord length [m]
ac=2.17;%aero center from nose [m], defined again later
lt=0.92;% distance of tail ac to body ac [m]
S=3.658;% wing area [m^2]
St=0.929;% tail area [m^2]

rho=0.7364;%air density [kg/m^3]

g=9.81;% gravitational acceleration [m/s^2]
depsda=0.25;% change in downwash wrt alpha [rad/rad]
alpha_L0=deg2rad(-4.5);% zero lift AoA [rad]


n=4.5837;% lift coeff linear region slope [1/rad]
alpha_switch=deg2rad(20);% change point of lift AoA regions
a3=0.8866;% lift coeff cubic region constants
a2=-7.5272;
a1=8.6947;
a0=-0.2152;

% CG location in Fm [m]
Xcg=-1.65;
Ycg=0;
Zcg=0;

% AC location in Fm [m]
Xac=-2.17;
Yac=0;
Zac=0;

% Engine1 location in Fm [m]
Xapt1=-3.6576;
Yapt1=0;
Zapt1=0;

% Engine2 location in Fm [m]
Xapt2=-3.6576;
Yapt2=0;
Zapt2=0;


%%
% Control saturations
% Achieved Explicitly


%%
% Intermediate variable
Va=sqrt(x1^2+x2^2+x3^2);% airspeed [m/s]

alpha=atan2(x3,x1);% AoA [rad]
beta=asin(x2/Va);% sidesplip angle [rad]

Q=0.5*rho*Va^2;% dynamic pressure [Pa]

wbe_b=[x4;x5;x6];% angular velocity vector [rad/s]
V_b=[x1;x2;x3];% velocity vector [m/s]


%%
% Aerodynamic force coeffs
% CL_wb
if alpha<=alpha_switch
    CL_wb=n*(alpha-alpha_L0);
else
    CL_wb=a3*alpha^3+a2*alpha^2+a1*alpha+a0;
end

% CL_t
epsilon=depsda*(alpha-alpha_L0);
alpha_t=alpha-epsilon+u2+x5*lt/Va;
CL_t=5.7296*(St/S)*alpha_t;

% toltal lift coeff
CL=CL_wb+CL_t;

% Drag coeff
CD=0.5727*alpha^2+0.2419*abs(alpha)+0.0160;

% Sideforce coeff
CY=-0.2678211*beta-0.1172088*u3;

%%
% Dimensional Aerodynamic Forces
FA_s=[-CD*Q*S CY*Q*S -CL*Q*S]';% in Fs

C_bs=[cos(alpha) 0 -sin(alpha)
    0 1 0
    sin(alpha) 0 cos(alpha)];
FA_b=C_bs*FA_s;% in Fb

%%
% Aerodynamic moment coeffs around ac

eta11=-0.0870139*beta;
eta21=0.0370211-3.7235077*(alpha);
eta31=0.0276400*beta;

eta=[eta11;eta21;eta31];

dCMdx=[-0.5700136 0 0.1390010
    0 -28.1586537 0
    0.0521194 0 -0.0154152];

dCMdu=[0.2704190 0 -0.0091219
    0 2.4964274 0
    0 0 0.0212393];

CMac_b=eta+dCMdx*wbe_b+dCMdu*[u1;u2;u3];

%%
% Aerodynamic moment about ac
MAac_b=CMac_b*Q*S*cbar;

%%
% Aerodynamic moment about cg
rcg_b=[Xcg;Ycg;Zcg];
rac_b=[Xac;Yac;Zac];

MAcg_b=MAac_b+cross(FA_b,rcg_b-rac_b);

%%
% Engine
F1=u4;
F2=u5;

FE1_b=[F1;0;0];
FE2_b=[F2;0;0];
FE_b=FE1_b+FE2_b;% assume that engines are aligned with Fb

mew1=[Xcg-Xapt1;Yapt1-Ycg;Zcg-Zapt1];
mew2=[Xcg-Xapt2;Yapt2-Ycg;Zcg-Zapt2];

MEcg1_b=cross(mew1,FE1_b);
MEcg2_b=cross(mew2,FE2_b);

% MEcg_b=MEcg1_b+MEcg2_b;% engine moments
MEcg_b=[0;0;0]+[0;155.688;0];%eninge shaft torque included

%%
% Gravity
g_b=[-g*sin(x8)
    g*cos(x8)*sin(x7)
    g*cos(x8)*cos(x7)];

Fg_b=m*g_b;

%%
% State derivatives
Ixx=6717.281;
Iyy=6343.943;
Izz=12484.576;

Ixy=0;
Ixz=477.031;
Iyz=0;

Ib=[Ixx Ixy Ixz
    Ixy Iyy Iyz
    Ixz Iyz Izz].*0.04214;% if openVSP gives lbm-ft^2
%.*1.3558179619;% if it was slugs-ft^2
invIb=inv(Ib);% inertia matrix and its inverse, [kg*m^2] and its inverse

F_b=Fg_b+FE_b+FA_b;% all forces
x1to3dot=F_b/m-cross(wbe_b,V_b);

Mcg_b=MAcg_b+MEcg_b;% all moments
x4to6dot=invIb*(Mcg_b-cross(wbe_b,Ib*wbe_b));

H_phi=[1 sin(x7)*tan(x8) cos(x7)*tan(x8)
    0 cos(x7) -sin(x7)
    0 sin(x7)/cos(x8) cos(x7)/cos(x8)];
x7to9dot=H_phi*wbe_b;% phitdot, thetadot, psidot

Xdot=[x1to3dot
    x4to6dot
    x7to9dot];

end





