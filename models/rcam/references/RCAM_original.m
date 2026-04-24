function [Xdot] = RCAM_original(X,U)
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
u2=U(2);% stablizer
u3=U(3);% rudder
u4=U(4);% throttle1
u5=U(5);% throttle2


%%
% Constants
m=120000;% mass [kg], assumed constant
cbar=6.6;% chord length [m]
lt=24.8;% distance of tail ac to body ac [m]
S=260;% wing area [m^2]
St=64;% tail area [m^2]

rho=1.225;% air density [kg/m^3]
g=9.81;% gravitational acceleration [m/s^2]
depsda=0.25;% change in downwash wrt alpha [rad/rad]
alpha_L0=deg2rad(-11.5);% zero lift AoA [rad]
n=5.5;% lift coeff linear region slope
a3=-768.5;% lift coeff cubic region constants
a2=609.2;
a1=-155.2;
a0=15.212;
alpha_switch=deg2rad(14.5);% change point of lift AoA regions


% CG location in Fm [m]
Xcg=cbar*0.23;
Ycg=cbar*0;
Zcg=cbar*0.1;

% AC location in Fm [m]
Xac=cbar*0.12;
Yac=cbar*0;
Zac=cbar*0;

% Engine1 location in Fm [m]
Xapt1=0;
Yapt1=-7.94;
Zapt1=-1.9;

% Engine2 location in Fm [m]
Xapt2=0;
Yapt2=7.94;
Zapt2=-1.9;


%%
% Control saturations
% Achieved Explicitly


%%
% Intermediate variable
Va=sqrt(x1^2+x2^2+x3^2);% airspeed

alpha=atan2(x3,x1);% AoA
beta=asin(x2/Va);% sidesplip angle

Q=0.5*rho*Va^2;% dynamic pressure

wbe_b=[x4;x5;x6];% angular velocity vector
V_b=[x1;x2;x3];% velocity vector


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
alpha_t=alpha-epsilon+u2+1.3*x5*lt/Va;
CL_t=3.1*(St/S)*alpha_t;

% toltal lift coeff
CL=CL_wb+CL_t;

% Drag coeff
CD=0.13+0.07*(5.5*alpha+0.654)^2;

% Sideforce coeff
CY=-1.6*beta+0.24*u3;

%%
% Dimensional Aerodynamic Forces
FA_s=[-CD*Q*S CY*Q*S -CL*Q*S]';% in Fs

C_bs=[cos(alpha) 0 -sin(alpha)
    0 1 0
    sin(alpha) 0 cos(alpha)];
FA_b=C_bs*FA_s;% in Fb

%%
% Aerodynamic moment coeffs around ac

eta11=-1.4*beta;
eta21=-0.59-(3.1*(St*lt)/(S*cbar))*(alpha-epsilon);
eta31=(1-alpha*(180/(15*pi)))*beta;

eta=[eta11;eta21;eta31];

dCMdx=(cbar/Va)*[-11 0 5
    0 (-4.03*(St*lt^2)/(S*cbar^2)) 0
    1.7 0 -11.5];

dCMdu=[-0.6 0 0.22
    0 (-3.1*(St*lt)/(S*cbar)) 0
    0 0 -0.63];

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
F1=u4*m*g;
F2=u5*m*g;

FE1_b=[F1;0;0];
FE2_b=[F2;0;0];
FE_b=FE1_b+FE2_b;% assume that engines are aligned with Fb

mew1=[Xcg-Xapt1;Yapt1-Ycg;Zcg-Zapt1];
mew2=[Xcg-Xapt2;Yapt2-Ycg;Zcg-Zapt2];

MEcg1_b=cross(mew1,FE1_b);
MEcg2_b=cross(mew2,FE2_b);

MEcg_b=MEcg1_b+MEcg2_b;% engine moments
MEcg_b=MEcg_b+[0;155.688;0];%eninge shaft torque included

%%
% Gravity
g_b=[-g*sin(x8)
    g*cos(x8)*sin(x7)
    g*cos(x8)*cos(x7)];

Fg_b=m*g_b;

%%
% State derivatives

Ib=m*[40.07 0 -2.0923
    0 64 0
    -2.0923 0 99.92];
invIb=inv(Ib);% inertia matrix and its inverse

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





