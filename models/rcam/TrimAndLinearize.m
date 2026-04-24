% Trim the system

clear;close all;clc;

p0=[0 0 0]';
h=4000;
geo0=[deg2rad(-157.9251) deg2rad(21.32451) h]';

tStop=300;

u0=[0;0;0;0.0079;0.0079];
x0=[61.7;0;0;0;0;0;0;0;0];
y0=[61.7;0];
ix=[2;7;9];
iy=[1;2];
[x_trimmed,u_trimmed,y_trimmed,dx_trimmed] = trim('RCAM_forTrimm',x0,u0,y0,ix,[],iy);
x_trimmed=x_trimmed
u_trimmed=u_trimmed

x=x_trimmed;
u=u_trimmed;

linsys=linmod('RCAM_forTrimm',x,u);
A=linsys.a
B=linsys.b;
C=linsys.c;
D=linsys.d;

C=eye(9);
D=zeros(9,5);


save('TrimmedLinearizedSystem','A','B','C','D',"x",'u');

% Split the A matrix into 2 seperate ones for longitudinal and lateral directions

Tinv=zeros(7);
Tinv(1,1)=1;
Tinv(2,3)=1;
Tinv(3,5)=1;
Tinv(4,8)=1;
Tinv(5,2)=1;
Tinv(6,4)=1;
Tinv(7,6)=1;
Tinv(8,7)=1;
Tinv(9,9)=1;
T=inv(Tinv);

Atilde=inv(T)*A*T;
Along=Atilde(1:4,1:4)%[u;w;q;theta]
Alat=Atilde(5:8,5:8)%[v;p;r;phi]
Apsi=Atilde(9,7);

% For longitudinal direction

[eigenVectors,eigenValues]=eig(Along);%[u;w;q;theta]
longitudinalEigenValues=diag(eigenValues)
myfun=@(t,lambda,v)real(v.*exp(lambda.*t));

t=linspace(0,10,1000);

figure
lambda=longitudinalEigenValues(1);
v=eigenVectors(2,1);
plot(t,myfun(t,lambda,v),'LineWidth',2)
hold on
plot(t,real(exp(real(lambda).*t)),'r--')
plot(t,-1.*real(exp(real(lambda).*t)),'r--')
xlabel('time [s]')
ylabel('Impulse Response')
title('ShortPeriodMode')

t=linspace(0,300,1000);

figure
lambda=longitudinalEigenValues(3);
v=eigenVectors(1,3);
plot(t,myfun(t,lambda,v),'LineWidth',2)
hold on
plot(t,real(exp(real(lambda).*t)),'r--')
plot(t,-1.*real(exp(real(lambda).*t)),'r--')
xlabel('time [s]')
ylabel('Impulse Response')
title('PhugoidMode')

% For lateral direction

[eigenVectors,eigenValues]=eig(Alat);%[v;p;r;phi]
lateralEigenValues=diag(eigenValues)
lateralEigenVectors=eigenVectors;

t=linspace(0,60,1000);

figure
lambda=lateralEigenValues(1);
v=lateralEigenVectors(1,1);
plot(t,myfun(t,lambda,v),'LineWidth',2)
hold on
plot(t,real(exp(real(lambda).*t)),'r--')
plot(t,-1.*real(exp(real(lambda).*t)),'r--')
xlabel('time [s]')
ylabel('Impulse Response')
title('RollSubsidenceMode')

t=linspace(0,300,1000);

figure
lambda=lateralEigenValues(2);
v=lateralEigenVectors(1,2);
plot(t,myfun(t,lambda,v),'LineWidth',2)
hold on
plot(t,real(exp(real(lambda).*t)),'r--')
plot(t,-1.*real(exp(real(lambda).*t)),'r--')
xlabel('time [s]')
ylabel('Impulse Response')
title('DutchRollMode')

t=linspace(0,60,1000);

figure
lambda=lateralEigenValues(4);
v=lateralEigenVectors(1,4);
plot(t,myfun(t,lambda,v),'LineWidth',2)
hold on
plot(t,real(exp(real(lambda).*t)),'r--')
plot(t,-1.*real(exp(real(lambda).*t)),'r--')
xlabel('time [s]')
ylabel('Impulse Response')
title('SpiralDivergenceMode')