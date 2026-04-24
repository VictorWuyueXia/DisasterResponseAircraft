clear; 
close all; 
clc;

% REQUIRED DATA:
% Wing area, S (m^2) 
% Wing taperd ratio, lambda 
% Stall speed, V_s(m/s) 
% Cl vs AoA slope for wing, C_Law 
% Air density, rho (kg/m^3)

% Constants
S = 0.929; lambda = 0.5;
V_s = 52.3; %knot 
C_Law = 6.07; rho = 1.225;

bi_b = 0.1; bo_b = 1;
Ca_C = 0.45;
% C_a/C determines aileron effectiveness fig.12.12
%% 
% 

% control surface AoA surface effectiveness parameter
tau = 0.65;

b = 2.43%HT span
Cbar = 1.25%HT mean cord
C_r = 0.5%HT root cord
yi = bi_b*b/2%inboard elevator location
yo = bo_b*b/2%outboard elevator location
C_LdA_1 = (2*C_Law*tau*C_r)/(S*b);
C_LdA_2 = ((yo^2/2)+(2/3)*((lambda-1)/b)*yo^3);
C_LdA_3 = ((yi^2/2)+(2/3)*((lambda-1)/b)*yi^3);
C_LdA = C_LdA_1*(C_LdA_2 - C_LdA_3)*pi/180 % [1/deg]

% MAX elevator deflection
dA_max = 15;%[deg]

C_L = C_LdA * dA_max