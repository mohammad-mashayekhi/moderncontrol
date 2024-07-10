% modeling of levitation ball

clc;
clear;
close all;


%% Define parameters

syms L E R x1 x3 m g u0 ur n s x2 l r2 r1 S t taw

% L : inductance
% E : input Voltage (input of system)
% R : the resistance of coil
% m : the math of ball
% n : the turns of coil
% l : the heigh of coil
% r1: innner radius coil
% r2: out Radius of coil
% S : the same surface between ball and coil

%% initialization and assignment

L_val = 7.4 * 10^(-3) ; 
R_val = 1 ;
x1_val = 1;
x2_val = 0.048 ; 
x3_val = 0;
m_val = 0.055;
g_val = 9.80665 ;
u0_val = 4*pi*10^(-7);
ur_val = u0_val * 1.00000037;
n_val = 100;
l_val = 0.012;
r2_val = 0.0225;
r1_val = 0.008 ;
% iron density = 7.874 g/cm^3;
S_val = 8.83552 * 10^(-6);

%% calculcate Jacobian and linearization

f = [(1/L)*(E-R*x1)   ;   x3    ;    (1/m) * (m*g-(((u0*ur*n^2)^2*S)/(8*u0)) * (x1^2) * ((x2+l)*asin(r2/(x2+l))-asin(r1/(x2+l))+ x2 * (asin(r1/x2)-asin(r2/x2)))^2)];

A_jacob = jacobian(f,[x1,x2,x3]);
B_jacob = jacobian(f,E);


A_jacob_numeric = subs(A_jacob , [L,R,x1,x2,x3,m,g,u0,ur,n,l,r1,r2,S],[L_val,R_val,x1_val,x2_val,x3_val,m_val,g_val,u0_val,ur_val,n_val,l_val,r1_val,r2_val,S_val]);
A_jacob_numeric = double(A_jacob_numeric);


B_jacob = subs(B_jacob,[L,R,x1,x2,x3,m,g,u0,ur,n,l,r1,r2,S],[L_val,R_val,x1_val,x2_val,x3_val,m_val,g_val,u0_val,ur_val,n_val,l_val,r1_val,r2_val,S_val]);
B_jacob_numeric = double(B_jacob);
%% Making system
% state k
%K = [162 72 -120.1];
A = A_jacob_numeric;
B = B_jacob_numeric;
K = acker(A,B,[-9 -3-3i -3+3i])
Acl= A-B*K;
%B = [0;0;0];
C = [0 , 0 , 1];
D = 0;

system = ss(A,B,C,D);

%% Controlability
Controlability = ctrb(A , B_jacob_numeric)
CTR_RANK = rank(Controlability)
%% Visibility
Visibility = obsv(A,C)
OBS_RANK = rank(Visibility)
%% Plot
step(system);

