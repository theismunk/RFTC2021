clear all;
close all;
clc;
%%
addpath('./Data')
addpath('./Models')

load('ECP_values.mat');
% Physical system parameters
J_1 = ECP_values(1);            % Disk 1 inertia kgm^2
J_2 = ECP_values(2);            % Disk 2 inertia kgm^2
J_3 = ECP_values(3);            % Disk 3 inertia kgm^2
k_1 = ECP_values(4);            % Shaft 1-2 stiffness Nm/rad
k_2 = ECP_values(5);            % Shaft 2-3 stiffness Nm/rad
b_1 = mean(ECP_values([6 7]));  % Disk 1 damping and friction Nms/rad
b_2 = mean(ECP_values([8 9]));  % Disk 2 damping and friction Nms/rad
b_3 = mean(ECP_values([10 11]));% Disk 3 damping and friction Nms/rad
T_Cp = ECP_values(12);          % Disk 1 Coulomb friction in positive direction
T_Cm = ECP_values(13);          % Disk 1 Coulomb friction in negative direction
atan_scale = 100;               % Sign approximation factor
w_th = 0.75;                     % Threshold angular velocity rad/s

% The system states are [theta_1;omega_1;theta_2;omega_2;theta_3;omega_3]
x_0 = [0;0;0;0;0;0];            % Initial conditions
T_s = 0.004;                    % Sampling period
sigma_meas = 0.0093*eye(3);     % Measurements covariance matrix

%% State space representation
A = [0, 1, 0, 0, 0, 0; 
    -k_1/J_1, -b_1/J_1, k_1/J_1, 0, 0, 0; 
    0,0,0, 1,0,0; 
    k_1/J_2, 0, -(k_1 + k_2)/J_2, -b_2/J_2, k_2/J_2, 0; 
    0,0,0,0,0,1; 
    0,0, k_2/J_3, 0, -k_2/J_3, -b_3/J_3];

B = [0,0;
    1/J_1, 0; 
    0,0; 
    0,1/J_2; 
    zeros(2)];

C = [1, zeros(1,5); 0,0,1,zeros(1,3); zeros(1,4), 1, 0]; 
D = zeros(3,2); 
Ex = [0; -1/J_1; zeros(4,1)]; 
Ey = zeros(3,1);
Fx = [0,     0, 0, 0, 0;
    1/J_1,     0, 0, 0, 0;
    0,     0, 0, 0, 0;
    0, 1/J_2, 0, 0, 0;
    0,     0, 0, 0, 0;
    0,     0, 0, 0, 0];
Fy = [0,0,1,0,0;
       0,0,0,1,0;
       0,0,0,0,1];

% Discrete time

sys = ss(A, [B, Ex, Fx], C, [D, Ey, Fy]);
sys_d = c2d(sys,T_s, 'tustin');
F_d = sys_d.A;
G_d = sys_d.B(:,1:2);

% State-feedback LQR design
Q_c = diag([2 0 2 0 2.5 0.0024]);
R_c = diag([10 10]);
K_c = dlqr(F_d,G_d,Q_c,R_c);

% Scaling of reference
C_ref = pinv(C(3,:) * inv(eye(6) - F_d + G_d * K_c) * G_d * K_c);

% Kalman filter with friction estimation - DO NOT MODIFY
F_aug = [F_d G_d(:,1);zeros(1,6) 1];
G_aug = [G_d;0 0];
C_aug = [C zeros(3,1)];
% Kalman gain
L_aug = dlqe(F_aug,eye(7),C_aug,1e-3*eye(7),deg2rad(0.0056)*eye(3));
L_o = L_aug(1:6,:);
L_d = L_aug(7,:);

%% Residual filter design

%% Strong and weak detectability
H_rf = tf(0);

%% GLR
f_m = [0;-0.025;0];  % Sensor fault vector (added to [y1;y2;y3])

%% Virtual actuator
% Failure in actuator 2
% Do the design first in continuous time
va_eig = [-0.3 + 50i, -0.3 - 50i, -0.6+20i, -0.6-20i, -1, -3, -2];     % Continuous time eigenvalues
va_eig_d = exp(va_eig*T_s);  % Discrete time eigenvalues
% Then discretise your VA

A_aug = [A B(:,1);zeros(1,6) 0];

B_f = B;
B_f(:,2) = 0;

B_aug = [B; 0, 0];
B_f_aug = [B_f; 0, 0];

G_f = G_d;
G_f(:,2) = 0;

G_f_aug = [G_f; 0, 0];

if rank(B_f_aug) == rank([B_aug, B_f_aug])
    disp('Perfect static matching for actuator fault');
else
    disp('Imperfect static matching for actuator fault');
end

if rank(ctrb(F_aug,G_f_aug)) == length(F_aug)
    disp('Faulty system is controllable');
else
    disp('Faulty system is not controllable');
end

% Continuous time
M = place(A_aug,B_f_aug,va_eig);
A_D = A_aug-B_f_aug*M;
N_D = pinv(B_f_aug)*B_aug;
B_D = B_aug-B_f_aug*N_D;
C_D = C_aug;

% % Discrete time
% M_d = place(F,G_f,va_eig_d);
% F_D = F-G_f*M_d;
% N_D_d = pinv(G_f)*G;
% G_D = G-G_f*N_D_d;
% C_D = C;

%% Simulation for sensor fault (f_u = 0)
simTime = 45;                   % Simulation duration in seconds
f_u_time = 25;                  % Actuator fault occurence time
detect_time = f_u_time + 3.75;
f_u = [0;0];                    % Actuator fault vector (added to [u1;u2])
u_fault = 0;                    % Disable VA meachanism
f_m_time = 8.5;                 % Sensor fault occurence time
sim('threeDiskOscillatorRig');

%% Simulation for actuator fault (f_m = 0)
f_u = [0;-0.1];                 % Actuator fault vector (added to [u1;u2])
u_fault = 1;                    % Enable VA meachanism
f_m = [0;0;0];                  % Sensor fault vector (added to [y1;y2;y3])
sim('threeDiskOscillatorRig_solution');

%% Plot settings
set(0,'DefaultTextInterpreter','latex');
set(0,'DefaultAxesFontSize',20);
set(0,'DefaultLineLineWidth', 2);

