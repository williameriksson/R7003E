close all;
clear all;
clc;

script2_adding_d;

% kP = -46.6; 
% kI = -260;
% kD = -0.1;

C = [1 0 0 0 ; 0 0 1 0];

% load the PID (From lab raport A)
kP = -83.9386;
kI = -469.2608;
kD = -0.1746;

L = (place(A', C', afBotPoles));
L = L';

%
syms x z u chi y_acc y_not_acc x_dot x_w x_w_dot theta_b theta_b_dot C_acc C_not_acc;
x = [x_w ; x_w_dot ; theta_b ; theta_b_dot];

C_acc = [1 0 0 0];
C_not_acc = [0 0 1 0];

V = [0 1 0 0 ; 0 0 1 0; 0 0 0 1];
T_inv = [C_acc ; V];
T = inv(T_inv); 

x_dot = A*x + B*u;
y_acc = C_acc * x;
y_not_acc = C_not_acc * x;

z = T_inv * x;
chi = [x_w_dot ; theta_b ; theta_b_dot];


A_tilde = T_inv * A * T;
B_tilde = T_inv * B;
C_acc_tilde = C_acc * T;
C_not_acc_tilde = C_not_acc * T;

z_dot = A_tilde * z + B_tilde * u;
y_acc = C_acc_tilde * z;
y_not_acc = C_not_acc_tilde * z;

%rewrite system as
% A_tilde_y_y = A(1:3, 1);
% A_tilde_y_chi = A(1:3, 2:4);
% A_tilde_chi_y = A(4, 1);
% A_tilde_chi_chi = A(4, 2:4);

A_tilde_y_y = A(1, 1);
A_tilde_y_chi = A(1, 2:4);
A_tilde_chi_y = A(2:4, 1);
A_tilde_chi_chi = A(2:4, 2:4);

B_tilde_y = B(1, 1);
B_tilde_chi = B(2:4, 1);

C_tilde_y = C(1, 1);
C_tilde_chi = C(1, 2:4);

A_novel = A_tilde_chi_chi;
C_novel = [A_tilde_y_chi ; C_tilde_chi];

lolPoles = (real(afSortedRoots(2:4))*5)+complex(afSortedRoots(2:4));

L_red = (place(A_novel', C_novel', lolPoles))';

L_red_acc = L_red(1:3, 1);
L_red_not_acc = L_red(1:3, 2);

M1 = (A_tilde_chi_chi - (L_red_acc * A_tilde_y_chi) - (L_red_not_acc * C_tilde_chi));
M2 = (B_tilde_chi - L_red_acc * B_tilde_y);
M3 = (A_tilde_chi_y - L_red_acc * A_tilde_y_y - L_red_not_acc * C_tilde_y);
M4 = (L_red_not_acc);
M5 = (L_red_acc);
M6 = [1 ; 0 ; 0 ; 0];

M7 = [0 0 0 ; 1 0 0 ; 0 1 0; 0 0 1];





% M1 = 1.0e+03 * [
%    -0.4358   -0.0072    0.0091
%    -0.0011   -0.0340    0.0010
%     1.8717   -0.0165   -0.0400
% ]
% 
% M2 = [
%    20.6000
%          0
%   -90.0000
% ]
% 
% M3 = [
%      0
%      0
%      0
% ]
% 
% M4 = [
%     1.1459
%    33.9942
%    78.4584
% ]
% 
% M5 = [
%     0.7598
%     1.1459
%    31.6987
% ]
% 
% M6 = [
%      1
%      0
%      0
%      0
% ]
% 
% M7 = [
%      0     0     0
%      1     0     0
%      0     1     0
%      0     0     1
% ]
% 
% L = [
%    15.0196    0.5900
%    -2.7287   18.9345
%     0.6038   44.9804
%    22.1715  435.4379
% ]
% 
% A = 1.0e+03 * [
%          0    0.0010         0         0
%          0   -0.4350   -0.0061    0.0091
%          0         0         0    0.0010
%          0    1.9034    0.0620   -0.0400
% ]
% 
% B = [
%          0
%    20.6000
%          0
%   -90.0000
% ]
% 
% C = [
%      1     0     0     0
%      0     0     1     0
% ]
% 
% D = [
%      0
% ]

