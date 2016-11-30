syms d g11 g12 g21 g22 x x_prim x_bis a11 a12 a13 a14 a21 a22 a23 a24 o o_prim o_bis b11 b12 b21 b22 v_m K_t R_m l_w l_b I_w m_w I_b m_b K_e b_f g;
g = 9.8;
b_f = 0;
m_b = 0.381;
l_b = 0.112;
b_f = 0;
m_b = 0.381;
l_b = 0.112;
I_b = 0.00616;
m_w = 0.036;
l_w = 0.021;
I_w = 0.00000746;
R_m = 4.4;
L_m = 0;
b_m = 0;
K_e = 0.444;
K_t = 0.470;

I_b = 0.00616;
m_w = 0.036;
l_w = 0.021;
I_w = 0.00000746;
R_m = 4.4;
L_m = 0;
b_m = 0;
K_e = 0.444;
K_t = 0.470;
g11 = m_b * l_b;
g12 = I_b + m_b * l_b^2;
g21 = m_w * l_w + I_w / l_w + m_b * l_w;
g22 = m_b * l_w * l_b;
M_g = [g11 g12; g21 g22];
a11 = 0;
a12 = b_f / l_w + (K_t * K_e) / (R_m * l_w);
a13 = m_b * l_b * g;
a14 = -b_f -((K_t * K_e) / R_m);
a21 = 0;
a22 = -((K_t * K_e) / (R_m * l_w)) - (b_f / l_w);
a23 = 0;
a24 = ((K_t * K_e) / R_m) + b_f;

b11 = -(K_t / R_m);
b12 = l_b;
b21 = K_t / R_m;
b22 = l_w;


M_a = [a11 a12 a13 a14; a21 a22 a23 a24];
M_b = [b11; b21];
M_b_sim = [b11 b12 ; b21 b22];
M_x = [x ; x_prim; o; o_prim];
M_xo = [x_bis; o_bis];

INV_M_g = inv(M_g);

At = INV_M_g * M_a;
Bt = INV_M_g * M_b;
Bt_sim = INV_M_g * M_b_sim;

A = [0 1 0 0; At(1,:); 0 0 0 1; At(2,:)];
B = [0; Bt(1,:); 0; Bt(2,:)];
C = [0 0 1 0];
D = [0];

B_sim = [0 0; Bt_sim(1,:); 0 0; Bt_sim(2,:)];
C_sim = [1 0 0 0; 0 1 0 0; 0 0 1 0; 0 0 0 1];
D_sim = [0 0; 0 0; 0 0; 0 0];

sys = ss(A, B, C, D);

[num, den] = ss2tf(A, B, C, D);%, 2); % A, B, C, D, 2?
[z, p, k] = tf2zp(num, den);

p1 = p(2);
p2 = p(3);
p3 = p(4);
p4 = p(1);

p1_D = p1;
p2_D = -10;
p3_D = p3;
p4_D = p4;

PID_P = ((p2_D * p3_D + p1_D * p3_D + p1_D * p2_D - p2 * p3 - p1 * p3 - p1 * p2) / k);
PID_I = (p1 * p2 * p3 - p1_D * p2_D * p3_D) / k;
PID_D = (p1 + p2 + p3 - p1_D - p2_D - p3_D) / k;

syms W; % closed loop tf
W = tf([(k * PID_D) (k * PID_P) (k * PID_I)], [1 (PID_D * k - p1 - p2 - p3) (PID_P * k + p2 * p3 + p1 * p3 + p1 * p2) (PID_I * k - p1 * p2 * p3)]);

O_M3 = obsv(A(1:3, 1:3), C(:, 1:3));
O_M = obsv(A,C);

C_M3 = ctrb(A(1:3, 1:3), B(1:3, :));
C_M = ctrb(A, B);
% C_TF = tf([PID_D PID_P PID_I], [1 0]);
% C_FB_TF = C_TF / (1 + C_TF);
s = tf('s');
[W_num, W_den] = tfdata(W, 'v');
[part_f_num, part_f_den, part_f_const] = residue(W_num, W_den);

SO_TF = (part_f_num(2) / (s - part_f_den(2) + part_f_num(3) / (s - part_f_den(3))));