syms g11 g12 g21 g22 x x_prim x_bis a11 a12 a13 a14 a21 a22 a23 a24 o o_prim o_bis b1 b2 v_m K_t R_m l_w l_b I_w m_w I_b m_b K_e b_f g;
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
M_a = [a11 a12 a13 a14; a21 a22 a23 a24];
b1 = -(K_t / R_m);
b2 = K_t / R_m;
M_b = [b1 ; b2];
M_x = [x ; x_prim; o; o_prim];
M_xo = [x_bis; o_bis];
INV_M_g = inv(M_g);
At = INV_M_g * M_a;
Bt = INV_M_g * M_b;
A = [0 0 1 0; At(1,:); 0 0 0 1; At(2,:)];
B = [0; Bt(1,:); 0; Bt(2,:)];
C = [0; 0; 1; 0]';
D = [0];
%syms x21 x22 x23 x24 x41 x42 x43 x44 s
%fictionalA = [0 1 0 0; x21 x22 x23 x24; 0 0 0 1; x41 x42 x43 x44];
%sI = [s 0 0 0; 0 s 0 0; 0 0 s 0; 0 0 0 s];
%fictionalDiff = sI - fictionalA;
%detFictionalDiff = det(fictionalDiff);

sys = ss(A, B, C, D);
[num, den] = ss2tf(A, B, C, D);
[z, p, k] = tf2zp(num, den);

p1 = p(2);
p2 = p(3);
p3 = p(4);
p1_D = p1;
p2_D = -4;
p3_D = p3;

PID_P = (p2_D * p3_D + p1_D * p3_D + p1_D * p2_D - p2 * p3 - p1 * p3 - p1 * p2) / k;
PID_I = (p1 * p2 * p3 - p1_D * p2_D * p3_D) / k;
PID_D = (p1 + p2 + p3 - p1_D - p2_D - p3_D) / k;

syms W; % closed loop tf
%W = k * (s * PID_P + PID_I + s^2 * PID_D) / ( (s - p1) * (s - p2) * (s - p3) + k * (s * PID_P + PID_I + s^2 * PID_D) );
W = tf([(k * PID_D) (k * PID_P) (k * PID_I)], [1 (PID_D * k - p1 - p2 - p3) (PID_P * k + p2 * p3 + p1 * p3 + p1 * p2) (PID_I * k - p1 * p2 * p3)]);
%PID_tf = tf(pid(PID_P, PID_I, PID_D));
%system_tf = tf([k 0], [1 (-p1 * -p2 * -p3) (p2 * p3 + p1 * p3 + p1 * p2) -(p1 * p2 * p3)]);
%W = PID_tf * system_tf / (1 + PID_tf * system_tf);

