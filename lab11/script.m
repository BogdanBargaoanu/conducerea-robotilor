Kp1 = 1;
Kp2 = 2;
Kd1 = 1;
Kd2 = 9;
q1 = 3;
q2 = 3;
G1 = 1;
G2 = 2;
D_diag_const = [0.026254 0; 0 0.023625];
C_diag_const = [0.24 0; 0 0.16];

zeta = 1;
wn = 2 * pi * 2;
Kp1 = D_diag_const(1,1) * wn^2;
Kp2 = D_diag_const(2,2) * wn^2;
Kd1 = 2 * zeta * wn * D_diag_const(1,1) - C_diag_const(1,1);
Kd2 = 2 * zeta * wn * D_diag_const(2,2) - C_diag_const(2,2);