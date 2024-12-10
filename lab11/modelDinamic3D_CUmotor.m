clear all;
clc;
close all;

syms q1 q2 dq1 dq2

syms L1 L2 m1 m2 g I1x I2y b1 b2 Jm1 Jm2 r1 r2 fb1 fb2 Km1 Km2 Ra1 Ra2 Fm1 Fm2

D = [I1x+L1^2*m1/4+L1^2*m2+L2^2*m2/4*cos(q2)^2 + L1*L2*m2*cos(q2) ,  0;
      0 ,                                                            m2*L2^2/4 + I2y]

C = [-L2^2*m2/8*sin(2*q2)*dq2-1/2*L1*L2*m2*sin(q2)*dq2 , -L2^2*m2/8*sin(2*q2)*dq1-1/2*L1*L2*m2*sin(q2)*dq1;
     L2^2*m2/8*sin(2*q2)*dq1 + 1/2*L1*L2*m2*sin(q2)*dq1 , 0 ]

G = [(-m1*g*L1+2*m2*g*L1)*sin(q1)/2-m2*g*L2*sin(q1)*cos(q2)/2;
      -m2*g*L2/2*cos(q1)*sin(q2)]

F = diag([fb1,fb2])*[dq1 ;dq2];

R= diag([r1,r2])

B = diag([b1,b2])

Jm = diag([Jm1,Jm2])

Fm = [Fm1 ; Fm2];

Km = diag([Km1,Km2])

Ra = diag([Ra1,Ra2])

D_prim = D+Jm
C_prim = B+R^2*C
F_prim = R*Fm + R^2*F
G_prim = R^2*G

K_prim = R*(Km.*Ra)

%%

L1 = 0.095
L2 = 0.1
m1 = 0.095
m2 = 0.37
g = 9.81
I1x = 2.27*10^-2
I2y = 2.27*10^-2


b1 = 0.24
b2 = 0.16

r1 = 1
r2 = 1

Jm1 = 0;
Jm2 = 0;

fb1 = 0
fb2 = 0

Km1 = 1
Km2 = 1

Ra1 = 1;
Ra2 = 1

Fm1 = 0;
Fm2 = 0;

D_prim=vpa(eval(D_prim),5)
C_prim=vpa(eval(C_prim),5)
G_prim=vpa(eval(G_prim),5)
F_prim=vpa(eval(F_prim),5)