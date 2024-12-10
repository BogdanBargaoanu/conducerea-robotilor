function [ddq1,ddq2] = robot3D(tau1,tau2,q1,q2,dq1,dq2)


D=[0.003515*cos(q2)+0.000925*cos(q2)^2+0.026254,      0
                                               0, 0.023625];

C=[0.24-0.5*dq2*(0.000925*sin(2.0*q2)+0.003515*sin(q2)), -0.004625*dq1*(0.1*sin(2.0*q2)+0.38*sin(q2))
    0.004625*dq1*(0.1*sin(2.0*q2)+0.38*sin(q2)),                0.16];

G = [-4.905*sin(q1)*(0.037*cos(q2)+0.079325)
                 -0.18149*cos(q1)*sin(q2)];

tau = [tau1;tau2];
ddq=inv(D)*(-C*[dq1;dq2]-G+tau);

F_prim =[0;0];
g1=G(1);
g2=G(2);


ddq=inv(D)*(-C*[dq1;dq2]-G+[u1;u2]-F_prim);

ddq1=ddq(1);
ddq2=ddq(2);
                                     