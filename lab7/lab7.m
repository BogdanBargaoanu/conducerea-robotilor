%% RTTT syms
syms q1
syms q2
syms q3
syms L1
syms L2
 
T01 = transl([0; 0; L1]) * rot(q1, 'z');
T12 = transl([0; 0; q2]);
T23 = transl([0; q3; 0]);
T34 = transl([0; L2; 0]);
T = T01 * T12 * T23 * T34; 
M{1} = T01;
M{2} = T12;
M{3} = T23;
M{4} = T34;
%DrawRobot(M);

%% simulare
q1 = 0:0.1:2*pi;
q2 = ones(1,length(q1));
q3 = ones(1,length(q1));

p_vector = mgd_vectorial(q1, q2, q3);
px = p_vector(1:length(q1));
py = p_vector(length(q1)+1:2*length(q1));
pz = p_vector(2*length(q1)+1:end);
%plot3(px, py, pz)

q_vector = mgi_vectorial(px, py, pz);
q1_est = q_vector(1:length(q1));
q2_est = q_vector(length(q1)+1:2*length(q1));
q3_est = q_vector(2*length(q1)+1:end);

%figure;
%plot3(q1, q2, q3);
%hold on
%plot3(q1_est, q2_est, q3_est);


t0 = 1;
tf = 15;
t = t0:0.2:tf;
q1t0 = 0.2;
q1tf = 1.6;
q2t0 = 1;
q2tf = 1;
q3t0 = 1;
q3tf = 1;
c1 = traj_4(t0, tf, q1t0, q1tf, 0, 0);
c2 = traj_4(t0, tf, q2t0, q2tf, 0, 0);
c3 = traj_4(t0, tf, q3t0, q3tf, 0, 0);

q1_p = inter_poli(t, c1);
q2_p = inter_poli(t, c2);
q3_p = inter_poli(t, c3);

p_vector_p = mgd_vectorial(q1_p, q2_p, q3_p);
px_p = p_vector_p(1:length(q1_p));
py_p = p_vector_p(length(q1_p)+1:2*length(q1_p));
pz_p = p_vector_p(2*length(q1_p)+1:end);
plot3(px_p, py_p, pz_p, 'm-*')

[ddx1, dx1, x1, tt1] = cart_blend(t0, tf, px_p(1), px_p(length(px_p)), 50);
[ddx2, dx2, x2, tt2] = cart_blend(t0, tf, py_p(1), py_p(length(py_p)), 50);
[ddx3, dx3, x3, tt3] = cart_blend(t0, tf, py_p(1), py_p(length(py_p)), 50);

figure; 
plot(tt1, dx1);
figure;
plot(tt2, dx2);
figure;
plot(tt3, dx3);

figure;
plot(tt1, x1);
figure;
plot(tt2, x2);
figure;
plot(tt3, x3);

q_vector_p = mgi_vectorial(x1, x2, x3);
q1_est_p = q_vector_p(1:length(x1));
q2_est_p = q_vector_p(length(x2)+1:2*length(x2));
q3_est_p = q_vector_p(2*length(x3)+1:end);

t = 1:71;
figure;
plot(tt1, q1_est_p);
figure;
plot(tt2, q2_est_p);
figure;
plot(tt3, q3_est_p);

function p_vector = mgd_vectorial(q1, q2, q3)
L1 = 20;
L2 = 30;
px = -sin(q1).*(q3 + L2);
py = cos(q1).*(q3 + L2);
pz = q2 + L1;
p_vector = [px py pz];
end

function q_vector = mgi_vectorial(px, py, pz)
L1 = 20;
L2 = 30;
q1 = atan2(py,px);
q2 = pz - L1;
q3 = -L2 + sqrt(px.^2 + py.^2);
q_vector = [q1 q2 q3];
end
function mat_transl = transl(v)
     mat_transl = [eye(3) v];
     mat_transl = [mat_transl; [0 0 0 1]];
end
 
function mat_rotatie = rot(nr, axa)
%nr = deg2rad(nr);
switch axa
    case 'x'
        mat_rotatie = [1 0 0 0; 0 cos(nr) -sin(nr) 0; 0 sin(nr) cos(nr) 0; 0 0 0 1];
    case 'y'
        mat_rotatie = [cos(nr) 0 sin(nr) 0; 0 1 0 0; -sin(nr) 0 cos(nr) 0; 0 0 0 1;];
    case 'z'
        mat_rotatie = [cos(nr) -sin(nr) 0 0; sin(nr) cos(nr) 0 0; 0 0 1 0; 0 0 0 1;];
end
end
 
function t_inv = transf_inv(T)
     t_inv = eye(4);
     t_inv(1:3,1:3) = T(1:3,1:3)';
     t_inv(1:3,4) = T(1:3,1:3)'*-T(1:3,4);
end


function c = traj_4(t0, tf, qt0, qtf, dqt0, dqtf)
dqt0 = 0;
dqtf = 0;
 
A = [1 t0 t0^2 t0^3;
    1 tf tf^2 tf^3;
    0 1 2*t0 3*t0^2;
    0 1 2*tf 3*tf^2;];

b = [qt0; qtf; dqt0; dqtf];
c = inv(A) * b;
end

function q = inter_poli(t, c)
q = c' * [ones(1,length(t)); t; t.^2; t.^3];
end

function [at,vt,xt,tt] = cart_blend(t0,tf,x0,xf,m)
t = t0:1/m:tf;
v_min = (xf - x0) / (tf - t0);
v_max = (xf - x0) / (tf/2 - t0);
  
vb = (v_min + v_max) / 2;

tb = (x0-xf+vb*(tf-t0))/vb;
 
 
a0_down = x0;
a0_up = xf - vb*tb/2;
a1_up = vb;
a1_down = 0;
a2_down = vb/ 2/ (tb-t0);
a2_up = - vb/2/tb;
 
 
%xt = zeros;
%dxt = [];
%ddxt = [];
 
xtb = (x0+xf)/2 - vb*(tf/2-tb);
 
xt = zeros;
vt = [];
at = [];
for i=1:length(t)
 if (t(i)>= t0 && t(i) < tb)
   xt(i) = a0_down+a1_down*(t(i)-t0)+a2_down*(t(i)-t0)^2;
   vt(i) = a1_down + 2*a2_down*(t(i)-t0);
   at(i) = 2*a2_down;
 elseif (t(i)>= tb && t(i) < tf - tb)
    xt(i) = xtb+vb*(t(i)-tb);
    vt(i) = vb;
    at(i) = 0;
 else 
    xt(i) = a0_up+a1_up*(t(i)-(tf-tb))+a2_up*(t(i)-(tf-tb))^2;
    vt(i) = a1_up + 2*a2_up*(t(i)-(tf -tb));
    at(i) = 2*a2_up;
 end
end
 
tt = t;
end