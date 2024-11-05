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
plot3(px, py, pz)

q_vector = mgi_vectorial(px, py, pz);
q1_est = q_vector(1:length(q1));
q2_est = q_vector(length(q1)+1:2*length(q1));
q3_est = q_vector(2*length(q1)+1:end);

figure;
plot3(q1, q2, q3);
hold on
plot3(q1_est, q2_est, q3_est);

%% RRT
syms q1
syms q2
syms L1
 
T01 = rot(q1, 'z');
T12 = transl([0; 0; L1]) * rot(-q2, 'y');
T23 = transl([L2; 0; 0]);
T = T01 * T12 * T23; 
M{1} = T01;
M{2} = T12;
M{3} = T23;
%DrawRobot(M);

%% simulare
q1 = 0:0.1:2*pi;
q2 = ones(1,length(q1));
q3 = ones(1,length(q1));

p_vector = mgd_vectorial(q1, q2, q3);
px = p_vector(1:length(q1));
py = p_vector(length(q1)+1:2*length(q1));
pz = p_vector(2*length(q1)+1:end);
plot3(px, py, pz)

q_vector = mgi_vectorial(px, py, pz);
q1_est = q_vector(1:length(q1));
q2_est = q_vector(length(q1)+1:end);

figure;
plot3(q1, q2);
hold on
plot3(q1_est, q2_est);

%% RRTRT
syms q1
syms q2
syms q3
syms a1
syms a2
syms a3
syms a4
syms a5
 
T01 = transl([0; 0; a1]) * rot(q1, 'z');
T12 = transl([a2; 0; 0]) * transl([0; 0; a3]) * rot(q2, 'z');
T23 = transl([a4; 0; 0]) * transl([0; 0; -q3]);
T34 = transl([0; 0; a5]);
T = T01 * T12 * T23 * T34; 
M{1} = T01;
M{2} = T12;
M{3} = T23;
M{4} = T34;
%DrawRobot(M);


function p_vector = mgd_vectorial(q1, q2, q3)
L1 = 20;
L2 = 30;
%px = -sin(q1).*(q3 + L2);
px = L2 * cos(q1).*cos(q2);
%py = cos(q1).*(q3 + L2);
py = L2 * cos(q2).*sin(q1);
%pz = q2 + L1;
pz = L1 + L2*sin(q2);
%plot3(px,py,pz);
p_vector = [px py pz];
end

function q_vector = mgi_vectorial(px, py, pz)
L1 = 20;
L2 = 30;
%q1 = atan(-px./py); %in functie de cadran, se adauga pi, 2pi
q1 = py./px;
%q2 = pz - L1;
%q3 = -L2 + sqrt(px.^2 + py.^2);
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