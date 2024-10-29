%% RTT
q1 = 2;
q2 = 3;
q3 = 5;
L1 = 1;
 
T01 = rot(q1, 'z');
T12 = transl([0; 0; q2]);
T23 = transl([0; q3; 0]);
T34 = transl([0; L1; 0]);
T = T01 * T12 * T23 * T34; 
M{1} = T01;
M{2} = T12;
M{3} = T23;
M{4} = T34;
 
DrawRobot(M);
%% RRRT
q1 = 10;
L1 = 5;
L2 = 3;
q2 = 30;
L3 = 2;
q3 = 15;
L4 = 2;
q4 = 10;
L5 = 5;
 
T01 = rot(q1, 'z');
T12 = transl([0; 0; L1]) * transl([L2; 0; 0]) * rot(q2, 'x'); %transl([0; 0; L1]) * transl([L2; 0; 0]) = transl([L2; 0; L1]);
T23 = transl([0; 0; L3]) * rot(-q3, 'x');
T34 = transl([-L2; L4; q4]); % acelasi lucru cu transl() de fiecare axa inmultite
T45 = transl([0; L5; 0]);
T = T01 * T12 * T23 * T34 * T45; 
M{1} = T01;
M{2} = T12;
M{3} = T23;
M{4} = T34;
M{5} = T45;
DrawRobot(M);
 
 
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