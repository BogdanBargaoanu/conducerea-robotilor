%% ex1 
v1 = [5; 2; 3];
T1 = transl(v1);
v2 = [0; -4; 0];
T2 = T1*transl(v2);
T3 = T2*rot(60, 'x');
v3 = [0; 0; 4];
T4 = T3*transl(v3);
T5 = T4*rot(90, 'y');
T_inv = transf_inv(T5);
 
%% ex 2
load('Objects.mat');
%V1 = V1';
Draw_Objects();
 
function mat_transl = transl(v)
     mat_transl = [eye(3) v];
     mat_transl = [mat_transl; [0 0 0 1]];
end
 
function mat_rotatie = rot(nr, axa)
nr = deg2rad(nr);
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