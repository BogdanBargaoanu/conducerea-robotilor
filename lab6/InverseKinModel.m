%run('C:\Users\Zsofi\Dropbox\TrueRehab\UR5_Sim\SM control\rvctools\rvctools\startup_rvc.m')
%% robot models
%Custom robot
%theta |d | a | alpha
% L1=10, L2=20, L3=5
L(1)=Link([0 10 0 0],'modified'); %modified DH method–the one from lecture/lab
L(2)=Link([pi/2 0 0 0 1],'modified'); % last parameter is to specify 
                             %revolute (0-default) or prismatic (1)
L(3)=Link([-pi/2 0 10 0],'modified');% se suprascrie valoare lui theta –trebuie adaugata 
%ca offset la q3 cu -pi/2
L(4)=Link([0 10 0 -pi/2 1],'modified');%se sprascrie valoare lui d –trebuie adaugata 
%ca offset la q4 cu 5
robarm1 = SerialLink(L, 'name', 'two link');
 
robarm1.offset(3)=-pi/2;
robarm1.offset(4)=5;
 
robarm1.plot([0 30 0.3 7],'workspace',[-50 50 -50 50 0 50]); 

%Puma robot
mdl_puma560

p560.plot([0.1 0.1 0.1 0.1 0.1 0.1])

%% Forward models
%Custom robot
robarm1.fkine([0 30 0.3 70]); % forward dynamic model for given input joint values

%Puma robot
figure
qz=[0, 0, 0, 0, 0, 0] %zero angle
qr=[0, pi/2, -pi/2, 0, 0, 0] %ready, the arm is straight and vertical
qs=[0, 0, -pi/2, 0, 0, 0] %stretch, the arm is straight and horizontal
qn=[0, pi/4, -pi, 0, pi/4, 0]% nominal, the arm is in a dextrous working pose
p560.plot(qz./2)
pause(1)
p560.plot(qz)
pause(1)
p560.plot(qr./2)
pause(1)
p560.plot(qr)
pause(1)
p560.plot(qs./2)
pause(1)
p560.plot(qs)
pause(1)
p560.plot(qn./2)
pause(1)
p560.plot(qn)
pause(1)

%% Inverse models
%Custom robot - numerical solution
Tr1=robarm1.fkine([0 30 0.3 70]);
%robarm1.ikine(Tr1, [1 .1 .1 .1], [1 1 1 0 0 0])
robarm1.ikine(Tr1,'q0', [1 .1 .1 .1], 'mask',[1 1 1 0 0 0]); 
% R.ikine(T, Q0, M, OPTIONS) similar to above but where M is a mask 
% vector (1x6) which specifies the Cartesian DOF (in the wrist coordinate 
% frame) that will be ignored in reaching a solution.  The mask vector 
% has six elements that correspond to translation in X, Y and Z, and rotation 
% about X, Y and Z respectively.  The value should be 0 (for ignore) or 1.
% The number of non-zero elements should equal the number of manipulator DOF.
%
% For example when using a 3 DOF manipulator rotation orientation might be 
% unimportant in which case  M = [1 1 1 0 0 0].
%
% For robots with 4 or 5 DOF this method is very difficult to use since
% orientation is specified by T in world coordinates and the achievable
% orientations are a function of the tool position.
% see Ch. 8.6 in Corke 2017


%Puma robot
q =[0 0.7854 3.1416 0 0.7854 0];
Tp1 = p560.fkine(q);
%numerical
qi = p560.ikine(Tp1) % different values but correct pose - check!
%analytical(only works for 6 DOF), see also ikine3s for 3DOF
qi = p560.ikine6s(Tp1) % different values but correct pose - check!

%% trajectories
t = [0:0.5:2]';
%Custom robot
Tr1=robarm1.fkine([0 3 0.03 7]);
Tr2=robarm1.fkine([0 30 1.3 70]);
%cartesian trajectory generation
Ts = ctraj(Tr1, Tr2, length(t)); % straight line motion
plot(t,transl(Ts)); % xyz - postion
plot(t, tr2rpy(Ts)); % euler angles - orientation
%joint trajectory generation
qc=jtraj([0 3 0.03 7],[0 30 1.3 70],t);% poly motion
robarm1.plot(qc,'workspace',[-50 50 -50 50 0 50]);

%Puma robot
q1 =[0 0.7854 3.1416 0 0.7854 0];
q2 =[20 1.5 30 1 -2 3];
Tp1 = p560.fkine(q1);
Tp2 = p560.fkine(q2);
Tp = ctraj(Tp1, Tp2, length(t));
qp = p560.ikine6s(Tp);
p560.plot(qp);
