%direct geometric model DH lynx motion robot
clear all
clc
%theta |d | a | alpha
% L1=10, L2=20, L3=5
L0=0.06;
L1=0.03;
L2=0.15;
L3=0.185;
L4=0.05;
L5=0.05;

L(1)=Link([0 L0+L1 0 0],'modified'); %modified DH method–the one from lecture/lab
L(2)=Link([0 0 0 pi/2],'modified'); % last parameter is to specify 
                             %revolute (0-default) or prismatic (1)
L(3)=Link([0 0 L2 0],'modified');% se sprascrie valoare lui theta –trebuie adaugata 
%ca offset la q3 cu -pi/2
L(4)=Link([0 0 L3 0],'modified');%se sprascrie valoare lui d –trebuie adaugata 
%ca offset la q4 cu 5
L(5)=Link([0 L4+L5 0 pi/2],'modified');
robarm1 = SerialLink(L, 'name', 'lynx');
robarm1.offset(2)=+pi/2;
robarm1.offset(3)=-pi/2;
robarm1.offset(4)=pi/2;

q1=0;
q2=0;
q3=deg2rad(-30);
q4=deg2rad(-40);
q5=0;
qa=[q1 q2 q3 q4 q5]

q1=deg2rad(30);
q2=0;
q3=deg2rad(-30);
q4=deg2rad(-40);
q5=0;
qb=[q1 q2 q3 q4 q5]

q1=deg2rad(30);
q2=deg2rad(30);
q3=deg2rad(-30);
q4=deg2rad(-40);
q5=deg2rad(30);
qc=[q1 q2 q3 q4 q5]


%robarm1.plot(qa,'workspace',[-0.5 0.5 -0.5 0.5 0 0.5]);
%pause(1)
%robarm1.plot(qb,'workspace',[-0.5 0.5 -0.5 0.5 0 0.5]); 
%pause(2)
%robarm1.plot(qc,'workspace',[-0.5 0.5 -0.5 0.5 0 0.5]); 
%robarm1.plot([0 0 0 0 0],'workspace',[-0.5 0.5 -0.5 0.5 0 0.5]);
%inputs are values of joint variables
%Vizualize robot movements by generating different joint values
 
%Check out PUMA robot “mdl_puma560”!

robarm1.fkine(qa)
robarm1.fkine(qb)
robarm1.fkine(qc)

t=0:0.1:10;
%t=0;
omega=1;
xv=0.25*ones(1,length(t));
zv=(0.2+0.05)+0.05*cos(omega*t);
yv=0.1*sin(omega*t);
qv=zeros(5,length(t))';
for i=1:length(t)
Tr1=eye(4);
Tr1(1,4)=xv(i);
Tr1(2,4)=yv(i);
Tr1(3,4)=zv(i);
%robarm1.ikine(Tr1, [1 .1 .1 .1], [1 1 1 0 0 0])
qv(i,:)=robarm1.ikine(Tr1,'q0', [1 .1 .1 .3 .3], 'mask',[1 1 1 0 0 0]);
end
%robarm1.plot(qv,'workspace',[-0.5 0.5 -0.5 0.5 0 0.5]); 

%% Experiment fizic

qvd = rad2deg(qv);

for i =1:length(qvd)
    %qvd(i,:)
    a = moveRobot(qvd(i,:))
    pause(0.5);
end


