function T=mgd(qq)
q1=qq(1);%deg2rad(-38);
q2=qq(2);%deg2rad(6);
q3=qq(3);%deg2rad(14);
q4=qq(4);%deg2rad(-68);
q5=qq(5);%deg2rad(1);


q=[q1,q2,q3,q4,q5]


a = [0, 0.002, 0.14533, 0.18233, 0]
d = [-0.06858, 0, -0.00077, 0.00645, -0.009699]

alpha = [pi, pi / 2, pi, pi, -pi / 2]

offset = [pi, -pi/2 - 0.0059, pi/2 - 0.0378, pi / 2 - 0.0319, pi]

for i=1:5

 L(i)=Link([q(i) d(i) a(i) alpha(i) 0],'modified')    
    
end

robot=SerialLink(L,'name','trei');

for i=1:5

robot.offset(i)=offset(i)
end

robot.plot([q1 q2 q3 q4 q5],'workspace',[-0.5 0.5 -0.5 0.5 0 0.5])
robot.fkine([q1 q2 q3 q4 q5])
end