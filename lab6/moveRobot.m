function robot_command = moveRobot(q, com)


L0=0.06;
L1=0.03;
L2=0.15;
L3=0.185;
L4=0.05;
L5=0.05;


L(1)=Link([0 L0+L1 0 0],'modified'); %modified DH methodthe one from lecture/lab
L(2)=Link([0 0 0 pi/2],'modified'); % last parameter is to specify 
                             %revolute (0-default) or prismatic (1)
L(3)=Link([0 0 L2 0],'modified');% se sprascrie valoare lui theta trebuie adaugata 
%ca offset la q3 cu -pi/2
L(4)=Link([0 0 L3 0],'modified');%se sprascrie valoare lui d trebuie adaugata 
%ca offset la q4 cu 5
L(5)=Link([0 L4+L5 0 pi/2],'modified');

robarm1 = SerialLink(L, 'name', 'lynx');

robarm1.offset(2)=+pi/2;
robarm1.offset(3)=-pi/2;
robarm1.offset(4)=+pi/2;


expected_position = robarm1.fkine(deg2rad(q))

q(1) = -q(1);
q(3) = -q(3);

q = q + 90;

disp(q)

if expected_position.t(3)<0.02
    disp("COLLISSION WITH THE TABLE");
    
    return;
end

if nargin == 1
    aux = seriallist;
    if length(aux) < 2
        disp("Port not found!");
            return;
    end
    com = aux(2);
end

command_value = zeros(1, length(q));


for i = 1:length(q)
    command_value(i) = string(round(q(i) * 11.1111 + 500));
end

robot_command = "#";
for i = 1:length(command_value)
    robot_command = strcat(robot_command, string(i-1));
    robot_command = strcat(robot_command, "P");
    robot_command = strcat(robot_command, string(command_value(i)));
    robot_command = strcat(robot_command, "S150#");
end

robot_command = strcat(robot_command, "\r");
robot_command = sprintf(robot_command);

disp(robot_command);


s = serial(com, 'BaudRate', 9600);
fopen(s);
fprintf(s, robot_command);
fclose(s);

end

