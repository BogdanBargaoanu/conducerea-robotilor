% Domsa Victor 30.09.2022
% Contact: domsavictor@gmail.com
% Tested for Matlab 2018 and up
% Matlab 2022a gives warnings about the 'serial' command. It may not work
% in future releases of Matlab, use 'serialport' instead


classdef SliderMove < handle
    %SLIDERMOVE Summary of this class goes here
    %   Class to control the robot arm 
    
    properties
        port;
        firstRun = 1;
        baudRate = 9600;

        % define sliders
        fig;
        grid;
        lbl1;
        lbl2;
        lbl3;
        lbl4;
        lbl5;
        lbl6;
        sld1;
        sld2;
        sld3;
        sld4;
        sld5;
        sld6;
    end
    
    methods
        function obj = SliderMove(port)
            %SLIDERMOVE Construct an instance of this class
            % Create the UI and initialize the port the robot is connected
            % to. Get port name with 'seriallist' command.

            if nargin > 0
                obj.port = port;
                
                obj.fig = uifigure;
                obj.grid = uigridlayout(obj.fig);

                obj.lbl1 = uilabel(obj.grid, "Text", 'q1');
                obj.sld1 = uislider(obj.grid,"Limits",[0, 180.001], ...
                    'ValueChangedFcn', @(sld, event) obj.sliderChangedValue(sld),...
                    'Value', 90, 'UserData', 1);

                obj.lbl2 = uilabel(obj.grid, "Text", 'q2');
                obj.sld2 = uislider(obj.grid,"Limits",[60, 120], ...
                    'ValueChangedFcn', @(sld, event) obj.sliderChangedValue(sld),...
                    'Value', 90, 'UserData',2);

                obj.lbl3 = uilabel(obj.grid, "Text", 'q3');
                obj.sld3 = uislider(obj.grid,"Limits",[60, 130], ...
                    'ValueChangedFcn', @(sld, event) obj.sliderChangedValue(sld),...
                    'Value', 90, 'UserData',3);

                obj.lbl4 = uilabel(obj.grid, "Text", 'q4');
                obj.sld4 = uislider(obj.grid,"Limits",[0, 180], ...
                    'ValueChangedFcn', @(sld, event) obj.sliderChangedValue(sld),...
                    'Value', 90, 'UserData',4);

                obj.lbl5 = uilabel(obj.grid, "Text", 'q5');
                obj.sld5 = uislider(obj.grid,"Limits",[30, 110], ...
                    'ValueChangedFcn', @(sld, event) obj.sliderChangedValue(sld),...
                    'Value', 90, 'UserData',5);

                obj.lbl6 = uilabel(obj.grid, "Text", 'q6');
                obj.sld6 = uislider(obj.grid,"Limits",[0, 180], ...
                    'ValueChangedFcn', @(sld, event) obj.sliderChangedValue(sld),...
                    'Value', 90, 'UserData', 6);
            end
        end
        
        function obj = sliderChangedValue(obj, sld)
            %sliderChangedValue 
            %   If run the first time it will first initialize all the 
            % joints. Then it will read the value from the slider, make 
            % the command string and send it to the robot.
            
            if (obj.firstRun == 1)                
                % Only run this the first time a slider is moved to place
                % the robot in the initial position.
                
                obj.firstRun = 0;
                
                initComm = "#0P1500S150#1P1500S150#2P1500S150#3P1500S150#4P1500S150#5P1500S150\r";
                initComm = sprintf(initComm);
                
                % Create serial object
                s = serial(obj.port, 'BaudRate', obj.baudRate);
            
                fopen(s);
                % Send the command to the robot
                fprintf(s, initComm);
                fclose(s);     
            end
            
            % read the slider/joint number and value
            jointNumber = sld.UserData - 1;
            jointAngle = sld.Value;

            % convert the angle from the slidert to the PWM value required
            % by the control board
            jointCommandValue = string(round(jointAngle * 11.1111 + 500));
            
            % create the command string according to the documentation
            command = "#";
            command = strcat(command, string(jointNumber));
            command = strcat(command, "P");
            command = strcat(command, jointCommandValue);
            command = strcat(command, "S150\r");
            command = sprintf(command);

            % Create serial object
            s = serial(obj.port, 'BaudRate', obj.baudRate);
            
            fopen(s);
            % Send the command to the robot
            fprintf(s, command);
            fclose(s);        
        end
    end
end

