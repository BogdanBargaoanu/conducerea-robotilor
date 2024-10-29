function DrawRobot(M)
%% Function for drawing the segments of a robot and their coordinate frames
% Input:        M          Cell array of size N. Each element represents a
%                               fundamendal transformation (Rotation or
%                               Translation). The coordinate frames are
%                               drawn after each transformation and
%                               segments are drawn between transformations

    P=eye(4);
    DrawCoord(P,2);
    for i=1:length(M)
        N=eye(4);
        for k=i:-1:1
            N=M{k}*N;
        end
        DrawCoord(N,1);
        DrawSegment(P,N);
        P=N;
    end
    
    % Set the ascpect ratio to 1 for each direction and set the camera at
    % point [1,1,1], looking towards point [0,0,0]
    set(gca,'DataAspectRatio',[1 1 1]);
    view(-235,45);
end

function obj = DrawSegment(M1,M2)
%% Function to draw a segment of a robot, based on the transformation matrices of its beginning and end points
    M=[M1(1:3,4)';M2(1:3,4)'];
    obj = plot3(M(:,1),M(:,2),M(:,3),'LineWidth',8);
end

function obj = DrawCoord(M,L)
%% Function to draw a coordinate frame in a 3D plot.
% Inputs: M     4x4 Homogenous transformation matrix
%              L       scalar indicating the length of the axes
%The colours of the coordinate frame are either black when l=2 or red,
%green and blue for the X, Y, and Z axes respectively
    
    %Obtain the direction and position of each of the axes of the
    %coordinate frame (CoX, CoY, CoZ) and the position of the
    %center of the frame
    CoX = M(1:3,4)+L*M(1:3,1);
    CoY = M(1:3,4)+L*M(1:3,2);
    CoZ = M(1:3,4)+L*M(1:3,3);
    CoO = M(1:3,4);

    axes(gca)
    hold on

    %Set the colours for the big (l=2) coordinate frame
    if L==2;
        colours={'k','k','k'};
    else
        colours={'r','g','b'};
    end
    % Plot origin in bone-plot
    obj{1} = plot3(CoO(1),CoO(2),CoO(3),'k.','MarkerSize',20);
    % Draw X-axis
    obj{2} = plot3([CoO(1);CoX(1)],[CoO(2);CoX(2)],...
        [CoO(3);CoX(3)],colours{1},'LineWidth',3);
    % Draw Y-axis
    obj{3} = plot3([CoO(1);CoY(1)],[CoO(2);CoY(2)],...
        [CoO(3);CoY(3)],colours{2},'LineWidth',3);
    % Draw Z-axis
    obj{4} = plot3([CoO(1);CoZ(1)],[CoO(2);CoZ(2)],...
        [CoO(3);CoZ(3)],colours{3},'LineWidth',3);
end