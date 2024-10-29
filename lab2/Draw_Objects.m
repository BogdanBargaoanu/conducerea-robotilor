% Start a new figure
figure
hold on

colours={'red','blue'};

% Plot the reference coordinate frame
plot3(0,0,0,'k.','MarkerSize',20);
plot3([0;5],[0;0],[0;0],'red','LineWidth',3);
plot3([0;0],[0;5],[0;0],'green','LineWidth',3);
plot3([0;0],[0;0],[0;5],'blue','LineWidth',3);

F={F1,F2};
V={V1(:,1:3),V2(:,1:3)};

% Draw the two objects
for ind = 1:length(F)
    if ~isempty(F{ind}) && ~isempty(V{ind})
        patch('Vertices', V{ind}, 'Faces', F{ind}, ...
            'FaceColor', colours{ind},...
            'EdgeColor', 'none',...
            'FaceLighting', 'gouraud');
    end
end

% Add lights on the figure to make the objects visible
light('Parent', gca, 'Position', [1 1 1],    'Style', 'infinite', 'Color', [1 1 1]);
light('Parent', gca, 'Position', [-1 -1 -1], 'Style', 'infinite', 'Color', [1 1 1]);

set(gca,'DataAspectRatio',[1 1 1]);