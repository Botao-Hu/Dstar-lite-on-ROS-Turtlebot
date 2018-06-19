function [] = Display_lite(M, M_change, x_init, y_init, x_goal, y_goal, trajectory, range)

% initialize video object
vidObj = VideoWriter('Trajectory2.avi');
vidObj.FrameRate = 20;
open(vidObj);

% draw bottom layer
scatter(10*(x_init - 0.5), 10*(y_init - 0.5), 500, 'ko', 'linewidth', 2); hold on;
scatter(10*(x_goal - 0.5), 10*(y_goal - 0.5), 500, 'kh', 'linewidth', 2); hold on;
axis([-10 10*size(M, 1)+10 -10 10*size(M, 2)+10]);
set(gca,'xtick',[])
set(gca,'xticklabel',[])
set(gca,'ytick',[])
set(gca,'yticklabel',[])
title('sensed\_map', 'Fontsize', 20);
set(gcf, 'Position', [0, 0, 1000, 1000]);

% draw global_map
for i = 1:size(M, 1)
    for j = 1:size(M, 2)
        if M(i, j) == 1
            rectangle('Position', [10*(i - 1), 10*(j - 1), 10, 10], 'FaceColor', 'b'); hold on;
        %else
        %    rectangle('Position', [i - 1, j - 1, 1, 1]); hold on;
        end
    end
end

% draw sensed_map with updated points
for i = 1:size(trajectory, 1)
    arr = M_change{i,1};
    if size(arr, 1) > 0
        for j = 1:2:size(arr,2)-1
            rectangle('Position', [10*arr(j), 10*arr(j+1), 10, 10], 'FaceColor', 'b'); hold on;
        end
    end
        
    if i > 1
        x = [10*(trajectory(i - 1, 1) - 0.5), 10*(trajectory(i, 1) - 0.5)];
        y = [10*(trajectory(i - 1, 2) - 0.5), 10*(trajectory(i, 2) - 0.5)];
        plot(x, y, 'r', 'linewidth', 3); hold on;
    end
    
    currFrame = getframe;
    writeVideo(vidObj, currFrame);
end
   
close(vidObj);

end