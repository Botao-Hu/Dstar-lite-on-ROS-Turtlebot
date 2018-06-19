% run this .m file to generate video
M = load('map.txt');
trajectory = load('path.txt');
file = fopen('updated_points.txt');
g = textscan(file,'%s','delimiter','\n');
fclose(file);
M_change = cell(size(trajectory,1),1);
for i = 1:size(trajectory, 1)
    M_change{i,1} = str2num(g{1,1}{i,1});
end
x_init = 2; y_init = 2;
x_goal = 100; y_goal = 100;
range = 3;
Display_lite_full(M, M_change, x_init, y_init, x_goal, y_goal, trajectory, range);