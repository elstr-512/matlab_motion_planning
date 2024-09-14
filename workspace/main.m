%% imports
addpath("workspace/");
addpath("utils/");
addpath("local_planner/");
addpath("global_planner/");

%% initialize

clear all;
clc;

% load environment
% load("gridmap_20x20_scene1.mat");
load("my_gridmap.mat");
map_size = size(grid_map);
G = 1;

% something about the angle ... to make them work ...
% start and goal pose
start = [4, 4, 0];
goal = [36, 46, 0];

% planner
global_planner_name = "a_star";
% global_planner_name = "dijkstra";
% global_planner_name = "voronoi_plan";

local_planner_name = "pid_plan";
% local_planner_name = "dwa_plan";


%% Path planning
global_planner = str2func(global_planner_name);
[path, global_flag, cost, expand] = global_planner(grid_map, start(:, 1:2), goal(:, 1:2));
% path = round(path);

%% trajectory planning
local_planner = str2func(local_planner_name);
[pose, traj, local_flag] = local_planner(start, goal, "path", path, "map", grid_map);

%% visualization
%clf;
%hold on

% plot grid map
%plot_grid(grid_map);
%plot_path(path, G);

% plot start and goal
%plot_square(start, map_size, G, "#f00");
%plot_square(goal, map_size, G, "#15c");

% animation
% record_video = true;
% animation(local_planner_name, pose, traj, G / 2, record_video);
%plot_path(pose, G);
%hold off

%% curvature
% Sample data: x and y coordinates of the path
x = flip(path(:, 2)); % colunm 2
y = flip(path(:, 1)); % column 1
% x = pose(:, 2); % colunm 2
% y = pose(:, 1); % column 1


% Parameter t can be the index or cumulative distance between points
t = linspace(0, 1, length(x)); % Evenly spaced parameter t

% Fit cubic splines for x(t) and y(t)
spline_x = pchip(t, x);
spline_y = pchip(t, y);

% Generate t values for evaluation
t_smooth = linspace(min(t), max(t), length(x));

% First derivatives of x(t) and y(t)
x_1st_derivative = ppval(fnder(spline_x, 1), t_smooth);
y_1st_derivative = ppval(fnder(spline_y, 1), t_smooth);

% Second derivatives of x(t) and y(t)
x_2nd_derivative = ppval(fnder(spline_x, 2), t_smooth);
y_2nd_derivative = ppval(fnder(spline_y, 2), t_smooth);

% Compute curvature using the parametric curvature formula
numerator = x_1st_derivative .* y_2nd_derivative - y_1st_derivative .* x_2nd_derivative;
denominator = (x_1st_derivative.^2 + y_1st_derivative.^2).^(3/2);


curvature = abs(numerator ./ denominator);

% Plot the original path
figure;
plot(x, y, 'o', 'DisplayName', 'Original Points');
hold on;
plot(ppval(spline_x, t_smooth), ppval(spline_y, t_smooth), '--', 'DisplayName', 'Interpolated Path');
xlabel('x');
ylabel('y');
legend;
title('Original Path and Spline Interpolation');
% hold off;

plot(0, 0, 's', 'DisplayName', 'Obstacle', ...
    'MarkerEdgeColor','black', 'MarkerFaceColor', 'black' ...
);

%plot points for obstacle
for i = 1:size(grid_map,1)
    for n = 1:size(grid_map,2)
        if grid_map(i,n) == 2
            plot( ...
                n, i, 's',... 
                'MarkerEdgeColor','black', 'MarkerFaceColor', 'black', ...
                'HandleVisibility', 'off'...
            );
        end
    end
end
hold off;

% Plot the absolute value of the curvature
figure;
hold on;
% plot(t_smooth, curvature, 'DisplayName', 'Curvature');
t_smooth_fitted = min(x) + t_smooth .* (max(x) - min(x));
curvature_fitted = (min(y) + curvature .* (max(y) - min(y)));
plot(t_smooth_fitted, curvature_fitted, 'DisplayName', 'Curvature');
title('Curvature of the Parametric Path');
%xlabel('t (parameter)');
%ylabel('|Curvature||');
xlabel('x');
ylabel('y');
legend;

curvature_fitted_mean = mean(curvature_fitted);
plot(0, curvature_fitted_mean, 'p', 'DisplayName', 'Curvature mean');


hold off;

