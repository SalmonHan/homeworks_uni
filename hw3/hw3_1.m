clearvars;
close all;
clc;

% 첫 과제의 경우 configuration space가 2차원 xy
x_0 = 0;
y_0 = 0;
speed = 0.05;

x_goal = 5.0;
y_goal = 0.0;

x_obstacle = 3.0;
y_obstacle = 0.0;
eta = 3;


x = x_0;
y = y_0;

figure;
hold on;
plot(x_goal, y_goal, 'ro', 'MarkerSize', 10, 'DisplayName', 'Goal');
plot(x_obstacle, y_obstacle, 'go', 'MarkerSize', 10, 'DisplayName', 'obstacle');
h = plot(x, y, 'bo', 'MarkerSize', 5, 'DisplayName', 'Moving Point');

xlim([-1, 6]);  % X축 범위 설정
ylim([-1, 6]);  % Y축 범위 설정


for t = 1:1000
    %goal grad
    dd_dx = 0;
    dd_dy = 0;
    if (x-x_goal)^2 + (y-y_goal)^2 < 3
        dd_dx = dd_dx + x - x_goal;
    else
        if x-x_goal == 0
            dd_dx = dd_dx;
        elseif abs(x-x_goal) > x-x_goal
            dd_dx = dd_dx - 1;
        elseif abs(x-x_goal) == x-x_goal
            dd_dx = dd_dx + 1;
        end
    end

    if (x-x_goal)^2 + (y-y_goal)^2 < 3
        dd_dy = dd_dy + y - y_goal;
    else
        if y-y_goal == 0
            dd_dy = dd_dy;
        elseif abs(y-y_goal) > y-y_goal
            dd_dy = dd_dy - 1;
        elseif abs(y-y_goal) == y-y_goal
            dd_dy = dd_dy + 1;
        end
    end
    
    %obstacle grad
    if (x-x_obstacle)^2 + (y-y_obstacle)^2 < 0.5
        dd_dx = dd_dx - eta * (x - x_obstacle);
    else
        dd_dx = dd_dx + 0;
    end

    if (x-x_obstacle)^2 + (y-y_obstacle)^2 < 0.5
        dd_dy = dd_dy - eta * (y - y_obstacle);
    else
        dd_dy = dd_dy + 0;
    end


    x = x - speed * dd_dx;
    y = y - speed * dd_dy;
    
    set(h, 'XData', x, 'YData', y);
    
    pause(0.05);
end

hold off;
