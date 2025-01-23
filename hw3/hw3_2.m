clearvars;
close all;
clc;

% 두번째 과제는 x,y 좌표가 있고, 회전각 theta가 존재.
% 4개의 point를 kinematic으로 계산할건데 x_goal, y_goal로 평행이동, theta_goal로 회전
function T = move(theta, x, y)
    T = [cosd(theta), -sind(theta), x;
         sind(theta),  cosd(theta), y;
         0,            0,           1];
end

car_width = 0.5;
car_height = 1.0;

x_0 = 0;
y_0 = 0;
theta_0 = 0;
speed = 0.05;

x_goal = 8.0;
y_goal = 6.0;
theta_goal = 90;

x = x_0;
y = y_0;
theta = theta_0;




figure;
hold on;
plot(x_goal, y_goal, 'ro', 'MarkerSize', 10, 'DisplayName', 'Goal');
h0 = plot(x, y, 'bo', 'MarkerSize', 5, 'DisplayName', 'Moving Point');
h1 = plot(x, y, 'bo', 'MarkerSize', 5, 'DisplayName', 'Moving Point');
h2 = plot(x, y, 'bo', 'MarkerSize', 5, 'DisplayName', 'Moving Point');
h3 = plot(x, y, 'bo', 'MarkerSize', 5, 'DisplayName', 'Moving Point');

xlim([-1, 10]);
ylim([-1, 10]);


for t = 1:1000
    %goal grad
    dd_dx = 0;
    dd_dy = 0;
    dd_dtheta = 0;
    if (x-x_goal)^2 + (y-y_goal)^2 < 1
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

    if (x-x_goal)^2 + (y-y_goal)^2  < 1
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

    dd_dtheta = theta - theta_goal;

    
    x = x - speed * dd_dx;
    y = y - speed * dd_dy;
    theta = theta - speed * dd_dtheta;

    T1 = move(theta, x, y);
    p0 = [0; 0; 1];
    p1 = [car_width; 0; 1];
    p2 = [car_width; car_height; 1];
    p3 = [0; car_height; 1];
    

    p0_moved = T1 * p0;
    p1_moved = T1 * p1;
    p2_moved = T1 * p2;
    p3_moved = T1 * p3;
    
    set(h0, 'XData', p0_moved(1), 'YData', p0_moved(2));
    set(h1, 'XData', p1_moved(1), 'YData', p1_moved(2));
    set(h2, 'XData', p2_moved(1), 'YData', p2_moved(2));
    set(h3, 'XData', p3_moved(1), 'YData', p3_moved(2));
    
    pause(0.05);
end

hold off;
