clearvars;
close all;
clc;

% 포텐셜을 이용하여 공전 구현
x_0 = 0;
y_0 = 3;
speed = 0.005;

x_goal = 0.0;
y_goal = 0.0;

x = x_0;
y = y_0;

figure;
hold on;
plot(x_goal, y_goal, 'ro', 'MarkerSize', 10, 'DisplayName', 'Goal');

h = plot(x, y, 'bo', 'MarkerSize', 5, 'DisplayName', 'Moving Point');

xlim([-4, 4]);  % X축 범위 설정
ylim([-4, 4]);  % Y축 범위 설정

%초기 dd_dx를 어떻게 주느냐에 따라 다양한 공전 구현 가능
dd_dx = 45;
dd_dy = 0;
for t = 1:1000
    %goal grad
    if (x-x_goal)^2 + (y-y_goal)^2 < 10
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

    if (x-x_goal)^2 + (y-y_goal)^2 < 10
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
    


    x = x - speed * dd_dx;
    y = y - speed * dd_dy;
    
    set(h, 'XData', x, 'YData', y);
    
    pause(0.05);
end

hold off;
