clearvars;
close all;
clc;

%대나무 헬리콥터

figure;
axis([-10 20 -10 10 -10 10]);
hold on;
grid on;
title('doraemon');
xlabel('X');
ylabel('Y');
zlabel('Z');
view(3);

function T = move(dist)
    % Z축 방향으로 dist만큼 이동
    T2 = [1 0 0 0;
          0 1 0 0;
          0 0 1 dist;
          0 0 0 1];
    
    % 나아가는 방향 정의 (새로운 Z축을 정함)
    T3 = [0 0 1 0;
          0 1 0 0;
          1 0 0 0;
          0 0 0 1];

    T =  T3 * T2;
end

function T = rotate(theta, d, alpha, a)
    T = [cosd(theta),  -sind(theta) * cosd(alpha),    sind(theta)*sind(alpha),      a * cosd(theta);
         sind(theta),  cosd(theta) * cosd(alpha),    -cosd(theta) * sind(alpha),   a*sind(theta);
         0,            sind(alpha),                  cosd(alpha),                  d;
         0,            0,                            0,                            1];
end


link1 = plot3([0, 0], [0, 0], [0, 0], 'r-', 'LineWidth', 2);
link2 = plot3([0, 0], [0, 0], [0, 0], 'r-', 'LineWidth', 2);
link3 = plot3([0, 0], [0, 0], [0, 0], 'r-', 'LineWidth', 2);

l1 = 3;  % 날개의 길이


% 애니메이션 루프
for t = 1:360
    theta_1 = 10 * t;
    theta_2 = theta_1 + 120;
    theta_3 = theta_1 + 240;

    dist = 0.1 * t;  % Z축 이동 거리


    T1 = move(dist) * rotate(theta_1, 0, 0, l1);
    T2 = move(dist) * rotate(theta_2, 0, 0, l1);
    T3 = move(dist) * rotate(theta_3, 0, 0, l1);

    p0 = [0; 0; 0; 1];
    p0_moved = move(dist) * p0;

    p1 = T1 * p0;
    p2 = T2 * p0;
    p3 = T3 * p0;

    set(link1, 'XData', [p0_moved(1), p1(1)], 'YData', [p0_moved(2), p1(2)], 'ZData', [p0_moved(3), p1(3)]);
    set(link2, 'XData', [p0_moved(1), p2(1)], 'YData', [p0_moved(2), p2(2)], 'ZData', [p0_moved(3), p2(3)]);
    set(link3, 'XData', [p0_moved(1), p3(1)], 'YData', [p0_moved(2), p3(2)], 'ZData', [p0_moved(3), p3(3)]);

    pause(0.05);
end
