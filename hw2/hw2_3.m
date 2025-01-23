clearvars;
close all;
clc;


figure;
axis([-10 20 -10 10 -10 10]);
hold on;
grid on;
title('Manipulator-X');
xlabel('X');
ylabel('Y');
zlabel('Z');
view(3);

link1 = plot3([0, 0], [0, 0], [0, 0], 'r-', 'LineWidth', 2);
link2 = plot3([0, 0], [0, 0], [0, 0], 'r-', 'LineWidth', 2);
link3 = plot3([0, 0], [0, 0], [0, 0], 'r-', 'LineWidth', 2);

% 
function T = move(theta, dist)
    % z축 방향으로 회전
    T1 = [cosd(theta) -sind(theta) 0 0;
          sind(theta) cosd(theta) 0 0;
          0 0 1 0;
          0 0 0 1];
    
    % z축 방향으로 이동
    T2 = [1 0 0 0;
          0 1 0 0;
          0 0 1 dist;
          0 0 0 1];
    
    %나아가는 방향 정의(새로운 z축)
    T3 = [0 0 1 0;
        0 1 0 0;
        1 0 0 0;
        0 0 0 1;]

    T =  T3 * T2 * T1;
end



% 애니메이션 루프
for t = 1:360
    theta = 20* t; 
    dist = 0.1 * t;


    
    % theta, d, alpha, a
    T = move(theta, dist); 

    p_0_0 = [0; 0; 1; 1];        
    p_0_1 = [-0.25; 0; 0; 1]; 
    p_0_2 = [0.25; 0; 0; 1]; 
    p1 = T * p_0_0;   
    p2 = T * p_0_1;  
    p3 = T * p_0_2;  
    
    set(link1, 'XData', [p1(1), p2(1)], 'YData', [p1(2), p2(2)], 'ZData', [p1(3), p2(3)]);
    set(link2, 'XData', [p1(1), p3(1)], 'YData', [p1(2), p3(2)], 'ZData', [p1(3), p3(3)]);
    set(link3, 'XData', [p2(1), p3(1)], 'YData', [p2(2), p3(2)], 'ZData', [p2(3), p3(3)]);

    
    pause(0.05);
end
