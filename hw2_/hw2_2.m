clearvars;
close all;
clc;

l1 = 0.128;
l2 = 0.124;
l3 = 0.126;




figure;
axis([-0.5 0.5 -0.5 0.5 -0.5 0.5]);
hold on;
grid on;
title('Manipulator-X');
xlabel('X');
ylabel('Y');
zlabel('Z');
view(3);

link1 = plot3([0, 0], [0, 0], [0, 0], 'b-o', 'LineWidth', 2);
link2 = plot3([0, 0], [0, 0], [0, 0], 'r-o', 'LineWidth', 2);
link3 = plot3([0, 0], [0, 0], [0, 0], 'g-o', 'LineWidth', 2);

% z축 회전(theta), x축 회전(alpha)만
function T = rotate(theta, d, alpha, a)
    % theta: 조인트 각도 (Z축 회전 각도)
    % d: Z축 이동 거리
    % alpha: X축 회전 각도 (링크 간의 회전)
    % a: X축 이동 거리 (링크 길이)
    % theta z축으로 돌리고 d만큼 이동, 거기 기준으로 x축으로 alpha만큼 회전, a만큼 이동 곱   
    T = [cosd(theta),  -sind(theta) * cosd(alpha),    sind(theta)*sind(alpha),      a * cosd(theta);
          sind(theta),  cosd(theta) * cosd(alpha),    -cosd(theta) * sind(alpha),   a*sind(theta);
          0,            sind(alpha),                  cosd(alpha),                  d;
          0,            0,                            0,                            1;]

end


% 애니메이션 루프
for t = 1:360
    theta_1 = -90 * sind(t); 
    theta_2 = 60 + 45 * sind(t) ; 
    theta_3 = -30 - 90 * sind(t); 


    
    % theta, d, alpha, a
    T1 = rotate(theta_1, l1,90,0); 
    T2 = rotate(theta_2, 0,0,l2);  
    T3 = rotate(theta_3, 0,0,l3); 

    p0 = [0; 0; 0; 1];        
    p1 = T1 * p0;   
    p2 = T1 * T2 * p0;  
    p3 = T1 * T2 * T3 * p0;  
    
    set(link1, 'XData', [p0(1), p1(1)], 'YData', [p0(2), p1(2)], 'ZData', [p0(3), p1(3)]);
    set(link2, 'XData', [p1(1), p2(1)], 'YData', [p1(2), p2(2)], 'ZData', [p1(3), p2(3)]);
    set(link3, 'XData', [p2(1), p3(1)], 'YData', [p2(2), p3(2)], 'ZData', [p2(3), p3(3)]);

    
    pause(0.05);
end
