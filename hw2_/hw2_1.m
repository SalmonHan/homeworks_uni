clearvars
close all
clc

l1 = 5; 
l2 = 5; 

figure;
axis([-15 15 -15 15]);
hold on;
grid on;
title('2-Link Manipulator Animation');
xlabel('X');
ylabel('Y');

link1 = plot([0, 0], [0, 0], 'b-o', 'LineWidth', 2);
link2 = plot([0, 0], [0, 0], 'r-o', 'LineWidth', 2);


for t = 1:300

    theta_1 =  0.5 * t;  
    theta_2 = 1 * t;     

    T1 = [cosd(theta_1), -sind(theta_1), l1 * cosd(theta_1);
          sind(theta_1),  cosd(theta_1), l1 * sind(theta_1);
          0,              0,            1];

    T2 = [cosd(theta_2), -sind(theta_2), l2 * cosd(theta_2);
          sind(theta_2),  cosd(theta_2), l2 * sind(theta_2);
          0,              0,            1];

    p0 = [0; 0; 1];          
    p1 = T1 * p0;                 
    p2 = T1 * T2 * p0;       

    set(link1, 'XData', [p0(1), p1(1)], 'YData', [p0(2), p1(2)]);
    set(link2, 'XData', [p1(1), p2(1)], 'YData', [p1(2), p2(2)]);

    pause(0.05);
end