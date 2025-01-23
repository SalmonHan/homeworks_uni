clearvars;
close all;
clc;

% 세번째 과제는 xyz 좌표에 로우 피치 요 3가지 회전각
% 3개의 point를 kinematic으로 계산할건데 x_goal, y_goal, z_goal로 평행이동, psi theta phi로 회전
function T = move(psi, theta, phi, dx, dy, dz) 
    % ψ (psi): z축에 대한 회전각 (Yaw)
    % θ (theta): y축에 대한 회전각 (Pitch)
    % φ (phi): x축에 대한 회전각 (Roll)
    T = [cosd(psi) * cosd(theta),   cosd(psi) * sind(theta) * sind(phi) - sind(psi) * cosd(phi),   cosd(psi) * sind(theta) * cosd(phi) + sind(psi) * sind(phi),   dx;
         sind(psi) * cosd(theta),   sind(psi) * sind(theta) * sind(phi) + cosd(psi) * cosd(phi),   sind(psi) * sind(theta) * cosd(phi) - cosd(psi) * sind(phi),   dy;
         -sind(theta),              cosd(theta) * sind(phi),                                       cosd(theta) * cosd(phi),                                       dz;
         0,                         0,                                                             0,                                                             1];
end

width = 0.5;
height = 1.0;

x_0 = 0;
y_0 = 0;
z_0 = 0;
psi_0 = 0;
theta_0 = 0;
phi_0 = 0;

speed = 0.05;

x_goal = 8.0;
y_goal = 6.0;
z_goal = 8.0;
psi_goal = 45;
theta_goal = 30;
phi_goal = 135;

x = x_0;
y = y_0;
z = z_0;
psi = psi_0;
theta = theta_0;
phi = phi_0;

figure;
axis([-10 20 -10 10 -10 10]);
hold on;
grid on;
title('Manipulator-X');
xlabel('X');
ylabel('Y');
zlabel('Z');
view(3);

hold on;
plot3(x_goal, y_goal, z_goal, 'ro', 'MarkerSize', 10, 'DisplayName', 'Goal');

p0 = [-1/2 * width; -1/2 * height; 0];
p1 = [1/2 * width; -1/2 * height; 0];
p2 = [0; 1/2 * height; 0];

h = fill3([p0(1), p1(1), p2(1)], [p0(2), p1(2), p2(2)], [p0(3), p1(3), p2(3)], 'b');

xlim([-1, 10]);
ylim([-1, 10]);
zlim([-1, 15]);

for t = 1:1000
    % Goal Gradient 계산
    dd_dx = 0;
    dd_dy = 0;
    dd_dz = 0;
    dd_dpsi = 0; 
    dd_dtheta = 0; 
    dd_dphi = 0; 

    if (x - x_goal)^2 + (y - y_goal)^2 + (z - z_goal)^2 < 1
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

    if (x - x_goal)^2 + (y - y_goal)^2 + (z - z_goal)^2 < 1
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

    if (x - x_goal)^2 + (y - y_goal)^2 + (z - z_goal)^2 < 1
        dd_dz = dd_dz + z - z_goal;
    else
        if z-z_goal == 0
            dd_dz = dd_dz;
        elseif abs(z-z_goal) > z-z_goal
            dd_dz = dd_dz - 1;
        elseif abs(y-y_goal) == z-z_goal
            dd_dz = dd_dz + 1;
        end
    end

    dd_dpsi = psi - psi_goal;
    dd_dtheta = theta - theta_goal;
    dd_dphi = phi - phi_goal;


    x = x - speed * dd_dx;
    y = y - speed * dd_dy;
    z = z - speed * dd_dz;
    psi = psi - speed * dd_dpsi;
    theta = theta - speed * dd_dtheta;
    phi = phi - speed * dd_dphi;


    T1 = move(psi, theta, phi, x, y, z);

    p0_moved = T1 * [p0; 1];
    p1_moved = T1 * [p1; 1];
    p2_moved = T1 * [p2; 1];


    set(h, 'XData', [p0_moved(1), p1_moved(1), p2_moved(1)], 'YData', [p0_moved(2), p1_moved(2), p2_moved(2)], 'ZData', [p0_moved(3), p1_moved(3), p2_moved(3)]);

    pause(0.05);
end

hold off;
