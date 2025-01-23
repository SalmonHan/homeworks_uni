% 파일에서 경로 데이터 불러오기
fileID = fopen('path_output_4_3.txt', 'r');

% 첫 번째 줄과 헤더를 건너뜀
fgetl(fileID); % 첫 번째 줄 (Path Data:)
fgetl(fileID); % 두 번째 줄 (Index x y z psi theta phi)

% 데이터 읽기
path_data = fscanf(fileID, '%d %f %f %f %f %f %f', [7, Inf]);
fclose(fileID);

% 데이터 전치 (MATLAB의 fscanf는 열 우선으로 데이터를 읽기 때문에)
path_data = path_data';

% 경로에서 각 지점 추출
path_points = path_data(:, 2:7); % x, y, z, psi, theta, phi 데이터

function T = move(dx, dy, dz, psi, theta, phi) 
    % ψ (psi): z축에 대한 회전각 (Yaw)
    % θ (theta): y축에 대한 회전각 (Pitch)
    % φ (phi): x축에 대한 회전각 (Roll)
    T = [cosd(psi) * cosd(theta),   cosd(psi) * sind(theta) * sind(phi) - sind(psi) * cosd(phi),   cosd(psi) * sind(theta) * cosd(phi) + sind(psi) * sind(phi),   dx;
         sind(psi) * cosd(theta),   sind(psi) * sind(theta) * sind(phi) + cosd(psi) * cosd(phi),   sind(psi) * sind(theta) * cosd(phi) - cosd(psi) * sind(phi),   dy;
         -sind(theta),              cosd(theta) * sind(phi),                                       cosd(theta) * cosd(phi),                                       dz;
         0,                         0,                                                             0,                                                             1];
end

% 막대랑 벽 collision box
rod = collisionBox(0.5, 0.5, 2);
wall_0 = collisionBox(5, 0.3, 5);
wall_1 = collisionBox(4, 0.3, 4);
wall_2 = collisionBox(4, 0.3, 4);




%rod.Pose = [rodRotation, rodPosition'; 0 0 0 1];
rod.Pose = move(0,0,0,0,0,0);

% 벽 위치 설정
wallPosition_0 = [-2.5, 1, 2.0]; % 벽의 위치 (x, y, z 좌표)
wallPosition_1 = [0.5 -1.5, -3.0];
wallPosition_2 = [3.2, 0, -1];


wall_0.Pose = trvec2tform(wallPosition_0);
wall_1.Pose = trvec2tform(wallPosition_1);
wall_2.Pose = trvec2tform(wallPosition_2);

walls = {wall_0,wall_1,wall_2};

% 박스의 크기 정의
width = 0.5;  % x 방향 크기
depth = 0.5;  % y 방향 크기
height = 2.0; % z 방향 크기

% 각 꼭지점의 상대 좌표 계산
vertices = [
    -width/2, -depth/2, -height/2, 1;  % 꼭지점 1
     width/2, -depth/2, -height/2, 1;  % 꼭지점 2
     width/2,  depth/2, -height/2, 1;  % 꼭지점 3
    -width/2,  depth/2, -height/2, 1;  % 꼭지점 4
    -width/2, -depth/2,  height/2, 1;  % 꼭지점 5
     width/2, -depth/2,  height/2, 1;  % 꼭지점 6
     width/2,  depth/2,  height/2, 1;  % 꼭지점 7
    -width/2,  depth/2,  height/2, 1;  % 꼭지점 8
];

% 각 면을 정의하는 Faces 배열
faces = [
    1, 2, 3, 4; % 아래 면
    5, 6, 7, 8; % 위 면
    1, 2, 6, 5; % 옆면 1
    2, 3, 7, 6; % 옆면 2
    3, 4, 8, 7; % 옆면 3
    4, 1, 5, 8  % 옆면 4
];


% 시각화
figure;
hold on;

start = [1,-3,-2,0,0,0];
goal = [-2,3,-1,55,95,120];

show(wall_0, 'Parent', gca);
show(wall_1, 'Parent', gca);
show(wall_2, 'Parent', gca);

rod.Pose = move(start(1),start(2),start(3),start(4),start(5),start(6));
show(rod, 'Parent', gca)
rod.Pose = move(goal(1),goal(2),goal(3),goal(4),goal(5),goal(6));
show(rod, 'Parent', gca)


% 각 지점을 연결하여 시각화
for i = 1:size(path_points, 1) - 1
    point1 = path_points(i, :);
    point2 = path_points(i + 1, :);
    
    % 8등분하여 중간 지점들을 계산하고 시각화
    for j = 0:1
        intermediate_point = (1 - j/2) * point1 + (j/2) * point2
        T_intermediate = move(intermediate_point(1), intermediate_point(2), intermediate_point(3), ...
                              intermediate_point(4), intermediate_point(5), intermediate_point(6));
        
        % 동차 좌표 계산
        now_rod = T_intermediate * vertices'; % (4,4) * (4,N)
        now_rod = now_rod(1:3, :)'; % Nx3 형식으로 전환
        
        % rod 시각화
        patch('Vertices', now_rod, 'Faces', faces, ...
              'FaceColor', 'r', 'FaceAlpha', 0.5, 'EdgeColor', 'k');
        
        pause(0.1);
    end
end

% 그래프 설정
xlabel('X');
ylabel('Y');
zlabel('Z');
grid on;
axis equal;
title('Replaying Path from Loaded Data');
hold off;