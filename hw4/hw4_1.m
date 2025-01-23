clearvars;
close all;
clc;

function T = move(dx, dy, dz, psi, theta, phi) 
    % ψ (psi): z축에 대한 회전각 (Yaw)
    % θ (theta): y축에 대한 회전각 (Pitch)
    % φ (phi): x축에 대한 회전각 (Roll)
    T = [cosd(psi) * cosd(theta),   cosd(psi) * sind(theta) * sind(phi) - sind(psi) * cosd(phi),   cosd(psi) * sind(theta) * cosd(phi) + sind(psi) * sind(phi),   dx;
         sind(psi) * cosd(theta),   sind(psi) * sind(theta) * sind(phi) + cosd(psi) * cosd(phi),   sind(psi) * sind(theta) * cosd(phi) - cosd(psi) * sind(phi),   dy;
         -sind(theta),              cosd(theta) * sind(phi),                                       cosd(theta) * cosd(phi),                                       dz;
         0,                         0,                                                             0,                                                             1];
end


function path = dijkstra(adj_matrix, start_idx, goal_idx)
    % 노드 수
    num_nodes = size(adj_matrix, 1);

    % 무한대 값 설정
    inf_val = inf;

    % 최단 거리 배열 및 이전 노드 배열 초기화
    dist = inf_val * ones(1, num_nodes);
    dist(start_idx) = 0;
    prev = -1 * ones(1, num_nodes);

    % 방문 여부 배열 초기화
    visited = false(1, num_nodes);

    % 우선순위 큐 초기화 (거리를 기준으로 정렬)
    pq = [start_idx, 0];

    while ~isempty(pq)
        % 우선순위 큐에서 최소 거리 노드 선택
        [~, min_idx] = min(pq(:, 2));
        current_node = pq(min_idx, 1);
        pq(min_idx, :) = [];

        % 현재 노드를 방문으로 표시
        visited(current_node) = true;

        % 목표 노드에 도달하면 경로 생성
        if current_node == goal_idx
            path = [];
            while current_node ~= -1
                path = [current_node, path];
                current_node = prev(current_node);
            end
            return;
        end

        % 인접한 모든 노드에 대해 거리 갱신
        for neighbor = 1:num_nodes
            if adj_matrix(current_node, neighbor) > 0 && ~visited(neighbor)
                new_dist = dist(current_node) + adj_matrix(current_node, neighbor);
                if new_dist < dist(neighbor)
                    dist(neighbor) = new_dist;
                    prev(neighbor) = current_node;
                    pq = [pq; neighbor, new_dist];
                end
            end
        end
    end

    % 경로를 찾지 못한 경우 빈 배열 반환
    path = [];
end



% 막대랑 벽 collision box
rod = collisionBox(0.5, 0.5, 2);
wall_0 = collisionBox(5, 0.3, 5);
wall_1 = collisionBox(5, 0.3, 5);
wall_2 = collisionBox(5, 0.3, 5);
wall_3 = collisionBox(5, 0.3, 5);

%rod.Pose = [rodRotation, rodPosition'; 0 0 0 1];
rod.Pose = move(0,0,0,0,0,0);

% 벽 위치 설정
wallPosition_0 = [-3.4, 0, 0]; % 벽의 위치 (x, y, z 좌표)
wallPosition_1 = [0, 0, -3.0];
wallPosition_2 = [3.4, 0, 0];
wallPosition_3 = [0, 0, 3.0];

wall_0.Pose = trvec2tform(wallPosition_0);
wall_1.Pose = trvec2tform(wallPosition_1);
wall_2.Pose = trvec2tform(wallPosition_2);
wall_3.Pose = trvec2tform(wallPosition_3);


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

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


% 샘플링 방법
range_x = [-3, 3];
range_y = [-3, 3];
range_z = [-3, 3];

%거리계산시 노멀라이즈가 필요할듯
range_psi = [0, 180];
range_theta = [0, 180];
range_phi = [0,180];


% 각 차원에 대해 무작위 샘플링
num_samples = 1000;
samples_x = (range_x(2) - range_x(1)) * rand(num_samples, 1) + range_x(1);
samples_y = (range_y(2) - range_y(1)) * rand(num_samples, 1) + range_y(1);
samples_z = (range_z(2) - range_z(1)) * rand(num_samples, 1) + range_z(1);
samples_psi = (range_psi(2) - range_psi(1)) * rand(num_samples, 1) + range_psi(1);
samples_theta = (range_theta(2) - range_theta(1)) * rand(num_samples, 1) + range_theta(1);
samples_phi = (range_phi(2) - range_phi(1)) * rand(num_samples, 1) + range_phi(1);

% 샘플링된 x, y, z 좌표를 결합
samples = [samples_x, samples_y, samples_z, samples_psi, samples_theta, samples_phi];

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% 시각화
figure;
hold on;
xlabel('X');
ylabel('Y');
zlabel('Z');
grid on;
axis equal;
title('Rod and Wall Visualization');

show(wall_0, 'Parent', gca);
show(wall_1, 'Parent', gca);
show(wall_2, 'Parent', gca);
show(wall_3, 'Parent', gca);

num = 0;

%충돌이 없는 rods 저장
sample_rods = [];

start = [1,-3,-2,0,0,0];
goal = [-2,3,-1,55,95,120];
sample_rods(end +1, :) = start;
sample_rods(end +1, :) = goal;


for i = 1:num_samples
    % 각 샘플에 따라 rod의 Pose 설정
    rod.Pose = move(samples(i, 1), samples(i, 2), samples(i, 3), ...
                    samples(i, 4), samples(i, 5), samples(i, 6));
    
    isColliding = false;
    isColliding = isColliding | checkCollision(rod, wall_0);
    isColliding = isColliding | checkCollision(rod, wall_1);
    isColliding = isColliding | checkCollision(rod, wall_2);
    isColliding = isColliding | checkCollision(rod, wall_3);
    

    if isColliding
        %disp('충돌이 발생했습니다.');
    else
        sample_rods(end +1, :) = [samples(i, 1), samples(i, 2), samples(i, 3), samples(i, 4), samples(i, 5), samples(i, 6)];
        %disp('충돌이 발생하지 않았습니다.'); 
        %num = num+1
    end

end



%start, goal 시각화


rod.Pose = move(start(1),start(2),start(3),start(4),start(5),start(6));
show(rod, 'Parent', gca)
rod.Pose = move(goal(1),goal(2),goal(3),goal(4),goal(5),goal(6));
show(rod, 'Parent', gca)

% 이제 모든 노드들에 대해 k개의 가까운 거리를 저장하는 인접행렬 생성.
% 다만 중간이 끊기지 않아야함.
% 이후 다익스트라같은 알고리즘으로 최단거리 탐색
% 시각화

num_nodes = size(sample_rods, 1);
adj_matrix = inf(num_nodes); % 무한대 거리로 초기화

% 거리 계산 및 인접 행렬 구성
for i = 1:num_nodes
    for j = i+1:num_nodes
        rod_a = sample_rods(i, 1:6);
        rod_a(4:6) = rod_a(4:6) / 360;
        rod_b = sample_rods(j, 1:6);
        rod_b(4:6) = rod_b(4:6) / 360;
        distance = norm(rod_a - rod_b); %각의 크기가 길이에 비해 매우 크므로 정규화가 필요함 /180 정도

        adj_matrix(i, j) = distance;
        adj_matrix(j, i) = distance;
        
    end
end

divide = 4;
%4등분해서 사이 3개점에서도 충돌 없으면 거리에 추가
min_k = 300;
for i = 1:num_nodes
    [sortedDistances, sortedIndices] = sort(adj_matrix(i, :), 'ascend');
    for k = 1 : min_k
        isColliding = false;
        for j = 1:divide-1
            point = j/divide * sample_rods(sortedIndices(k),:) + (1-j/divide)*sample_rods(i,:);
            rod.Pose = move(point(1),point(2),point(3),point(4),point(5),point(6));

            isColliding = isColliding | checkCollision(rod, wall_0);
            isColliding = isColliding | checkCollision(rod, wall_1);
            isColliding = isColliding | checkCollision(rod, wall_2);
            isColliding = isColliding | checkCollision(rod, wall_3);
            
            if isColliding == true
                break
            end
        end

        if isColliding
         %disp('경로에 충돌이 발생했습니다.');
            adj_matrix(i, sortedIndices(k)) = inf;
            adj_matrix(sortedIndices(k), i) = inf;
        end

     end



    % 짧은 거리 10개를 제외한 나머지 연결을 inf로 설정
    for k = min_k+1:length(sortedDistances)
        adj_matrix(i, sortedIndices(k)) = inf;
        adj_matrix(sortedIndices(k), i) = inf;
    end
end

%경로 시각화


adj_matrix

% 시작점과 목표점 인덱스
start_idx = 1;
goal_idx = 2;

% 다익스트라 알고리즘으로 최단 경로 탐색
path = dijkstra(adj_matrix, start_idx, goal_idx);
 
if isempty(path)
    disp('경로 생성 실패');
else
    disp('경로 탐색 성공');
    % 최단 경로 시각화

    for i = 1:length(path)-1
        point1 = sample_rods(path(i), 1:6);
        point2 = sample_rods(path(i+1), 1:6);
        
        % 8등분하여 중간 지점들을 계산하고 시각화
        for j = 0:7
            intermediate_point = (1 - j/8) * point1 + (j/8) * point2;
            T_intermediate = move(intermediate_point(1), intermediate_point(2), intermediate_point(3), ...
                                  intermediate_point(4), intermediate_point(5), intermediate_point(6));
            

            now_rod = T_intermediate * vertices'; % (4,4) * (4,8)
            now_rod = now_rod(1:3, :)';
            patch('Vertices', now_rod, 'Faces', faces, ...
                  'FaceColor', 'r', 'FaceAlpha', 0.5, 'EdgeColor', 'k');
            
            pause(0.2);
        end
    end
end
hold off;




%path를 찾는데 시간이 너무 많이 걸리는데 영상을 찍기 힘들어서 txt로 저장하고 나중에 촬영
fileID = fopen('path_output.txt', 'w');
fprintf(fileID, 'Path Data:\n');
fprintf(fileID, 'Index   x       y       z       psi     theta   phi\n');

for i = 1:length(path)
    fprintf(fileID, '%d      %.2f   %.2f   %.2f   %.2f   %.2f   %.2f\n', ...
            path(i), sample_rods(path(i), 1), sample_rods(path(i), 2), sample_rods(path(i), 3), ...
            sample_rods(path(i), 4), sample_rods(path(i), 5), sample_rods(path(i), 6));
end

fclose(fileID);