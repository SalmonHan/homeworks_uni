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

function result = preorderTraversal(G, currentNode, result)
    % 현재 노드를 결과에 추가
    result(end + 1) = currentNode;
    
    % 현재 노드의 자식 노드(후속 노드) 가져오기
    childNodes = successors(G, currentNode);
    
    % 모든 자식 노드에 대해 재귀적으로 전위 순회 수행
    for i = 1:length(childNodes)
        result = preorderTraversal(G, childNodes(i), result);
    end
end

function isColliding = check_collision(rod, position, walls)
    isColliding = false;
    rod.Pose = move(position(1),position(2),position(3),position(4),position(5),position(6));
    
    wall_0 = walls{1};
    wall_1 = walls{2};
    wall_2 = walls{3};
    wall_3 = walls{4};
    
   
    isColliding = isColliding | checkCollision(rod, wall_0);
    isColliding = isColliding | checkCollision(rod, wall_1);
    isColliding = isColliding | checkCollision(rod, wall_2);
    %isColliding = isColliding | checkCollision(rod, wall_3);
    

end

function path_positions = getPathPositions(tree_start, tree_node_list, goal)
    % 시작 지점(1번 인덱스)에서 목표 지점(goal)까지의 경로 탐색
    start_idx = 1;
    
    % 시작 지점에서 목표 지점까지의 최단 경로 찾기
    path_indices = shortestpath(tree_start, start_idx, goal);
    
    % 경로의 위치 벡터를 저장할 배열 초기화
    path_positions = [];
    
    % 각 경로 인덱스에 대해 위치 벡터 추가
    for i = 1:length(path_indices)
        node_index = path_indices(i);
        path_positions(end + 1, :) = tree_node_list(node_index, :); % 위치 벡터 추가
    end
    
    % 경로 위치 벡터 출력
    disp('경로의 위치 벡터:');
    disp(path_positions);
end

function distance = calc_distance(point1, point2)

    dist_x = point1(1) - point2(1);
    dist_y = point1(2) - point2(2);
    dist_z = point1(3) - point2(3);
    dist_psi = min(point1(4) - point2(4), 360 - (point1(4) - point2(4))) / 60;
    dist_theta = min(point1(5) - point2(5), 360 - (point1(5) - point2(5))) / 60;
    dist_phi = min(point1(6) - point2(6), 360 - (point1(6) - point2(6))) / 60;

    
    distance = norm([dist_x,dist_y,dist_z,dist_psi,dist_theta,dist_phi]);
end

function samples = create_collision_free_samples(n)
    range_x = [-3, 3];
    range_y = [-3, 3];
    range_z = [-3, 3];
    
    range_psi = [0, 360];
    range_theta = [0, 360];
    range_phi = [0, 360];

    num_samples = n;
    samples_x = (range_x(2) - range_x(1)) * rand(num_samples, 1) + range_x(1);
    samples_x = min(3, samples_x);
    samples_x = max(-3, samples_x);
    samples_y = (range_y(2) - range_y(1)) * rand(num_samples, 1) + range_y(1);
    samples_y = min(3, samples_y);
    samples_y = max(-3, samples_y);
    samples_z = (range_z(2) - range_z(1)) * rand(num_samples, 1) + range_z(1);
    samples_z = min(3, samples_z);
    samples_z = max(-3, samples_z);
    samples_psi = (range_psi(2) - range_psi(1)) * rand(num_samples, 1) + range_psi(1);
    samples_psi = min(samples_psi, 360-samples_psi);
    samples_theta = (range_theta(2) - range_theta(1)) * rand(num_samples, 1) + range_theta(1);
    samples_theta = min(samples_theta, 360-samples_theta);
    samples_phi = (range_phi(2) - range_phi(1)) * rand(num_samples, 1) + range_phi(1);
    samples_phi = min(samples_phi, 360 - samples_phi);
    % 샘플링된 x, y, z 좌표를 결합
    samples = [samples_x, samples_y, samples_z, samples_psi, samples_theta, samples_phi];
end



% 막대랑 벽 collision box
rod = collisionBox(0.5, 0.5, 2);
wall_0 = collisionBox(5, 0.3, 5);
wall_1 = collisionBox(4, 0.3, 4);
wall_2 = collisionBox(4, 0.3, 4);
wall_3 = collisionBox(5, 0.3, 5);



%rod.Pose = [rodRotation, rodPosition'; 0 0 0 1];
rod.Pose = move(0,0,0,0,0,0);

% 벽 위치 설정
wallPosition_0 = [-2.5, 1, 2.0]; % 벽의 위치 (x, y, z 좌표)
wallPosition_1 = [0.5 -1.5, -3.0];
wallPosition_2 = [3.2, 0, -1];
wallPosition_3 = [0, -0.5, 4.0];

wall_0.Pose = trvec2tform(wallPosition_0);
wall_1.Pose = trvec2tform(wallPosition_1);
wall_2.Pose = trvec2tform(wallPosition_2);
wall_3.Pose = trvec2tform(wallPosition_3);

walls = {wall_0,wall_1,wall_2,wall_3};

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

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% 벽 시각화

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
%show(wall_3, 'Parent', gca);


start = [1,-3,-2,0,0,0];
goal = [-2,3,-1,55,95,120];

rod.Pose = move(start(1),start(2),start(3),start(4),start(5),start(6));
show(rod, 'Parent', gca)
rod.Pose = move(goal(1),goal(2),goal(3),goal(4),goal(5),goal(6));
show(rod, 'Parent', gca)




% 1. tree를 구성
% 각 q의 인덱스를 트리의 노드로 사용.
% tree_node_list가 인덱스에 해당하는 6차원 좌표.

tree_start = digraph(); 
tree_node_list = [];
tree_node_list(end +1, :) = start;

dist_min = inf;
% 1. random point R를 하나 찍는다.
% 2. 트리에서 R과 가장 가까운 node node_nearest를 찾는다
% 3. node_nearest에서 R까지 stepsize만큼 다가간 새로운 지점이 collision_free한지 확인하고 free하다면 트리에 추가
% 4. 추가한 노드가 goal에 충분히 가깝다면 goal과 연결할 수 있는지 check
% 5. 실패할때까지 추가하다가 실패하면 다시 1번으로 회귀
explore_success = false;
goal_index = -1;

step_size = 0.5;
while true
    w_max = -inf;
    q_list = [];
    q_selected = 0;

    if size(tree_node_list,1) == 1
        q_list = [1];
    else
        q_list = preorderTraversal(tree_start, 1, q_list);
    end
    
    q_selected = randi(size(q_list));
    %q_selected를 선택했다면 이제 여기서 다음 노드를 랜덤으로 뽑아서 연결 되는지 보고 연결.

    samples = create_collision_free_samples(1);

    for i = 1:1
        devide = round(calc_distance(tree_node_list(q_selected, :), samples(i,:))/ step_size);
        isColliding = false;
        for j  = 1:devide
            point = (1-(j/devide))*tree_node_list(q_selected, :) + (j/devide)*samples(i,:);
            %check_collision(rod, point, walls)
            isColliding = isColliding | check_collision(rod, point, walls);
            
            
            if isColliding == true
                break
            else
                %새로운 노드 추가
                tree_node_list(end +1, :) = point;
                if j == 1
                    tree_start = addedge(tree_start, q_selected, size(tree_node_list,1));
                else
                    tree_start = addedge(tree_start, size(tree_node_list,1) - 1, size(tree_node_list,1));
                end

                %추가했다면 추가한 샘플들 중에 goal과 충분히 가깝다면 goal과 직빵 연결 가능한애 있는지 check
                
                dist_min = min(dist_min, calc_distance(point, goal(:)));
                if calc_distance(point, goal(:)) < 1.5
                    devide = 8;
                    isColliding = false;
                    for l  = 1:devide-1
                        inter_point = (l/devide)*point + (1-(l/devide))*goal;
                        isColliding = isColliding | check_collision(rod, inter_point, walls);
                
                        if isColliding == true
                            fprintf("막혀있음");
                            break
                        end
                
                    end
                
                if isColliding == false
                    fprintf("탐색 성공\n");
            
                    %마지막 point와 goal 사이에 포인트들을 입력
                    last_interpolation = round(calc_distance(point, goal(:))*2);
                    last_interpolation
                    for l = 1:last_interpolation-1
                        inter_point = (l/last_interpolation)*point + (1-(l/last_interpolation))*goal;
                        tree_node_list(end +1, :) = inter_point(:);
                        tree_start = addedge(tree_start, size(tree_node_list,1)-1, size(tree_node_list,1));
                    end
            
            
                    tree_node_list(end +1, :) = goal;
                    tree_start = addedge(tree_start, size(tree_node_list,1)-1, size(tree_node_list,1));
                    goal_index = size(tree_node_list,1);
                    explore_success = true;
                end    
                
                end
            end
        end
    end






    
    if explore_success == true
        break
    end
    size(tree_node_list, 1)
    dist_min
    tree_node_list(q_selected, 1:6)
end


plot(tree_start);















path = getPathPositions(tree_start, tree_node_list, goal_index);
for i = 1:length(path)-1
    point = path(i,:);
    
    T= move(point(1), point(2), point(3), point(4), point(5), point(6));
    
    now_rod = T * vertices'; % (4,4) * (4,8)
    now_rod = now_rod(1:3, :)';
    patch('Vertices', now_rod, 'Faces', faces, ...
          'FaceColor', 'r', 'FaceAlpha', 0.5, 'EdgeColor', 'k');
    %pause(0.1);

end
hold off;




%path를 찾는데 시간이 너무 많이 걸리는데 영상을 찍기 힘들어서 txt로 저장하고 나중에 촬영
fileID = fopen('path_output_hw_4_3.txt', 'w');
fprintf(fileID, 'Path Data:\n');
fprintf(fileID, 'Index   x       y       z       psi     theta   phi\n');

% 경로 데이터 파일에 쓰기
for i = 1:size(path, 1)
    fprintf(fileID, '%-10d %-10.4f %-10.4f %-10.4f %-10.4f %-10.4f %-10.4f\n', ...
            i, path(i, 1), path(i, 2), path(i, 3), path(i, 4), path(i, 5), path(i, 6));
end

% 파일 닫기
fclose(fileID);

disp('경로 데이터가 파일로 저장되었습니다.');