clearvars;
close all;
clc;

function prob_z_x = sensor_on(sensors, prob_z_x)

    sensor_prob = [0.05,0.1,0.9,0.1,0.05];
    sensor_prob = sensor_prob / sum(sensor_prob);
    for i = 1: size(sensors,1)
        for j = 1: size(sensor_prob(:),1)
            prob_z_x(sensors(i,1)+j-3, 1, 1) = sensor_prob(j);
            prob_z_x(sensors(i,2)+j-3, 1, 2) = sensor_prob(j);
        end
    end
    %확률 출력
    prob_z_x(:,1,1) = prob_z_x(:,1,1) / sum(prob_z_x(:,1,1));
    prob_z_x(:,1,2) = prob_z_x(:,1,2) / sum(prob_z_x(:,1,2));
end



%완전한 연속은 불가능.
n = 200;
states_x = linspace(0, n, n);
states_y = linspace(0, n, n);

%초기 belief 모든 state에 대해 계산됨
belief = ones(n, 1, 2) / n;

%prob(z|x) sensor
%센서 설치하면 이 값에 반영
prob_z_x = ones(n,1,2) / n;
prob_z_xy = ones(n,n) / (n^2);

%sensor 설치
%sensors = randi(n-10, 200,2)+5;

sensors = [
    15,10;
    60,40;
    120,80;
    180,120;

    60,120;
    80,110;
    140,70;
    190,80;
    ];

state_transition_x = [0.05;0.1;0.9;0.1;0.05];
state_transition_x = state_transition_x / sum(state_transition_x);

state_transition_y = [0.01;0.9;0.01;0.000001;0.000001];
state_transition_y = state_transition_y / sum(state_transition_y);


%모든 스테이트에 대해 적용
state_transition_matrix_x = zeros(n,n);
state_transition_matrix_y = zeros(n,n);
for j = 1:n
    for i = 1:5
        %대각성분에서 밑으로 state_transition들 모두 채워주기
        if j + i <= n
            state_transition_matrix_x(j + i, j) = state_transition_x(i);
            state_transition_matrix_y(j + i, j) = state_transition_y(i);
        end
    end
end

for j = 1:n
    if sum(state_transition_matrix_x(:, j)) > 0
        state_transition_matrix_x(:, j) = state_transition_matrix_x(:, j) / sum(state_transition_matrix_x(:, j));
    end
    if sum(state_transition_matrix_y(:, j)) > 0
        state_transition_matrix_y(:, j) = state_transition_matrix_y(:, j) / sum(state_transition_matrix_y(:, j));
    end
end

%state_transition_matrix(n,:)
%size(state_transition_matrix(n,:))




x = [0,0];







% 3D 플롯 설정
figure;
[X, Y] = meshgrid(states_x, states_y);
belief_xy = belief(:,:,1) * belief(:,:,2)';
belief_plot = surf(X, Y, belief_xy, 'EdgeColor', 'none');
xlabel('State X');
ylabel('State Y');
zlabel('Belief');
title('3D Belief Distribution');
colormap jet;
colorbar;
zlim([0, 0.2]); 
hold on;



plot3(states_x(sensors(:, 1)), states_y(sensors(:, 2)), zeros(size(sensors, 1), 1), 'ro', 'MarkerSize', 5, 'MarkerFaceColor', 'r');
x_marker = plot3(states_x(x(1) + 1), states_y(x(2) + 1), 0, 'ko', 'MarkerSize', 8, 'MarkerFaceColor', 'k');
p_z_x_distribution = prob_z_x(:,:,1) * prob_z_x(:,:,2)';
sensor_plot = surf(X, Y, p_z_x_distribution, 'EdgeColor', 'none', 'FaceAlpha', 0.5);

for t = 1:n
    if x(1) > n || x(2) > n
        fprintf("최후에 도달");
        break
    end
    
    %이동
    random_num = randi(100)/100;
    temp = 0;
    for i = 1:size(state_transition_x(:),1)
        temp = temp + state_transition_x(i);
        if random_num < temp
            x(1) = x(1)+ i;
            break
        end
    end
    temp=0;
    random_num = randi(100)/100;
    for i = 1:size(state_transition_y(:),1)
        temp = temp + state_transition_y(i);
        if random_num < temp
            x(2) = x(2)+ i;
            break
        end
    end
    
    %sensor에 닿으면 prob_z_x 떡상
    prob_z_x = ones(n,1,2)/ n;
    for i = 1:size(sensors,1)
        if norm(x-sensors(i,:),2) <= 5
            prob_z_x = sensor_on(sensors, prob_z_x);
            fprintf('sensor on x : (%d, %d)\n', sensors(i,1),sensors(i,2));
        end

    end
   

    %size(state_transition_matrix )
    %size(belief)
    %size(prob_z_x(:,:,1))
    %size((state_transition_matrix_x * belief(:,:,1)))

    belief(:,:,1) =  prob_z_x(:,:,1) .* (state_transition_matrix_x * belief(:,:,1));
    belief(:,:,1) = belief(:,:,1) /sum(belief(:,:,1));

    belief(:,:,2) =  prob_z_x(:,:,2) .* (state_transition_matrix_y * belief(:,:,2));
    belief(:,:,2) = belief(:,:,2) /sum(belief(:,:,2));
    
    %belief_xy = belief(:,:,2) * belief(:,:,1)';
    belief_xy = ones(n, n)/(n^2);
    for i = 1:n
        for j = 1:n
            belief_xy(i,j) = prob_z_x(i,1,1) * prob_z_x(j,1,2) * belief(i,1,1) * belief(j,1,2);
        end
    end

    
    belief_xy = belief_xy /sum(belief_xy(:));
    %belief_xy(t:t+ 5,t:t+5)

    set(belief_plot, 'ZData', belief_xy);
    set(x_marker, 'XData', states_x(x(1) + 1), 'YData', states_y(x(2) + 1), 'ZData', 0);
    drawnow;

    pause(0.2);
end
