clearvars;
close all;
clc;

function prob_z_x = sensor_on(sensors, prob_z_x)

    sensor_prob = [0.1,0.2,0.5,0.2,0.1];
    sensor_prob = sensor_prob / sum(sensor_prob);
    for i = 1: size(sensors(:),1)
        for j = 1: size(sensor_prob(:),1)
            prob_z_x(sensors(i)+ j -3) = sensor_prob(j);
        end
    end
    %확률 출력
    prob_z_x = prob_z_x / sum(prob_z_x);
end



%완전한 연속은 불가능.
n = 1000;
states = linspace(0, 100, n);

%초기 belief 모든 state에 대해 계산됨
belief = ones(n, 1) / n;

%prob(z|x) sensor
%센서 설치하면 이 값에 반영
prob_z_x = ones(n,1) / n;

%sensor 설치
sensors = [30,170, 410, 710];


state_transition = [0.1;0.2;0.8;0.2;0.1];
state_transition = state_transition / sum(state_transition);

%모든 스테이트에 대해 적용
state_transition_matrix = zeros(n,n);
for j = 1:n
    for i = 1:5
        %대각성분에서 밑으로 state_transition들 모두 채워주기
        if j + i <= n
            state_transition_matrix(j + i, j) = state_transition(i);
        end
    end
end

for j = 1:n
    col_sum = sum(state_transition_matrix(:, j));
    if col_sum > 0  % 열의 합이 0이 아닐 경우에만 정규화
        state_transition_matrix(:, j) = state_transition_matrix(:, j) / col_sum;
    end
end
%state_transition_matrix(n,:)
%size(state_transition_matrix(n,:))






% Set up figure for visualization
figure;
hold on;
belief_plot = plot(states, belief, 'b', 'LineWidth', 2);
sensor_plot = plot(states, prob_z_x, 'r', 'LineWidth', 2);
x_marker = plot(states(1), 0, 'ko', 'MarkerSize', 8, 'MarkerFaceColor', 'k');
xlabel('State');
ylabel('Probability');
legend('Belief', 'Sensor Probability', 'Current Position');

for k = 1:length(sensors)
    plot(states(sensors(k)), 0, 'gx', 'MarkerSize', 10, 'LineWidth', 2,'HandleVisibility', 'off'); 
end


ylim([0, 0.25]);

x = 0;
for t = 1:n
    if x > n
        fprintf("최후에 도달");
        break
    end

    random_num = randi(100)/100;
    temp = 0;
    for i = 1:size(state_transition(:),1)
        temp = temp + state_transition(i);
        if random_num < temp
            x = x+ i;
            break
        end
    end
    
    %sensor에 닿으면 prob_z_x 떡상

    prob_z_x = ones(n,1)/ n;
    for i = 1:size(sensors(:) ,1)
        if abs(x-sensors(i)) <= 2
            prob_z_x = sensor_on(sensors, prob_z_x);
            fprintf('sensor on x : %d\n', x);
        end

    end
   

    %size(state_transition_matrix )
    %size(belief)

    belief =  prob_z_x .* (state_transition_matrix * belief);
    % eta 계산
    belief = belief /sum(belief);



    set(belief_plot, 'YData', belief); 
    set(sensor_plot, 'YData', prob_z_x);  
    set(x_marker, 'XData', states(x + 1)); 
    drawnow;     
    
    pause(0.05);
end
