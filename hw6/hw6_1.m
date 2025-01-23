clearvars;
close all;
clc;

u = 5;
state_transition = 1;

process_noise_variance = 2;
measurement_noise_variance = 10;

sensors = [70, 145, 200, 340];
num_sensors = length(sensors);

mu = 0;
sigma2 = 5;

x = 0;
z = NaN;

% 그래프 시각화 설정
figure;
hold on;
x_range = -10:0.1:400;

% 센서 위치를 표시
plot(sensors, zeros(size(sensors)), 'ro', 'MarkerSize', 10, 'LineWidth', 2);
text(sensors + 2, zeros(size(sensors)), {'Sensor 1', 'Sensor 2', 'Sensor 3', 'Sensor 4'}, 'FontSize', 10);

for t = 1:100
    x = x + u + randn * process_noise_variance;
    
    % 센서 안에 들어오면 sensor 내에서 랜덤 샘플링
    z = NaN; % 기본값: NaN, 센서 밖이면 update x
    for i = 1:length(sensors)
        if abs(x - sensors(i)) < 5
            z = sensors(i) + randn * measurement_noise_variance;
        end
    end

    % predict
    mu_predict = mu + u;
    sigma2_predict = state_transition^2 * sigma2 + process_noise_variance;

    % update
    if ~isnan(z) 
        kalman_gain = sigma2_predict / (sigma2_predict + measurement_noise_variance);
        mu = mu_predict + kalman_gain * (z - mu_predict);
        sigma2 = (1 - kalman_gain) * sigma2_predict;
    else
        % z가 없을 경우 업데이트 없이 예측만 유지
        mu = mu_predict;
        sigma2 = sigma2_predict;
    end


    bel = normpdf(x_range, mu, sqrt(sigma2));




    clf;
    hold on;

    plot(sensors, zeros(size(sensors)), 'ro', 'MarkerSize', 10, 'LineWidth', 2);
    plot(x, 0, 'bo', 'MarkerSize', 10, 'LineWidth', 2);
    plot(x_range, bel, 'k-', 'LineWidth', 2);



    title(['Kalman Filter Visualization at Time t = ', num2str(t)]);
    xlabel('Position');
    ylabel('Probability Density');
    ylim([0, 0.5]);

    pause(0.15);
end
