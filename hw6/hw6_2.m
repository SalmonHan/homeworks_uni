clearvars;
close all;
clc;


u = [5; 3]; 
state_transition = eye(2); 
process_noise_variance = 2;
measurement_noise_variance = 10;



sensors = rand(2, 50) * 200;
num_sensors = size(sensors, 2);

mu = [0; 0];
sigma2 = eye(2) * 4; 

x = [0; 0]; 
z = NaN(2, 1);

figure;
hold on;
x_range = 0:0.5:200;
y_range = 0:0.5:200;

plot(sensors(1, :), sensors(2, :), 'ro', 'MarkerSize', 10, 'LineWidth', 2);


for t = 1:100
    x = x + u + randn(2, 1) * process_noise_variance;

    z = NaN(2, 1); 
    for i = 1:num_sensors
        if norm(x - sensors(:, i)) < 10
            z = sensors(:, i) + randn(2, 1) * measurement_noise_variance;
        end
    end

    % predict
    mu_predict = mu + u;
    sigma2_predict = state_transition * sigma2 * state_transition' + eye(2) * process_noise_variance;

    % update
    if ~isnan(z(1))
        kalman_gain = sigma2_predict / (sigma2_predict + eye(2) * measurement_noise_variance);
        mu = mu_predict + kalman_gain * (z - mu_predict);
        sigma2 = (eye(2) - kalman_gain) * sigma2_predict;
    else% z가 없을 경우 업데이트 없이 예측만 유지
        mu = mu_predict;
        sigma2 = sigma2_predict;
    end


    [X, Y] = meshgrid(x_range, y_range);
    grid_points = [X(:), Y(:)]; % Each row is a [x, y] pair
    
    bel = mvnpdf(grid_points, mu', sigma2);
    bel = reshape(bel, size(X)); 
    











    % Visualization
    clf; % Clear previous plot
    hold on;

    % Plot sensors
    plot3(sensors(1, :), sensors(2, :), zeros(1, num_sensors), 'ro', 'MarkerSize', 5, 'LineWidth', 1);

    % Plot true position
    plot3(x(1), x(2), 0, 'bo', 'MarkerSize', 10, 'LineWidth', 2);

    % Plot estimated position
    plot3(mu(1), mu(2), 0, 'go', 'MarkerSize', 10, 'LineWidth', 2);

    % Plot belief distribution as 3D surface
    surf(X, Y, bel, 'EdgeColor', 'none');
    colorbar; % Add color bar to indicate probability density

    % Axes and title
    title(['3D Kalman Filter Belief Visualization at Time t = ', num2str(t)]);
    xlabel('X Position');
    ylabel('Y Position');
    zlabel('Probability Density');
    axis([0 200 0 200 0 0.2]);
    grid on;
    view(45, 30);


    pause(0.2);
end