%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Task 4 - Extended Kalman Filter for Localisation
% - Combines motion prediction (velocity model) with landmark observations (ArUco)
% - Reduces pose uncertainty using sensor fusion
% - Saves filtered pose estimate and comparison plots
% Author: Kaung Min Khant
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

clear; clc;

%%  Load Required Data 
% Load:
% - Ground truth pose [x y theta] from simulation
% - Noisy odometry from Task 2 (velocity motion model)
% - Velocity commands [v; w] from Task 3 controller
% - ArUco range & bearing observations (9 markers per timestep)
gt_pose    = load('/home/student/Downloads/Assignment/Task3/task3_dataset/ground_truth.dat');      
noisy_pose = load('/home/student/Downloads/Assignment/Task3/task3_dataset/noisy_pose.dat');         
vels       = load('/home/student/Downloads/Assignment/Task3/task3_dataset/vel_sent.dat')';      
z          = load('/home/student/Downloads/Assignment/Task3/task3_dataset/aruco_measurements.dat');

% Marker landmark positions (IDs 1–9)
landmarks = [
   -0.1, 2.0;    
   -3.0, 1.0;    
    0.9, 0.5;    
    1.0, 3.0;    
    3.0, 1.0;    
    2.0, -3.0;   
    0.5, -0.1;   
   -3.0, -1.0;   
   -2.5, -3.0    
];

%% EKF Parameters
alpha = [0.1, 0.02, 0.3, 0.01, 0.001, 0.01];        % Motion noise parameters
R = diag([0.05, deg2rad(2)]).^2;                   % Observation noise (range, bearing)
dt = 0.03;                                         % Time step (30Hz control rate)

%% EKF Initialisation 
num_steps = size(gt_pose, 1);                      % Number of time steps
mu = noisy_pose(1,:)';                             % Initial state estimate (x, y, theta)
Sigma = eye(3) * 0.1;                              % Initial covariance
mu_history = zeros(3, num_steps);                  % To store pose estimates over time
mu_history(:,1) = mu;

%% EKF Loop: Prediction + Correction 
for t = 1:(num_steps - 1)
    % Prediction step (motion model) 
    v = vels(t,1);
    w = vels(t,2);
    theta = mu(3);

    % Predict next state
    mu_bar = mu + [v * cos(theta) * dt;
                   v * sin(theta) * dt;
                   w * dt];
    mu_bar(3) = wrapToPi(mu_bar(3));               % Keep theta in [-π, π]

    % Jacobians
    G = [1, 0, -v * sin(theta) * dt;
         0, 1,  v * cos(theta) * dt;
         0, 0, 1];

    V = [cos(theta)*dt, 0;
         sin(theta)*dt, 0;
         0, dt];

    M = [alpha(1)*v^2 + alpha(2)*w^2, 0;
         0, alpha(3)*v^2 + alpha(4)*w^2];

    Sigma_bar = G * Sigma * G' + V * M * V';       % Predicted covariance

    % Correction step (landmark observations) 
    for j = 1:9
        z_t = z(t, (2*j-1):(2*j))';                 % Get [range; bearing] for marker j

        if all(isnan(z_t)) || all(z_t == 0)
            continue;                              % Skip if marker not seen
        end

        % Compute expected measurement
        lx = landmarks(j,1);
        ly = landmarks(j,2);
        dx = lx - mu_bar(1);
        dy = ly - mu_bar(2);
        q = dx^2 + dy^2;

        z_hat = [sqrt(q);                           % Predicted range
                 wrapToPi(atan2(dy, dx) - mu_bar(3))];  % Predicted bearing

        % Measurement Jacobian
        H = [-dx/sqrt(q), -dy/sqrt(q), 0;
              dy/q,       -dx/q,      -1];

        % Innovation
        y = z_t - z_hat;
        y(2) = wrapToPi(y(2));                      % Normalize angle

        % Kalman update
        S = H * Sigma_bar * H' + R;
        K = Sigma_bar * H' / S;

        mu_bar = mu_bar + K * y;
        mu_bar(3) = wrapToPi(mu_bar(3));            % Normalize updated theta
        Sigma_bar = (eye(3) - K * H) * Sigma_bar;
    end

    % Update state and covariance
    mu = mu_bar;
    Sigma = Sigma_bar;
    mu_history(:, t+1) = mu;
end

%% Save Results 
if ~exist('task4_dataset', 'dir'), mkdir('task4_dataset'); end
save('task4_dataset/ekf_estimate.mat', 'mu_history');
save('task4_dataset/ekf_estimates.dat', 'mu_history', '-ascii');

%% Plot 1: Trajectory Comparison (Cleaned)
figure(1); clf; hold on;

% Paths
plot(gt_pose(:,1), gt_pose(:,2), 'g-', 'LineWidth', 1.5);         % Ground Truth
plot(noisy_pose(:,1), noisy_pose(:,2), 'r--', 'LineWidth', 1.2);  % Noisy Odometry
plot(mu_history(1,:), mu_history(2,:), 'b-', 'LineWidth', 1.5);   % EKF Estimate

% Start and End Markers
plot(gt_pose(1,1), gt_pose(1,2), 'go', 'MarkerSize', 10, 'MarkerFaceColor','g');     % Start (GT)
plot(gt_pose(end,1), gt_pose(end,2), 'gx', 'MarkerSize', 10, 'LineWidth', 2);        % End (GT)
plot(mu_history(1,1), mu_history(2,1), 'bo', 'MarkerSize', 10, 'MarkerFaceColor','b'); % Start (EKF)
plot(mu_history(1,end), mu_history(2,end), 'bx', 'MarkerSize', 10, 'LineWidth', 2);  % End (EKF)

% Labels, Title, Legend
xlabel('X Position (m)');
ylabel('Y Position (m)');
title('Trajectory Comparison: Ground Truth vs Noisy vs EKF');
axis equal; grid on;

% Custom legend to avoid duplicates
legend({
    'Ground Truth', ...
    'Noisy Odometry', ...
    'EKF Estimate', ...
    'Start (GT)', ...
    'End (GT)', ...
    'Start (EKF)', ...
    'End (EKF)'
}, 'Location', 'best');

% Save figure
saveas(gcf, 'task4_dataset/ekf_pose_comparison.png');

%% Plot 2: L2 Norm Error Over Time (Cleaned)
min_len = min([size(gt_pose,1), size(noisy_pose,1), size(mu_history,2)]);

l2_noisy = vecnorm(gt_pose(1:min_len,1:2)' - noisy_pose(1:min_len,1:2)');
l2_ekf   = vecnorm(gt_pose(1:min_len,1:2)' - mu_history(1:2,1:min_len));

figure(2); clf; hold on;

plot(l2_noisy, 'r--', 'LineWidth', 1.2);  % Noisy vs GT
plot(l2_ekf,   'b-',  'LineWidth', 1.5);  % EKF vs GT

xlabel('Time Step');
ylabel('L2 Norm Error (m)');
title('L2 Pose Error Over Time: Noisy vs EKF');
legend({'Noisy vs Ground Truth', 'EKF vs Ground Truth'}, 'Location', 'best');
grid on;

saveas(gcf, 'task4_dataset/ekf_l2_errors.png');
