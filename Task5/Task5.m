%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Task 5 - Particle Filter for Localisation
% - Uses velocity motion model + ArUco marker observations
% - Estimates robot pose with probabilistic filtering
% - Saves estimated pose and comparison plots
% Author: Kaung Min Khant
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

clear; clc;

%% Load Dataset
% - Ground truth pose from simulation
% - Noisy odometry (from velocity motion model)
% - Motor commands issued by robot
% - Range & bearing observations of ArUco markers
gt_pose    = load('/home/student/Downloads/Assignment/Task3/task3_dataset/ground_truth.dat');
noisy_pose = load('/home/student/Downloads/Assignment/Task3/task3_dataset/noisy_pose.dat');
vels       = load('/home/student/Downloads/Assignment/Task3/task3_dataset/vel_sent.dat')';
z          = load('/home/student/Downloads/Assignment/Task3/task3_dataset/aruco_measurements.dat');

% Landmark positions for 9 ArUco markers (ID 1–9)
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

%% PF Parameters 
num_particles = 500;
alpha = [0.1, 0.02, 0.3, 0.01, 0.001, 0.01]';   % Motion noise
R = diag([0.1, deg2rad(10)]).^2;                    % Measurement noise
dt = 0.03;
n_steps = size(gt_pose, 1);

% Initialise particles near first noisy pose
particles = repmat(noisy_pose(1,:), num_particles, 1);
particles(:,1:2) = particles(:,1:2) + randn(num_particles,2)*0.1;
particles(:,3) = wrapToPi(particles(:,3) + randn(num_particles,1)*0.05);
weights = ones(num_particles,1) / num_particles;

% Storage for final filtered pose estimates
pf_estimates = zeros(3, n_steps);
pf_estimates(:,1) = mean(particles)';

%% Main PF Loop 
for t = 2:n_steps
    u = vels(t-1,:)';         % Control input [v; w]
    z_t = z(t,:);             % Marker measurements at time t

    % Prediction Step 
    for i = 1:num_particles
        x_prev = particles(i,:)';
        x_new = velocity_motion_model(u, dt, x_prev, alpha);
        particles(i,:) = x_new';
    end

    % Measurement Update (Weighting) 
    for i = 1:num_particles
        total_log_prob = 0;
        for j = 1:9
            z_ij = z_t((2*j-1):(2*j));  % [range; bearing] for marker j
            if all(z_ij == 0), continue; end

            % Predict expected observation from particle
            dx = landmarks(j,1) - particles(i,1);
            dy = landmarks(j,2) - particles(i,2);
            q = dx^2 + dy^2;
            z_hat = [sqrt(q); wrapToPi(atan2(dy, dx) - particles(i,3))];

            innovation = z_ij' - z_hat;
            innovation(2) = wrapToPi(innovation(2));

            % Log-likelihood of this observation
            log_prob = -0.5 * innovation' / R * innovation ...
                       - log((2*pi)^2 * sqrt(det(R)));
            total_log_prob = total_log_prob + log_prob;
        end
        weights(i) = exp(total_log_prob);
    end

    % Normalize weights
    weights = weights + 1e-300;
    weights = weights / sum(weights);

    % Store Weighted Estimate BEFORE resampling
    pf_estimates(:,t) = (particles' * weights)';
    
    % Compute Effective Sample Size
    Neff = 1 / sum(weights.^2);
    
    % Resample only if Neff below threshold
    if Neff < num_particles / 2
        indices = systematic_resample(weights);
        particles = particles(indices,:);
        weights = ones(num_particles,1) / num_particles;
    end
end

%% Save Results 
if ~exist('task5_dataset', 'dir'), mkdir('task5_dataset'); end
save('task5_dataset/pf_estimates.dat', 'pf_estimates', '-ascii');

%% Plot 1: Trajectory
figure(1); clf; hold on;
plot(gt_pose(:,1), gt_pose(:,2), 'g-');                     % Ground Truth
plot(noisy_pose(:,1), noisy_pose(:,2), 'r--');              % Noisy Odometry
plot(pf_estimates(1,:), pf_estimates(2,:), 'b-');           % PF Estimate

% Start and End markers
plot(gt_pose(1,1), gt_pose(1,2), 'go', 'MarkerFaceColor','g');   % Start (GT)
plot(gt_pose(end,1), gt_pose(end,2), 'gx', 'MarkerSize',10,'LineWidth',2);  % End (GT)
plot(pf_estimates(1,1), pf_estimates(2,1), 'bo', 'MarkerFaceColor','b');    % Start (PF)
plot(pf_estimates(1,end), pf_estimates(2,end), 'bx', 'MarkerSize',10,'LineWidth',2);  % End (PF)

xlabel('X'); ylabel('Y'); axis equal; grid on;
title('Pose Comparison (GT vs Noisy vs PF)');

% explicit legend
legend({
    'Ground Truth', ...
    'Noisy Odometry', ...
    'PF Estimate', ...
    'Start (GT)', ...
    'End (GT)', ...
    'Start (PF)', ...
    'End (PF)'
}, 'Location', 'best');

saveas(gcf, 'task5_dataset/pf_pose_comparison.png');

%% Plot 2: L2 Error
min_len = min([size(gt_pose,1), size(noisy_pose,1), size(pf_estimates,2)]);
l2_noisy = vecnorm(gt_pose(1:min_len,1:2)' - noisy_pose(1:min_len,1:2)');
l2_pf    = vecnorm(gt_pose(1:min_len,1:2)' - pf_estimates(1:2,1:min_len));

figure(2); clf; hold on;
plot(l2_noisy, 'r--');  % Noisy
plot(l2_pf, 'b-');      % PF

xlabel('Time Step'); ylabel('L2 Norm Error');
title('L2 Pose Error Over Time');

legend({'Noisy vs GT', 'PF vs GT'}, 'Location', 'best');
grid on;

saveas(gcf, 'task5_dataset/pf_l2_errors.png');

function idx = systematic_resample(weights)
    N = length(weights);
    positions = ((0:N-1) + rand) / N;
    idx = zeros(N,1);
    cumulative_sum = cumsum(weights);
    i = 1; j = 1;
    while i <= N
        if positions(i) < cumulative_sum(j)
            idx(i) = j;
            i = i + 1;
        else
            j = j + 1;
        end
    end
end