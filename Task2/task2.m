%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Task 2 - Velocity Motion Model with Noise
%
% Description:
% Applies a probabilistic velocity motion model to generate a noisy pose
% estimate of the robot based on velocity commands from Task 1.
%
% Functionality:
% - Loads ground truth pose and velocity data
% - Applies motion model using 6 noise parameters (α1 to α6)
% - Simulates dead reckoning with motion noise over time (fixed 30Hz rate)
% - Saves noisy pose data and comparison plot for evaluation
%
% Author: Kaung Min Khant 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

clear; clc;

% Load ground truth and velocity command data from Task 1
base_path = '/home/student/Downloads/Assignment/Task3/task3_dataset/';
robot_pose = load(fullfile(base_path, 'ground_truth.dat'));  % [3 x N]
load(fullfile(base_path, 'vel_sent.mat'));  % vel_sent [2 x N], time_log [1 x N]

% Define noise parameters for motion model
alpha = [0.1, 0.02, 0.3, 0.01, 0.001, 0.01];

% Initialise pose estimate
n = size(vel_sent, 2);
x_est = zeros(3, n);
x_est(:,1) = robot_pose(:,1);  % Start from initial ground truth

% Simulate noisy pose using velocity motion model
for i = 2:n
    dt = 0.033;  % Fixed timestep (approx. 30Hz)
    u = vel_sent(:,i-1);  % Velocity command at previous step
    x_est(:,i) = velocity_motion_model(u, dt, x_est(:,i-1), alpha);
end

% Display end-point comparison for verification
fprintf("Pose difference (start to end): %.4f m\n", norm(x_est(1:2,end) - x_est(1:2,1)));
disp(['Final GT pose: ', num2str(robot_pose(:,end)')]);
disp(['Final noisy pose: ', num2str(x_est(:,end)')]);

%% Plot Pose Comparison 
figure;
plot(robot_pose(1,:), robot_pose(2,:), 'b-', 'LineWidth', 2); hold on;
plot(x_est(1,:), x_est(2,:), 'm--o', 'LineWidth', 1.5);  % Dashed magenta line
legend('Ground Truth Path', 'Noisy Pose Estimate');
xlabel('X (m)'); ylabel('Y (m)');
title('Ground Truth vs Noisy Pose (Velocity Motion Model)');
axis equal; grid on;

%% Save Outputs 
% Create output directory if needed
if ~exist('task2', 'dir'); mkdir('task2'); end

% Save as .mat
save('task2/motion_model_pose.mat', 'x_est');

% Save figure
saveas(gcf, 'task2/task2_pose_comparison.png');

% Save pose as plain text (for use in later tasks)
fileID = fopen('task2/noisy_pose.dat', 'w');
fprintf(fileID, '%f %f %f\n', x_est);
fclose(fileID);

disp('Task 2 complete: Results saved in /task2.');
