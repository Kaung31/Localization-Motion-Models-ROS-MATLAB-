%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Task 3 - Data Logging for Mapping and Localisation
%
% Description:
% Runs the robot using the reactive controller from Task 1 and logs all
% required data for occupancy grid mapping and future localisation (EKF/PF).
%
% Logs:
% - Ground truth pose (from /odom)
% - Velocity commands sent
% - Lidar readings (front, left, right)
% - ArUco marker observations (range, bearing per ID)
%
% Output files saved into: /task3_dataset/
%
% Author: Kaung Min Khant
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

clear; clc;

global GL_robot_pose GL_ranges GL_markers;

n = 6500;                      % Total iterations (~3 mins @ 30Hz)
sensor_max = 3.5;              % Maximum usable Lidar range (meters)

% Data storage arrays
robot_pose  = zeros(3, n);     % Ground truth pose: [x; y; theta]
ranges      = zeros(3, n);     % Lidar readings: [front; left; right]
vel_sent    = zeros(2, n);     % Velocity commands sent: [linear; angular]
time_log    = zeros(1, n);     % Execution time per loop
aruco_data  = zeros(2, 9, n);   % Marker data [range; bearing] x 9 markers

% Reset simulation
[resetclient, resetmsg] = rossvcclient('gazebo/reset_world');
resetclient.call(resetmsg);
pause(1);

% ROS setup
get_scan = rossubscriber("/scan", "sensor_msgs/LaserScan", @ROSRangeCallback, "DataFormat", "struct");
get_odom = rossubscriber("/odom", "nav_msgs/Odometry", @ROSPoseCallback, "DataFormat", "struct");
get_aruco = rossubscriber("/aruco_marker_publisher/markers", "aruco_msgs/MarkerArray", @ROSMarkerCallback, "DataFormat", "struct");
[velcmd, vel] = rospublisher("/cmd_vel", "geometry_msgs/Twist", "DataFormat", "struct");

% Wait for sensor callback
disp('Waiting for Lidar...');
while isempty(GL_ranges)
    pause(0.1);
end
disp('Lidar ready.');

% Control logic setup
turning = false;
turn_counter = 0;

% Main control loop
for i = 1:n
    tic;
    pause(0.03);  % Control rate ~30Hz

    % Ensure Lidar has valid data
    if isempty(GL_ranges)
        continue;
    end

    % Clamp noisy readings
    front = min(GL_ranges(1), sensor_max);
    left  = min(GL_ranges(2), sensor_max);
    right = min(GL_ranges(3), sensor_max);

    % Log current data
    ranges(:, i) = [front; left; right];
    robot_pose(:, i) = GL_robot_pose;

    % Obstacle avoidance logic    
    if turning
        vel.Linear.X = 0.05;
        vel.Angular.Z = vel.Angular.Z;
        turn_counter = turn_counter - 1;
        if turn_counter <= 0
            turning = false;
        end
    elseif front < 0.9
        % Obstacle detected
        if left > right
            vel.Angular.Z = 0.8;
        else
            vel.Angular.Z = -0.8;
        end
        vel.Linear.X = 0.05;
        turning = true;
        turn_counter = 20;
    else
        % Clear path ahead
        vel.Linear.X = 0.2;
        vel.Angular.Z = 0.0;
    end

    % Send command and log
    vel_sent(:, i) = [vel.Linear.X; vel.Angular.Z];
    send(velcmd, vel);
    time_log(i) = toc;
    
    % Save ArUco marker readings
    if ~isempty(GL_markers)
        aruco_data(:, :, i) = GL_markers;
    end

    % Print debug info every 100 iterations
    if mod(i, 100) == 0
        fprintf('[%04d] Front=%.2f | Left=%.2f | Right=%.2f\n', i, front, left, right);
    end
end

% Stop robot
vel.Linear.X = 0;
vel.Angular.Z = 0;
send(velcmd, vel);

%% Save Dataset 
if ~exist('task3_dataset', 'dir')
    mkdir('task3_dataset');
end

% Save required dataset files
save('task3_dataset/ground_truth.dat', 'robot_pose', '-ascii');
save('task3_dataset/ranges.dat', 'ranges', '-ascii');
save('task3_dataset/motor_commands.dat', 'vel_sent', '-ascii');

% Convert 3D ArUco matrix to flat 2D [n x 18]
aruco_flat = zeros(n, 18);
for i = 1:n
    aruco_flat(i, :) = reshape(aruco_data(:,:,i), 1, []);
end
save('task3_dataset/aruco_measurements.dat', 'aruco_flat', '-ascii');

% Optional: also save as .mat for EKF/PF use
save('task3_dataset/robot_pose.mat', 'robot_pose');
save('task3_dataset/vel_sent.mat', 'vel_sent', 'time_log');
save('task3_dataset/ranges.mat', 'ranges');

disp('Task 3 complete: all data saved in task3_dataset_2.');

%% Path Plot 
figure;
plot(robot_pose(1,:), robot_pose(2,:), 'b-', 'LineWidth', 2); hold on;
plot(robot_pose(1,1), robot_pose(2,1), 'go', 'MarkerSize', 10, 'DisplayName', 'Start');
plot(robot_pose(1,end), robot_pose(2,end), 'rx', 'MarkerSize', 10, 'LineWidth', 2, 'DisplayName', 'End');
grid on; axis equal;
xlabel('X Position (m)');
ylabel('Y Position (m)');
title('Robot Ground Truth Path');
legend('Path', 'Start', 'End');

