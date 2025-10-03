function ROSRangeCallback(~, message)
% ROSRangeCallback - Processes Lidar scan data for front, left, and right directions.
%
% This function extracts and filters Lidar scan data received via ROS and stores
% the minimum values at front (0°), left (+90°), and right (−90°) directions into 
% a global variable for real-time obstacle avoidance.
%
% Input:
%   message - ROS LaserScan message structure
%
% Output:
%   Updates global variable GL_ranges with:
%     GL_ranges(1) = front
%     GL_ranges(2) = left
%     GL_ranges(3) = right

    global GL_ranges
    sensor_max = 3.5;  % Max reliable Lidar range

    % Convert to double and replace NaNs
    ranges = double(message.Ranges);
    ranges(isnan(ranges)) = sensor_max;

    % Angular information
    angle_min = message.AngleMin;
    angle_inc = message.AngleIncrement;
    num_ranges = length(ranges);

    % Desired directions (in radians)
    angle_front = 0;
    angle_left  = pi/2;
    angle_right = -pi/2;

    % Convert angles to indices
    idx_front = round((angle_front - angle_min) / angle_inc) + 1;
    idx_left  = round((angle_left  - angle_min) / angle_inc) + 1;
    idx_right = round((angle_right - angle_min) / angle_inc) + 1;

    % Clamp indices to valid range
    idx_front = min(max(idx_front, 1), num_ranges);
    idx_left  = min(max(idx_left,  1), num_ranges);
    idx_right = min(max(idx_right, 1), num_ranges);

    % Use a small window around each direction to smooth values
    win = 5;
    front_idx = max(1, idx_front - win):min(num_ranges, idx_front + win);
    left_idx  = max(1, idx_left  - win):min(num_ranges, idx_left  + win);
    right_idx = max(1, idx_right - win):min(num_ranges, idx_right + win);

    % Store min distance from each direction, fallback to max if empty
    if ~isempty(front_idx)
        GL_ranges(1) = min(ranges(front_idx), [], 'omitnan');
    else
        GL_ranges(1) = sensor_max;
    end

    if ~isempty(left_idx)
        GL_ranges(2) = min(ranges(left_idx), [], 'omitnan');
    else
        GL_ranges(2) = sensor_max;
    end

    if ~isempty(right_idx)
        GL_ranges(3) = min(ranges(right_idx), [], 'omitnan');
    else
        GL_ranges(3) = sensor_max;
    end
end
