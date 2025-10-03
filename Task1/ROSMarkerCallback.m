function ROSMarkerCallback(~, msg)
% ROSMarkerCallback - Callback for receiving ArUco marker detections.
%
% This function processes marker data published on the /aruco_marker_publisher/markers
% topic and extracts range and bearing measurements for each detected marker ID.
% It updates a global matrix for use in mapping and localisation.
%
% Inputs:
%   msg - ROS MarkerArray message containing marker poses and IDs
%
% Outputs:
%   Updates global variable:
%     GL_markers [2 x N] = [range; bearing] for each marker ID (up to max_tags)

    persistent lastUpdateTime;

    % Limit callback rate to ~10Hz (optional throttling)
    if isempty(lastUpdateTime)
        lastUpdateTime = tic;
    end
    if toc(lastUpdateTime) < 0.1
        return;  % Skip this frame if it's too soon
    end
    lastUpdateTime = tic;

    global GL_markers;
    max_tags = 9;                  % Max number of unique ArUco IDs expected
    GL_markers = zeros(2, max_tags);  % Reset marker data

    % Process each detected marker in the message
    for i = 1:length(msg.Markers)
        tag = msg.Markers(i);

        % Check that the marker has a valid ID
        if isfield(tag, 'Id')
            tag_index = double(tag.Id);

            if tag_index <= max_tags
                % Extract marker's relative pose
                x = tag.Pose.Pose.Position.X;
                y = tag.Pose.Pose.Position.Y;

                % Convert to polar coordinates (range and bearing)
                r = hypot(x, y);         % Euclidean distance
                b = atan2(y, x);         % Angle to marker

                % Store in global variable
                GL_markers(:, tag_index) = [r; b];
            end
        else
            warning('Tag does not have an "Id" field.');
        end
    end
end
