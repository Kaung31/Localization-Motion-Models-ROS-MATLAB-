function ROSPoseCallback(~, message)
%% A call back function that is called when a message arrives on the /odom topic 
    % Use global variable to store position and orientation
    global GL_robot_pose
    % Extract 2D position and theta orientation from the ROS message and assign the
    % data to the global variable. rosmsg show geometry_msgs/Pose Point Position  {X Y Z}
    % Quaternion Orientation {X Y Z W}
    % convert quaternion orientation to Euler angles (alpha, beta, gamma) 
    quant = [message.Pose.Pose.Orientation.X message.Pose.Pose.Orientation.Y message.Pose.Pose.Orientation.Z message.Pose.Pose.Orientation.W];
    eula = quat2eul(quant);
    % only pass back rotation around Z axis (gamma euler angle) that we
    % refer to as theta
    GL_robot_pose = [message.Pose.Pose.Position.X message.Pose.Pose.Position.Y eula(3)];
end