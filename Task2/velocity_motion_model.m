function x_new = velocity_motion_model(u, dt, x_prev, alpha)
% velocity_motion_model - Predicts next robot pose using velocity motion model with noise.
%
% This function applies a probabilistic motion model to simulate odometry drift
% based on control inputs and noise parameters.
%
% Inputs:
%   u      - [2x1] control input [v; w] (linear and angular velocity)
%   dt     - Scalar time step (seconds)
%   x_prev - [3x1] previous robot pose [x; y; theta]
%   alpha  - [1x6] motion noise parameters [α1–α6]
%
% Output:
%   x_new  - [3x1] predicted noisy pose [x; y; theta]

    % Validate input dimensions (defensive programming)
    assert(length(u) == 2 && length(x_prev) == 3 && length(alpha) == 6, ...
        'Inputs must be of correct dimension.');

    % Extract control inputs and current orientation
    v = u(1);             % Linear velocity
    w = u(2);             % Angular velocity
    theta = x_prev(3);    % Current heading

    % Unpack alpha noise parameters
    a1 = alpha(1); a2 = alpha(2); a3 = alpha(3);
    a4 = alpha(4); a5 = alpha(5); a6 = alpha(6);

    % Apply zero-mean Gaussian noise to velocities
    v_hat      = v + sample_norm(a1*v^2 + a2*w^2);
    w_hat      = w + sample_norm(a3*v^2 + a4*w^2);
    gamma_hat  = sample_norm(a5*v^2 + a6*w^2);  % Angular noise (drift)

    % Initialize output pose
    x_new = zeros(3,1);

    % Motion update: depends on whether turning or moving straight
    if abs(w_hat) > 1e-6
        % Robot is turning (circular motion)
        x_new(1) = x_prev(1) - (v_hat/w_hat) * sin(theta) + (v_hat/w_hat) * sin(theta + w_hat*dt);
        x_new(2) = x_prev(2) + (v_hat/w_hat) * cos(theta) - (v_hat/w_hat) * cos(theta + w_hat*dt);
    else
        % Robot is moving straight (w ≈ 0)
        x_new(1) = x_prev(1) + v_hat * dt * cos(theta);
        x_new(2) = x_prev(2) + v_hat * dt * sin(theta);
    end

    % Update heading with noisy turn and wrap to [-pi, pi]
    x_new(3) = wrapToPi(theta + w_hat * dt + gamma_hat * dt);
end

%% ------------------------------------------------------------------------
function x = sample_norm(variance)
% sample_norm - Approximates a zero-mean Gaussian sample with given variance.
%
% Uses the Thrun approximation (sum of uniform samples) to simulate Gaussian noise.
%
% Input:
%   variance - Desired variance of the distribution
% Output:
%   x        - Sample from N(0, variance)

    stddev = sqrt(abs(variance));  % Avoid imaginary values if variance is small or negative
    x = 0.5 * sum(2 * stddev * (rand(1,12) - 0.5));  % Approximate Gaussian
end
