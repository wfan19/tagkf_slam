function states_next = predict_state_transition(states, u, ~, dt)
% Function input:
%   states_now: states x 1 column vector of current states
%   u: 6 x 1 column vector of IMU measurements
%
% This is an implementation of the quaternion based EKF predict functions
% found in the following two papers:
% For a general overview, and the world-frame position equation:
% https://arxiv.org/pdf/1606.05285.pdf
%
% For specifically integrating Apriltags and camera extrinsics, see here:
% https://arxiv.org/pdf/1507.02081.pdf

% Vector conventions:
%{

State vector:
[r_wb, v_b, q_wb, b_f, b_omega, r_bv, q_vb, r_T1, q_T1, r_T2, q_T2, ...]

Where:
- r_wb [x, y, z] is position of the body in the world frame, from the world to the body
- v_b [x, y, z] is velocity of the body in the body frame (Need to verify frame)
- q_wb [w, x, y, z] is the rotation from the body frame to the world frame
- b_f [x, y, z] is gyro linear acceleration bias
- b_omega [x, y, z] is the gyro angular rate bias
- r_bv [x, y, z] is the estimated extrinsic positional offset from the IMU to the camera
- q_vb [w, x, y, z] is the estimated extrinsic orientation offset from the body to the camera
- r_v-Ti [x, y, z] is the i-th tag position in the camera frame
- q_Ti-v [w, x, y, z] is the i-th tag quaternion in the camera frame


Input vector:
[f, omega]

Where:
- f [x, y, z] is linear acceleration of the body in the body frame
- omega [x, y, z] is angular velocity of the body in the body frame

%}

%% Fetch state and input vectors
% TODO: figure out a better way to do this (wrap in a function call?)
% States:
posn_body = states(1:3);
vel_body = states(4:6);
quat_body = quaternion(states(7:10)');

% bias_f = states(11:13);
% bias_omega = states(14:16);

bias_f = 0;
bias_omega = 0;

% Inputs:
f_meas = u(1:3);
omega_meas = u(4:6);

%% Preprocess data
% Correct for bias and noise in measurements
f_corrected = f_meas - bias_f; % TODO: Subtract w term
omega_corrected = omega_meas - bias_omega; % TODO: Subtract w term

% Create skew symmetric matrix of omega
% - mat_omega * v is equivalent to cross(omega, v)
% - mat_omega is an element of the lie algebra of SO(3)
% See https://arxiv.org/pdf/1812.01537.pdf for more on skew symmetric
% matrices and how they're a lie algebra of SO(3)
mat_omega = mat_skew_sym(omega_corrected);

%% Calculate state transitions
% Predict next position
posn_body_next = posn_body + transpose(dt*(rotatepoint(quat_body, vel_body(:)'))); % TODO: Add w term

% Predict next velocity
g_rotated = transpose(rotatepoint(quat_body, [0, 0, -1]));
vel_body_next = vel_body + ...
                dt*(g_rotated + f_corrected - mat_omega*vel_body);

% Predict next orientation
% The 
quat_body_next = quat_body * quaternion(rotm2quat(exp(dt*mat_omega)));
v_quat_body_next = compact(quat_body_next);

%% Set output state vector
states_next = zeros(size(states));
states_next(1:3) = posn_body_next(:);
states_next(4:6) = vel_body_next(:);
states_next(7:10) = v_quat_body_next(:);

end

