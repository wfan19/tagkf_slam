function v_states_next = predict_state_transition(v_states, ~, v_inputs, dt)
% Function input:
%   states_now: states x 1 column vector of current states
%   u: 6 x 1 column vector of IMU measurements
%
% This is an implementation of the quaternion based EKF predict functions
% found in the following two papers:
% For a general overview, and the world-frame position equation:
% https://arxiv.org/pdf/1606.05285.pdf (Paper 1)
%
% For specifically integrating Apriltags and camera extrinsics, see here:
% https://arxiv.org/pdf/1507.02081.pdf (Paper 2)

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

%% Convert input vectors to structs for sanity
states = state_vec_to_struct(v_states);
inputs = input_vec_to_struct(v_inputs);
states_next = state_vec_to_struct(zeros(length(v_states), 1));

%% Preprocess data
% Correct for bias and noise in measurements
f_corrected = inputs.f - states.bias_f; % TODO: Subtract w term
omega_corrected = inputs.omega - states.bias_omega; % TODO: Subtract w term

% Create skew symmetric matrix of omega
% - mat_omega * v is equivalent to cross(omega, v)
% - mat_omega is an element of the lie algebra of SO(3)
% See https://arxiv.org/pdf/1812.01537.pdf for more on skew symmetricvel_camera = rotatepoint(quat_vb, 
% matrices and how they're a lie algebra of SO(3)
mat_omega = mat_skew_sym(omega_corrected);

%% Calculate state transitions
% Predict next position
states_next.posn_body = states.posn_body + ...
                    transpose(dt*(rotatepoint(states.quat_body, states.vel_body(:)'))); % TODO: Add w term

% Predict next velocity
g_rotated = transpose(rotatepoint(conj(states.quat_body), [0, 0, 0])); % Turns off gravity
%g_rotated = transpose(rotatepoint(conj(quat_body), [0, 0, -9.8])); % Gravity acceleration vector
states_next.vel_body = states.vel_body + ...
            dt*(g_rotated + f_corrected - mat_omega * states.vel_body);

% Predict next orientation
% - See Equation (56) from Paper (1)
% - expm is the matrix exponential / exponential map
% - The exponential map of an angular velocity matrix (the lie algebra) is
%   the corresponding change in 3d rotation (the lie group)
states_next.quat_body = states.quat_body * quaternion(rotm2quat(expm(dt*mat_omega)));

% "Predict" next biases and camera extrinsics
% In this case, it's just adding a noise term
% TODO: Actually add the noise term instead of feeding forward current state
states_next.bias_f = states.bias_f;
states_next.bias_omega = states.bias_omega;

states_next.posn_bv = states.posn_bv;
states_next.quat_vb = states.quat_vb;

% Predict next tag position in the camera frame
% Equation (13) from simplifies to this if you distribute out matrix and
% quaternion multiplications

states_next.posn_tag = states.posn_tag;
mat_omega_camera = mat_skew_sym(rotatepoint(states.quat_vb, omega_corrected(:)')); % Body angular vel in the camera frame
states_next.posn_tag = states.posn_tag + ...
                        dt*(-mat_omega_camera * states.posn_tag - ...
                        rotatepoint(states.quat_vb, (mat_omega * states.posn_bv)' + states.vel_body')');

% Predict next tag orientation in the camera frame
states_next.quat_tag = quaternion([1, 0, 0, 0]);

%% Generate output state vector

v_states_next = state_struct_to_vec(states_next);

end

