function v_meas = measurement_func(v_state, ~, tag_size, mat_camera)
%% Measurement Function
% The measurement function (Usually denoted linearly as matrices H or C,
% nonlinearly as h(x, v)) is a function that maps your current states, to
% what the way you are measuring things. 
%
% Say you are measuring the angle of
% a pendulum, but your encoder is geared 2:1 to the pendulum. Thus, your
% measurement readings are 2x the actual state variable. To perform your
% kalman filter update step, you need to calculate the error between your
% measurement (the doubled state variable), and the predicted state
% variable. This is when you need the measurement function to take the
% predicted state, and say "what is the predicted measurement for this
% state"
%
% In our case, while the actual states are poses of the body & tags, the
% measurements we are making (Apriltag detections) are coordinates of the
% Apriltag corners. Thus, we need to take our predicted state (poses of
% the body & tags), and map those to theoretical tag coordinates. This can
% be done through multiplying by the camera intrinsic matrix.

%{
Tag-corner Conventions:
- For the tag corner ordering, we start at the bottom left, and go around
counterclockwise
- Tag corner indexing starts at 0

Naming Conventions:
- r_T-Cj: Position vector from tag origin to j-th corner
- r_v-Ti: Position vector from camera to i-th tag
- q_Ti-v: Quaternion rotation from camera to i-th tag
%}

%% Convert vector to struct
state = state_vec_to_struct(v_state);

%% Camera corners
mat_corners = [-1, -1, 0;
                1, -1, 0;
                1,  1, 0;
               -1,  1, 0];

% Calculate positions of tag corners in the tag frame
r_TCj = mat_corners .* tag_size; % tag_size is in meters

points = mat_camera * (state.posn_tag + ...
            rotatepoint(conj(state.quat_tag), r_TCj)');

% "Normalize" the "z" in the camera matrix outputs
% Each row is a single "point" in pixel space, so we "normalize" each row on
% its own
for i_row = 1 : size(points, 1)
    points(i_row, :) = points(i_row, :) ./ points(i_row, 3); 
end

% Reshape into column vector for output
v_meas = reshape(points, numel(points), 1);

end

