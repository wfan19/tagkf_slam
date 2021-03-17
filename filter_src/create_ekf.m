function obj_ekf = create_ekf()
% This function generates an EKF object for localizing using a mix of
% Apriltag detections and IMU data.

% State vector conventions:
%{

State vector:
[r_b, v_b, q_b, b_f, b_omega, r_v, q_v, r_T1, q_T1, r_T2, q_T2, ...]

Where:
- r_b [x, y, z] is position of the body in the world frame
- v_b [x, y, z] is velocity of the body in the body frame (Need to verify frame)
- q_b [w, x, y, z] is the orientation of the body in the world frame
- b_f [x, y, z] is gyro linear acceleration bias
- b_omega [x, y, z] is the gyro angular rate bias
- r_v [x, y, z] is the estimated extrinsic positional offset between camera and IMU
- q_v [w, x, y, z] is the estimated extrinsic orientation offset between camera and IMU
- r_Ti [x, y, z] is the i-th tag position
- q_Ti [w, x, y, z] is the i-th tag quaternion

%}

end