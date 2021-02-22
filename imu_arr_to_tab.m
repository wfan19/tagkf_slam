function tab_imu = imu_arr_to_tab(arr_imu)
secs = zeros(length(arr_imu), 1);
orntns = zeros(length(arr_imu), 4);
accels_lin = zeros(length(arr_imu), 3);
vels_ang = zeros(length(arr_imu), 3);

covs_orntn = zeros(length(arr_imu), 9);
covs_lin = zeros(length(arr_imu), 9);
covs_ang = zeros(length(arr_imu), 9);

for i = 1 : length(arr_imu)
    imu_msg = arr_imu{i};
    secs(i) = imu_msg.Header.Stamp.Sec + imu_msg.Header.Stamp.Nsec*1e-9;
    
    orientation = imu_msg.Orientation;
    orntns(i, :) = [orientation.W, orientation.X, orientation.Y, orientation.Z];
    
    linear_accel = imu_msg.LinearAcceleration;
    accels_lin(i, :) = [linear_accel.X, linear_accel.Y, linear_accel.Z];
    
    angular_vel = imu_msg.AngularVelocity;
    vels_ang(i, :) = [angular_vel.X, angular_vel.Y, angular_vel.Z];
    
    % TODO: save all the other covariances
    covs_orntn(i, :) = imu_msg.OrientationCovariance;
    covs_lin(i, :) = imu_msg.LinearAccelerationCovariance;
    covs_ang(i, :) = imu_msg.AngularVelocityCovariance;
end

tab_imu = table(secs, orntns, accels_lin, vels_ang, covs_orntn, covs_lin, covs_ang);

end

