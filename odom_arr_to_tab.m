function tab_out = odom_arr_to_tab(arr_odom)
% Turns a cell array of odometry messages into a table, where each row
% represents a single message, and the columns represent fields in the
% message

secs = zeros(length(arr_odom), 1);

posns = zeros(length(arr_odom), 3);
orntns = zeros(length(arr_odom), 4);

vels_lin = zeros(length(arr_odom), 3);
vels_ang = zeros(length(arr_odom), 3);

for i = 1 : length(arr_odom)
    current_odom = arr_odom{i};
    
    secs(i, 1) = current_odom.Header.Stamp.Sec + current_odom.Header.Stamp.Nsec * 1e-9;
    
    position = current_odom.Pose.Pose.Position;
    posns(i, :) = [position.X, position.Y, position.Z];
    
    orientation = current_odom.Pose.Pose.Orientation;
    orntns(i, :) = [orientation.W, orientation.X, orientation.Y, orientation.Z];
    
    linear_vel = current_odom.Twist.Twist.Linear;
    vels_lin(i, :) = [linear_vel.X, linear_vel.Y, linear_vel.Z];
    
    angular_vel = current_odom.Twist.Twist.Angular;
    vels_ang(i, :) = [angular_vel.X, angular_vel.Y, angular_vel.Z];
end

tab_out = table(secs, posns, orntns, vels_lin, vels_ang);

end

