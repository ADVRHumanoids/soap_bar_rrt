close all
% clear all

% load
% load('ros_msg_parser__0_2021_09_20__15_02_13.mat')

%% Remove first header.stamp
cartesian_force_estimation_l_ball_tip.header_stamp = cartesian_force_estimation_l_ball_tip.header_stamp - xbotcore_ft_r_leg_ft.header_stamp(1);
cartesian_force_estimation_r_ball_tip.header_stamp = cartesian_force_estimation_r_ball_tip.header_stamp - xbotcore_ft_r_leg_ft.header_stamp(1);
force_opt_force_l_ball_tip_reference.header_stamp = force_opt_force_l_ball_tip_reference.header_stamp - xbotcore_ft_r_leg_ft.header_stamp(1);
force_opt_force_r_ball_tip_reference.header_stamp = force_opt_force_r_ball_tip_reference.header_stamp - xbotcore_ft_r_leg_ft.header_stamp(1);
force_opt_force_l_sole_reference.header_stamp = force_opt_force_l_sole_value.header_stamp - xbotcore_ft_l_leg_ft.header_stamp(1);
force_opt_force_r_sole_reference.header_stamp = force_opt_force_r_sole_value.header_stamp - xbotcore_ft_r_leg_ft.header_stamp(1);
force_opt_force_l_ball_tip_value.header_stamp = force_opt_force_l_ball_tip_value.header_stamp - xbotcore_ft_r_leg_ft.header_stamp(1);
force_opt_force_r_ball_tip_value.header_stamp = force_opt_force_r_ball_tip_value.header_stamp - xbotcore_ft_r_leg_ft.header_stamp(1);
force_opt_force_l_sole_value.header_stamp = force_opt_force_l_sole_value.header_stamp - xbotcore_ft_l_leg_ft.header_stamp(1);
force_opt_force_r_sole_value.header_stamp = force_opt_force_r_sole_value.header_stamp - xbotcore_ft_r_leg_ft.header_stamp(1);
force_opt_pose_l_sole.header_stamp = force_opt_pose_l_sole.header_stamp -xbotcore_ft_l_leg_ft.header_stamp(1);
force_opt_pose_r_sole.header_stamp = force_opt_pose_r_sole.header_stamp - xbotcore_ft_r_leg_ft.header_stamp(1);
force_opt_pose_l_ball_tip.header_stamp = force_opt_pose_l_ball_tip.header_stamp - xbotcore_ft_r_leg_ft.header_stamp(1);
force_opt_pose_r_ball_tip.header_stamp = force_opt_pose_r_ball_tip.header_stamp - xbotcore_ft_r_leg_ft.header_stamp(1);
xbotcore_ft_l_leg_ft.header_stamp = xbotcore_ft_l_leg_ft.header_stamp - xbotcore_ft_l_leg_ft.header_stamp(1);
xbotcore_ft_r_leg_ft.header_stamp = xbotcore_ft_r_leg_ft.header_stamp - xbotcore_ft_r_leg_ft.header_stamp(1);

%% transform from local to global 'world' frame

% Change sensor data sign
xbotcore_ft_l_leg_ft.wrench_force_x = -xbotcore_ft_l_leg_ft.wrench_force_x;
xbotcore_ft_l_leg_ft.wrench_force_y = -xbotcore_ft_l_leg_ft.wrench_force_y;
xbotcore_ft_l_leg_ft.wrench_force_z = -xbotcore_ft_l_leg_ft.wrench_force_z;

xbotcore_ft_r_leg_ft.wrench_force_x = -xbotcore_ft_r_leg_ft.wrench_force_x;
xbotcore_ft_r_leg_ft.wrench_force_y = -xbotcore_ft_r_leg_ft.wrench_force_y;
xbotcore_ft_r_leg_ft.wrench_force_z = -xbotcore_ft_r_leg_ft.wrench_force_z;

% Interpolate onto the same dt
force_opt_pose_l_sole.pose_orientation_x = interp1(force_opt_pose_l_sole.header_stamp, force_opt_pose_l_sole.pose_orientation_x, xbotcore_ft_l_leg_ft.header_stamp);
force_opt_pose_l_sole.pose_orientation_y = interp1(force_opt_pose_l_sole.header_stamp, force_opt_pose_l_sole.pose_orientation_y, xbotcore_ft_l_leg_ft.header_stamp);
force_opt_pose_l_sole.pose_orientation_z = interp1(force_opt_pose_l_sole.header_stamp, force_opt_pose_l_sole.pose_orientation_z, xbotcore_ft_l_leg_ft.header_stamp);
force_opt_pose_l_sole.pose_orientation_w = interp1(force_opt_pose_l_sole.header_stamp, force_opt_pose_l_sole.pose_orientation_w, xbotcore_ft_l_leg_ft.header_stamp);

force_opt_pose_r_sole.pose_orientation_x = interp1(force_opt_pose_r_sole.header_stamp, force_opt_pose_r_sole.pose_orientation_x, xbotcore_ft_r_leg_ft.header_stamp);
force_opt_pose_r_sole.pose_orientation_y = interp1(force_opt_pose_r_sole.header_stamp, force_opt_pose_r_sole.pose_orientation_y, xbotcore_ft_r_leg_ft.header_stamp);
force_opt_pose_r_sole.pose_orientation_z = interp1(force_opt_pose_r_sole.header_stamp, force_opt_pose_r_sole.pose_orientation_z, xbotcore_ft_r_leg_ft.header_stamp);
force_opt_pose_r_sole.pose_orientation_w = interp1(force_opt_pose_r_sole.header_stamp, force_opt_pose_r_sole.pose_orientation_w, xbotcore_ft_r_leg_ft.header_stamp);

force_opt_pose_l_ball_tip.pose_orientation_x = interp1(force_opt_pose_l_ball_tip.header_stamp, force_opt_pose_l_ball_tip.pose_orientation_x, xbotcore_ft_l_leg_ft.header_stamp);
force_opt_pose_l_ball_tip.pose_orientation_y = interp1(force_opt_pose_l_ball_tip.header_stamp, force_opt_pose_l_ball_tip.pose_orientation_y, xbotcore_ft_l_leg_ft.header_stamp);
force_opt_pose_l_ball_tip.pose_orientation_z = interp1(force_opt_pose_l_ball_tip.header_stamp, force_opt_pose_l_ball_tip.pose_orientation_z, xbotcore_ft_l_leg_ft.header_stamp);
force_opt_pose_l_ball_tip.pose_orientation_w = interp1(force_opt_pose_l_ball_tip.header_stamp, force_opt_pose_l_ball_tip.pose_orientation_w, xbotcore_ft_l_leg_ft.header_stamp);

force_opt_pose_r_ball_tip.pose_orientation_x = interp1(force_opt_pose_r_ball_tip.header_stamp, force_opt_pose_r_ball_tip.pose_orientation_x, xbotcore_ft_r_leg_ft.header_stamp);
force_opt_pose_r_ball_tip.pose_orientation_y = interp1(force_opt_pose_r_ball_tip.header_stamp, force_opt_pose_r_ball_tip.pose_orientation_y, xbotcore_ft_r_leg_ft.header_stamp);
force_opt_pose_r_ball_tip.pose_orientation_z = interp1(force_opt_pose_r_ball_tip.header_stamp, force_opt_pose_r_ball_tip.pose_orientation_z, xbotcore_ft_r_leg_ft.header_stamp);
force_opt_pose_r_ball_tip.pose_orientation_w = interp1(force_opt_pose_r_ball_tip.header_stamp, force_opt_pose_r_ball_tip.pose_orientation_w, xbotcore_ft_r_leg_ft.header_stamp);

cartesian_force_estimation_l_ball_tip.wrench_force_x = interp1(cartesian_force_estimation_l_ball_tip.header_stamp, cartesian_force_estimation_l_ball_tip.wrench_force_x, xbotcore_ft_l_leg_ft.header_stamp);
cartesian_force_estimation_l_ball_tip.wrench_force_y = interp1(cartesian_force_estimation_l_ball_tip.header_stamp, cartesian_force_estimation_l_ball_tip.wrench_force_y, xbotcore_ft_l_leg_ft.header_stamp);
cartesian_force_estimation_l_ball_tip.wrench_force_z = interp1(cartesian_force_estimation_l_ball_tip.header_stamp, cartesian_force_estimation_l_ball_tip.wrench_force_z, xbotcore_ft_l_leg_ft.header_stamp);

cartesian_force_estimation_r_ball_tip.wrench_force_x = interp1(cartesian_force_estimation_r_ball_tip.header_stamp, cartesian_force_estimation_r_ball_tip.wrench_force_x, xbotcore_ft_r_leg_ft.header_stamp);
cartesian_force_estimation_r_ball_tip.wrench_force_y = interp1(cartesian_force_estimation_r_ball_tip.header_stamp, cartesian_force_estimation_r_ball_tip.wrench_force_y, xbotcore_ft_r_leg_ft.header_stamp);
cartesian_force_estimation_r_ball_tip.wrench_force_z = interp1(cartesian_force_estimation_r_ball_tip.header_stamp, cartesian_force_estimation_r_ball_tip.wrench_force_z, xbotcore_ft_r_leg_ft.header_stamp);

force_opt_force_l_ball_tip_value.wrench_force_x = interp1(force_opt_force_l_ball_tip_value.header_stamp, force_opt_force_l_ball_tip_value.wrench_force_x, xbotcore_ft_l_leg_ft.header_stamp);
force_opt_force_l_ball_tip_value.wrench_force_y = interp1(force_opt_force_l_ball_tip_value.header_stamp, force_opt_force_l_ball_tip_value.wrench_force_y, xbotcore_ft_l_leg_ft.header_stamp);
force_opt_force_l_ball_tip_value.wrench_force_z = interp1(force_opt_force_l_ball_tip_value.header_stamp, force_opt_force_l_ball_tip_value.wrench_force_z, xbotcore_ft_l_leg_ft.header_stamp);

% Find rotation matrices
for i = [1 : length(force_opt_pose_l_sole.pose_orientation_x)]
    quat = [force_opt_pose_l_sole.pose_orientation_w(i), ...
            force_opt_pose_l_sole.pose_orientation_x(i), ...
            force_opt_pose_l_sole.pose_orientation_y(i), ...
            force_opt_pose_l_sole.pose_orientation_z(i)];
    if sum(isnan(quat)) > 0
        quat = [0 0 0 1];
    end
    rot = quat2rotm(quat);
    force_opt_pose_l_sole.rot(:,:,i) = rot(:,:);
end

for i = [1 : length(force_opt_pose_r_sole.pose_orientation_x)]
    quat = [force_opt_pose_r_sole.pose_orientation_w(i), ...
            force_opt_pose_r_sole.pose_orientation_x(i), ...
            force_opt_pose_r_sole.pose_orientation_y(i), ...
            force_opt_pose_r_sole.pose_orientation_z(i)];
    if sum(isnan(quat)) > 0
        quat = [0 0 0 1];
    end
    rot = quat2rotm(quat);
    force_opt_pose_r_sole.rot(:,:,i) = rot(:,:);
end

for i = [1 : length(force_opt_pose_l_ball_tip.pose_orientation_x)]
    quat = [force_opt_pose_l_ball_tip.pose_orientation_w(i), ...
            force_opt_pose_l_ball_tip.pose_orientation_x(i), ...
            force_opt_pose_l_ball_tip.pose_orientation_y(i), ...
            force_opt_pose_l_ball_tip.pose_orientation_z(i)];
    if sum(isnan(quat)) > 0
        quat = [0 0 0 1];
    end
    rot = quat2rotm(quat);
    force_opt_pose_l_ball_tip.rot(:,:,i) = rot(:,:);
end

for i = [1 : length(force_opt_pose_r_ball_tip.pose_orientation_x)]
    quat = [force_opt_pose_r_ball_tip.pose_orientation_w(i), ...
            force_opt_pose_r_ball_tip.pose_orientation_x(i), ...
            force_opt_pose_r_ball_tip.pose_orientation_y(i), ...
            force_opt_pose_r_ball_tip.pose_orientation_z(i)];
    if sum(isnan(quat)) > 0
        quat = [0 0 0 1];
    end
    rot = quat2rotm(quat);
    force_opt_pose_r_ball_tip.rot(:,:,i) = rot(:,:);
end

% Apply rotation
for i = [1 : length(force_opt_pose_l_sole.pose_orientation_x)]
    rot = force_opt_pose_l_sole.rot(:,:,i);
    force = [xbotcore_ft_l_leg_ft.wrench_force_x(i);
             xbotcore_ft_l_leg_ft.wrench_force_y(i);
             xbotcore_ft_l_leg_ft.wrench_force_z(i)];     
    force = inv(rot) * force;
    xbotcore_ft_l_leg_ft.wrench_force_x(i) = force(1);
    xbotcore_ft_l_leg_ft.wrench_force_y(i) = force(2);
    xbotcore_ft_l_leg_ft.wrench_force_z(i) = force(3);
end

for i = [1 : length(force_opt_pose_r_sole.pose_orientation_x)]
    rot = force_opt_pose_r_sole.rot(:,:,i);
    force = [xbotcore_ft_r_leg_ft.wrench_force_x(i);
             xbotcore_ft_r_leg_ft.wrench_force_y(i);
             xbotcore_ft_r_leg_ft.wrench_force_z(i)];     
    force = inv(rot) * force;
    xbotcore_ft_r_leg_ft.wrench_force_x(i) = force(1);
    xbotcore_ft_r_leg_ft.wrench_force_y(i) = force(2);
    xbotcore_ft_r_leg_ft.wrench_force_z(i) = force(3);
end

for i = [1 : length(force_opt_pose_l_ball_tip.pose_orientation_x)]
    rot = force_opt_pose_l_ball_tip.rot(:,:,i);
    force = [cartesian_force_estimation_l_ball_tip.wrench_force_x(i);
             cartesian_force_estimation_l_ball_tip.wrench_force_y(i);
             cartesian_force_estimation_l_ball_tip.wrench_force_z(i)];     
    force = inv(rot) * force;
    cartesian_force_estimation_l_ball_tip.wrench_force_x(i) = force(1);
    cartesian_force_estimation_l_ball_tip.wrench_force_y(i) = force(2);
    cartesian_force_estimation_l_ball_tip.wrench_force_z(i) = force(3);
end

for i = [1 : length(force_opt_pose_r_ball_tip.pose_orientation_x)]
    rot = force_opt_pose_r_ball_tip.rot(:,:,i);
    force = [cartesian_force_estimation_r_ball_tip.wrench_force_x(i);
             cartesian_force_estimation_r_ball_tip.wrench_force_y(i);
             cartesian_force_estimation_r_ball_tip.wrench_force_z(i)];     
    force = inv(rot) * force;
    cartesian_force_estimation_r_ball_tip.wrench_force_x(i) = force(1);
    cartesian_force_estimation_r_ball_tip.wrench_force_y(i) = force(2);
    cartesian_force_estimation_r_ball_tip.wrench_force_z(i) = force(3);
end

%% Plot
% Left Foot
figure(1) 
hold on
stairs(force_opt_force_l_sole_reference.header_stamp, force_opt_force_l_sole_reference.wrench_force_x, 'r--', 'LineWidth', 1)
stairs(force_opt_force_l_sole_reference.header_stamp, force_opt_force_l_sole_reference.wrench_force_y, 'g--', 'LineWidth', 1)
stairs(force_opt_force_l_sole_reference.header_stamp, force_opt_force_l_sole_reference.wrench_force_z, 'b--', 'LineWidth', 1)
plot(xbotcore_ft_l_leg_ft.header_stamp, xbotcore_ft_l_leg_ft.wrench_force_x, 'r', 'LineWidth', 1)
plot(xbotcore_ft_l_leg_ft.header_stamp, xbotcore_ft_l_leg_ft.wrench_force_y, 'g', 'LineWidth', 1)
plot(xbotcore_ft_l_leg_ft.header_stamp, xbotcore_ft_l_leg_ft.wrench_force_z, 'b', 'LineWidth', 1)

% Right Foot
figure(2) 
hold on
stairs(force_opt_force_r_sole_reference.header_stamp, force_opt_force_r_sole_reference.wrench_force_x, 'r--', 'LineWidth', 1)
stairs(force_opt_force_r_sole_reference.header_stamp, force_opt_force_r_sole_reference.wrench_force_y, 'g--', 'LineWidth', 1)
stairs(force_opt_force_r_sole_reference.header_stamp, force_opt_force_r_sole_reference.wrench_force_z, 'b--', 'LineWidth', 1)
plot(xbotcore_ft_r_leg_ft.header_stamp, xbotcore_ft_r_leg_ft.wrench_force_x, 'r', 'LineWidth', 1)
plot(xbotcore_ft_r_leg_ft.header_stamp, xbotcore_ft_r_leg_ft.wrench_force_y, 'g', 'LineWidth', 1)
plot(xbotcore_ft_r_leg_ft.header_stamp, xbotcore_ft_r_leg_ft.wrench_force_z, 'b', 'LineWidth', 1)

% Left Hand
figure(3) 
hold on
stairs(force_opt_force_l_ball_tip_reference.header_stamp, force_opt_force_l_ball_tip_reference.wrench_force_x, 'r--', 'LineWidth', 1)
stairs(force_opt_force_l_ball_tip_reference.header_stamp, force_opt_force_l_ball_tip_reference.wrench_force_y, 'g--', 'LineWidth', 1)
stairs(force_opt_force_l_ball_tip_reference.header_stamp, force_opt_force_l_ball_tip_reference.wrench_force_z, 'b--', 'LineWidth', 1)
plot(xbotcore_ft_l_leg_ft.header_stamp, cartesian_force_estimation_l_ball_tip.wrench_force_x, 'r', 'LineWidth', 1)
plot(xbotcore_ft_l_leg_ft.header_stamp, cartesian_force_estimation_l_ball_tip.wrench_force_y, 'g', 'LineWidth', 1)
plot(xbotcore_ft_l_leg_ft.header_stamp, cartesian_force_estimation_l_ball_tip.wrench_force_z, 'b', 'LineWidth', 1)

% Right Hand
figure(4) 
hold on
stairs(force_opt_force_r_ball_tip_reference.header_stamp, force_opt_force_r_ball_tip_reference.wrench_force_x, 'r--', 'LineWidth', 1)
stairs(force_opt_force_r_ball_tip_reference.header_stamp, force_opt_force_r_ball_tip_reference.wrench_force_y, 'g--', 'LineWidth', 1)
stairs(force_opt_force_r_ball_tip_reference.header_stamp, force_opt_force_r_ball_tip_reference.wrench_force_z, 'b--', 'LineWidth', 1)
plot(xbotcore_ft_r_leg_ft.header_stamp, cartesian_force_estimation_r_ball_tip.wrench_force_x, 'r', 'LineWidth', 1)
plot(xbotcore_ft_r_leg_ft.header_stamp, cartesian_force_estimation_r_ball_tip.wrench_force_y, 'g', 'LineWidth', 1)
plot(xbotcore_ft_r_leg_ft.header_stamp, cartesian_force_estimation_r_ball_tip.wrench_force_z, 'b', 'LineWidth', 1)