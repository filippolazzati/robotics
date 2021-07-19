 %%%%%%%%%%%%%%%%%%%%%%%%%Given constants%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%radius:
r = 0.1575;
%real_baseline:
real_baseline = 0.583;
%gear_ratio:
gear_ratio = 0.0262;

%%%%%%%%%%%%%%%%%%%%%%%%%Read the files%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%CSVData
gt_pose = readmatrix('gt_pose.csv');
motor_speed_fl = readmatrix('motor_speed_fl.csv');
motor_speed_fr = readmatrix('motor_speed_fr.csv');
motor_speed_rl = readmatrix('motor_speed_rl.csv');
motor_speed_rr = readmatrix('motor_speed_rr.csv');
%CSVData from our node
%gt_pose = readmatrix('gt_pose_from_our_node.csv');
%motor_speed_fl = readmatrix('motor_speed_fl_from_our_node.csv');
%motor_speed_fre = readmatrix('motor_speed_fr_from_our_node.csv');
%motor_speed_rl = readmatrix('motor_speed_rl_from_our_node.csv');
%motor_speed_rr = readmatrix('motor_speed_rr_from_our_node.csv');

%%%%%%%%%%%%%%%%%%%%%%%%%Data from gt_pose%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%data from gt_pose: (18009 rows)
timestamp_gt_pose = gt_pose(:,3);
position_x = gt_pose(:,5);
position_y = gt_pose(:,6);
position_z = gt_pose(:,7);
%theta_x = gt_pose(:,8);
%theta_y = gt_pose(:,9);
%theta_z = gt_pose(:,10);
theta_w = gt_pose(:,11);

%%%%%%%%%%%%%%%%%%%%%%%%%Data from motors%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%data from motor_speed_fl: (14136 rows)
timestamp_fl = motor_speed_fl(:,3);
w_fl = motor_speed_fl(:,5)  * (-1); %left wheels rotate in opposite direction
%data from motor_speed_fr: (14136 rows)
timestamp_fr = motor_speed_fr(:,3);
w_fr = motor_speed_fr(:,5);
%data from motor_speed_rl: (14136 rows)
timestamp_rl = motor_speed_rl(:,3);
w_rl = motor_speed_rl(:,5)  * (-1); %left wheels rotate in opposite direction
%data from motor_speed_rr: (14136 rows)
timestamp_rr = motor_speed_rr(:,3);
w_rr = motor_speed_rr(:,5);

%convert rpm in rad/s
w_fl_rad = ((w_fl / 60) * 2 * pi) * gear_ratio;
w_fr_rad = ((w_fr / 60) * 2 * pi) * gear_ratio;
w_rl_rad = ((w_rl / 60) * 2 * pi) * gear_ratio;
w_rr_rad = ((w_rr / 60) * 2 * pi) * gear_ratio;

%average angular velocity for tread
w_l = (w_fl_rad + w_rl_rad)/2;
w_r = (w_fr_rad + w_rr_rad)/2;

%%%%%%%%%%%%%%%%%%%%%%%%%%%delta timestamps%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%delta timestamp of gt_pose
delta_timestamp_gt_pose = zeros(18008, 1); %it will have 1 value less than timestamp_gt_pose
for i = 1:18008
    delta_timestamp_gt_pose(i) = timestamp_gt_pose(i+1) - timestamp_gt_pose(i);
end

%delta timestamp of motors
delta_timestamp_motors_fl = zeros(14135, 1); %it will have 1 value less than the motor
delta_timestamp_motors_fr = zeros(14135, 1); %it will have 1 value less than the motor
delta_timestamp_motors_rl = zeros(14135, 1); %it will have 1 value less than the motor
delta_timestamp_motors_rr = zeros(14135, 1); %it will have 1 value less than the motor
for i = 1:14135
    delta_timestamp_motors_fl(i) = timestamp_fl(i+1) - timestamp_fl(i);
    delta_timestamp_motors_fr(i) = timestamp_fr(i+1) - timestamp_fr(i);
    delta_timestamp_motors_rl(i) = timestamp_rl(i+1) - timestamp_rl(i);
    delta_timestamp_motors_rr(i) = timestamp_rr(i+1) - timestamp_rr(i);
end
delta_timestamp_motors = (delta_timestamp_motors_fl + delta_timestamp_motors_fr + delta_timestamp_motors_rl + delta_timestamp_motors_rr) / 4;
                          

%%%%%%%%%%%%%%%%%%%%%%%%%%%%APPARENT_BASELINE%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%2 * y0 = r * integral((w_r - w_l)dt) / theta

% 1) compute integral((w_r - w_l)dt)
%      - I assume that w_r and w_l are steady during dt
integral = 0; %initialization
for i = 1:14135
    integral = integral + (delta_timestamp_motors(i) * (w_r(i) - w_l(i)));
end
% 2) compute theta = total rotation around the Euler axis
%      - I assume that the Euler axis is always perpendicular to the robot
%      - I assume the robot does not rotate more than 180° in dt 
theta_values = zeros(18009,1); %initialization
for i = 1:18009
    if theta_w(i) > 0
        theta_values(i) = 2 * acos(theta_w(i));
    else
        theta_values(i) = (-2) * acos(theta_w(i));
    end
end
theta = sum(theta_values);

% 3) compute apparent baseline
apparent_baseline = r * integral / theta;

