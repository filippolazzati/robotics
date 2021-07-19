%%%%%%%%%%%%%%%%%%%%%%%%%Given constants%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%radius:
r = 0.1575;
%real_baseline:
real_baseline = 0.583;
%gear_ratio:
gear_ratio = 0.0262;

%%%%%%%%%%%%%%%%%%%%%%%%%Read the files%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%CSVData
scout_odom = readmatrix('scout_odom.csv');
motor_speed_fl = readmatrix('motor_speed_fl.csv');
motor_speed_fr = readmatrix('motor_speed_fr.csv');
motor_speed_rl = readmatrix('motor_speed_rl.csv');
motor_speed_rr = readmatrix('motor_speed_rr.csv');

%%%%%%%%%%%%%%%%%%Data from scout odom and motors%%%%%%%%%%%%%%%%%%%%%%%%%%

%data from scout_odom: (14135 rows)
v_x = scout_odom(:,49);
w_z = scout_odom(:,54);

%data from motor_speed_fl: (14136 rows)
w_fl = motor_speed_fl(:,5)  * (-1); %left wheels rotate in opposite direction
%data from motor_speed_fr: (14136 rows)
w_fr = motor_speed_fr(:,5);
%data from motor_speed_rl: (14136 rows)
w_rl = motor_speed_rl(:,5)  * (-1); %left wheels rotate in opposite direction
%data from motor_speed_rr: (14136 rows)
w_rr = motor_speed_rr(:,5);

%convert rpm in rad/s and apply the gear_ratio
w_fl_rad = ((w_fl / 60) * 2 * pi) * gear_ratio;
w_fr_rad = ((w_fr / 60) * 2 * pi) * gear_ratio;
w_rl_rad = ((w_rl / 60) * 2 * pi) * gear_ratio;
w_rr_rad = ((w_rr / 60) * 2 * pi) * gear_ratio;

%%%%%%%%%%%%%%%%%%Compute velocities from sensors data%%%%%%%%%%%%%%%%%%%%%
%apply v = w * r, where r = wheels radius 
v_fl = w_fl_rad * r ; %(14136 rows)
v_fr = w_fr_rad * r ; %(14136 rows)
v_rl = w_rl_rad * r ; %(14136 rows)
v_rr = w_rr_rad * r ; %(14136 rows)

%average velocities of left and right treads
v_l = (v_fl + v_rl)/2;
v_r = (v_fr + v_rr)/2;

%%%%%%%%%%%%%%%%%%%%%%%%%APPARENT BASELINE%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%temporary variable:
delta_v = (v_r - v_l);

%2 algorithms:
%   1) I do not consider a value when w_z = 0;
%   2) I assume apparent_baseline = real_baseline when w_z = 0.

% 1) %RELIABLE METHOD
%I preallocate this array for efficiency reasons)
app_baseline1 = zeros(14135, 1);
for i = 1:14135
    if w_z(i) ~= 0
        app_baseline1(i) = delta_v(i) / w_z(i);
    end
end
%customised mean algorithm (without zero values)
sum = 0;
count = 0;
for i = 1:14135
    if app_baseline1(i) ~= 0
        count = count + 1;
        sum = sum + app_baseline1(i);
    end
end
%Compute the mean of the baseline:
avg_app_baseline = sum / count

% 2) %DEPRECATED
%I initialise this array to the physical value of the baseline
app_baseline2 = zeros(14135, 1) + real_baseline;
for i = 1:14135
    if w_z(i) ~= 0
        app_baseline2(i) = delta_v(i) / w_z(i);
    end
end
%Compute the mean of the baseline:
avg_app_baseline2 = mean(app_baseline2);