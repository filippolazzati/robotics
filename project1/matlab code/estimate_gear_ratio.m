%%%%%%%%%%%%%%%%%%%%%%%%%Given constants%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%radius:
r = 0.1575;
%real_baseline:
real_baseline = 0.583;

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

%data from motor_speed_fl: (14136 rows)
w_fl = motor_speed_fl(:,5)  * (-1); %left wheels rotate in opposite direction
%data from motor_speed_fr: (14136 rows)
w_fr = motor_speed_fr(:,5);
%data from motor_speed_rl: (14136 rows)
w_rl = motor_speed_rl(:,5)  * (-1); %left wheels rotate in opposite direction
%data from motor_speed_rr: (14136 rows)
w_rr = motor_speed_rr(:,5);

%convert rpm in rad/s
w_fl_rad = ((w_fl / 60) * 2 * pi);
w_fr_rad = ((w_fr / 60) * 2 * pi);
w_rl_rad = ((w_rl / 60) * 2 * pi);
w_rr_rad = ((w_rr / 60) * 2 * pi);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%GEAR RATIO:%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%average angular velocity for tread
w_l = (w_fl_rad + w_rl_rad)/2;
w_r = (w_fr_rad + w_rr_rad)/2;

%gear ratio: (2 * v_x) / ((w_l + w_r) * r);
num = (2 * v_x); %temp variable
den = ((w_l + w_r) * r); %temp variable
gear_ratio = zeros(14136, 1); %preallocate the array

%fill in the gear_ratio array with '0' instead of outliers
%(Assumption: outlier = gear ratio < 1/100 or > 1/5)
for i = 1:14135
    %avoid division by zero
    if den(i) ~= 0
        gear_ratio(i) = num(i) / den(i);
    end
    %discard outliers
    if gear_ratio(i) < 1 / 100 || gear_ratio(i) > 1 / 5
        gear_ratio(i) = 0;
    end
end

%customised mean algorithm (without zero values)
sum = 0;
count = 0;
for i = 1:14135
    if gear_ratio(i) ~= 0
        count = count + 1;
        sum = sum + gear_ratio(i);
    end
end

%print the estimates
avg_gear_ratio = sum / count;

%plot
ciao = zeros(14136, 1);

for i = 1:14136
    ciao(i) = i;
end

plot(ciao, gear_ratio)







