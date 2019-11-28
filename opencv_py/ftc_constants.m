############
clear all;
ticks_per_rev=383.6
WHEEL_RADIUS=2.0
# manually push robot to verify gear_ratio, motor encoder;
old_gear_ratio=2;
ticks=1447
distance=75.84
ticks_to_inch=WHEEL_RADIUS*2*pi*old_gear_ratio*ticks/ticks_per_rev
new_gear_ratio=distance*ticks_per_rev/(WHEEL_RADIUS*2*pi*ticks)
new_ticks_to_inch=WHEEL_RADIUS*2*pi*new_gear_ratio*ticks/ticks_per_rev
t=(99.5 / 13.7) * (32.0 / 16)
# caculate kV; 
max_rpm=435
rpm_to_vel=max_rpm*old_gear_ratio*2*pi*WHEEL_RADIUS/60;
kV=1.92*1.0/rpm_to_vel
new_kV=0.0095#0.009;#0.0103;#0.009;#0.01053#0.0093 #0.01053
# use actual RPM to get velocityF 
new_rpm_to_vel=1.0/new_kV;
new_rpm=new_rpm_to_vel*60/new_gear_ratio/(2*pi*WHEEL_RADIUS)
actual_rpm_ratio=new_rpm/max_rpm

ticks_per_second=new_rpm*ticks_per_rev/60;
vel_F=32767/ticks_per_second

##################
kP=0.1*vel_F
kI=0.1*kP
kD=0

############################
#edmund's code's values
v_f=32767/(60*2786/60)

return;


Kv=max_rpm * GEAR_RATIO * 2 * pi * WHEEL_RADIUS / 60.0;
Kv=1/Kv
######## plot X, Y, heading values and errors ###########
a=load('x_err.log');
plot(a(:, 1), "g-");
hold on;
plot(a(:, 2), "b+");
hold on;
plot(a(:, 1)+a(:, 2), "r*");
legend('X', 'x_err', 'distance');
title('target = 72 inches');

figure;
plot(a(:, 3), "g-");
hold on;
plot(a(:, 4), "b+");
hold on;
plot(a(:, 3)+a(:, 4), "r*");
legend('Y', 'y_err', 'distance');
title('target = 72 inches');

figure;
plot(a(:, 5), "g-");
hold on;
plot(a(:, 6), "b+");
hold on;
plot(a(:, 5)+a(:, 6), "r*");
legend('heading', 'heading_err', 'distance');
title('target = 72 inches');


