clear;
a=load('imu.txt');
plot(1:length(a(:,1)), a(:, 1), a(:, 2));
legend('IMU (degrees)', 'non-IMU (degrees)');
figure;
plot(1:400, a(1:400, 1), a(1:400, 2));
legend('IMU (degrees)', 'non-IMU (degrees)');
