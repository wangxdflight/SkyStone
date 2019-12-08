# odometry 3 wheels, each has 2 column;
# power readings for 4 wheels;
# base 4 wheels, each has 2 columns;
a=load('t.log');
plot(a(:, 2));
hold on;
plot(a(:, 12));
