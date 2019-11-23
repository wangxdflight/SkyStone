max_rpm=435;
WHEEL_RADIUS=2;
GEAR_RATIO=0.5
Kv=max_rpm * GEAR_RATIO * 2 * pi * WHEEL_RADIUS / 60.0;

Kv=1/Kv
return;

a=load('..\curve.txt');

plot(a(:, 1));
hold on;
plot(a(:, 2));
hold on;
plot(a(:, 3));
hold on;
plot(a(:, 4));
hold on;
xlabel('time')
ylabel('inch/second');
title("velocity");
figure;

plot(a(:, 5));
hold on;
plot(a(:, 6));
hold on;
plot(a(:, 7));
hold on;
plot(a(:, 8));
hold on;
xlabel('time');
ylabel('power');
title("power");

figure;
b=a(:, 1)./a(:, 5)
ylim([-200, 2000])
plot(b);
hold on;
b=a(:, 2)./a(:, 6)
plot(b);
ylim([-200, 2000])
hold on;
b=a(:, 3)./a(:, 7)
plot(b);
ylim([-200, 2000])
hold on;
b=a(:, 4)./a(:, 8)
plot(b);
ylim([-200, 2000])
ylabel('velocity/power');
xlabel('time');
title("velocity/power");
c=1./b;
mean(c(100:300))
