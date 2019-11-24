################ plot velocity/power ##################
a=load('curve2.log');

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
mean(c(200:300))

figure;

target_v=a(:, 9);
plot(target_v, '-');
hold on;
v0=a(:, 10)
plot(v0, '-')
hold on;
v1=a(:, 12)
plot(v1, '-');
hold on;
v2=a(:, 14)
plot(v2, '-');
hold on;
v3=a(:, 16)
plot(v3, '-');
legend('target velocity', 'wheel 0', 'wheel 1', 'wheel 2', 'wheel 3');
title("velcoity");

figure;
title("error 0, 1, 2, 3")
e0=a(:, 11);
plot(e0, '-');
hold on;
e1=a(:, 13);
plot(e1, '-');
hold on;
e2=a(:, 15);
plot(e2, '-');
hold on;
e3=a(:, 17);
plot(e3, '-');
legend('e0', 'e1', 'e2', 'e3');
