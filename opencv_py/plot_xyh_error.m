clear;
a=load('plot.txt');
subplot(3, 1, 1);
plot(a(:, 1));
title('xError');
ylabel("inches");
subplot(3, 1, 2);
plot(a(:, 2));
title('yError');
ylabel("inches");
subplot(3, 1, 3);
plot(a(:, 3)*180/3.14159);
title('headingError');
ylabel("degree");