function plot_hist_acc_ang(data)
%
% 2015-09-15 LLW
% 
%  usage: plot_hist(data)

ang = data.ang(1:1000*60*5,:);
acc = data.acc(1:1000*60*5,:);

figure
center = 0;
width = 0.04;
height = 7500;
hw = width * 0.5;
title('Angular Velocity Histogram');
s = ang * (180/pi);
subplot(311);histogram(s(:,1)); grid on; 
axis([center- hw center + hw 0 height]);
xlabel('X (deg/sec)');

subplot(312);histogram(s(:,2)); grid on; 
axis([center- hw center + hw 0 height]);
xlabel('Y (deg/sec)');

subplot(313);histogram(s(:,3)); grid on; 
axis([center- hw center + hw 0 height]);
xlabel('Z (deg/sec)');

print -dpng hist_ang_vel.png
print -djpeg100 hist_ang_vel.jpeg
orient landscape
print -dpdf hist_ang_vel.pdf

figure
title('Linear Acceleration Histogram');
s = acc;
width = 0.025;
height = 7500;
hw = width * 0.5;

subplot(311);histogram(s(:,1)); grid on; 
center = 0;
axis([center- hw center + hw 0 height]);
xlabel('X (g)');

subplot(312);histogram(s(:,2)); grid on; 
axis([center- hw center + hw 0 height]);
xlabel('Y (g)');

center = 1;
subplot(313);histogram(s(:,3)); grid on; 
axis([center- hw center + hw 0 height]);
xlabel('Z (g)');

print -dpng hist_acc.png
print -djpeg100 hist_acc.jpeg
orient landscape
print -dpdf hist_acc.pdf


