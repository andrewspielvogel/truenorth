function f = plotrd(data)

% cut_freq_hz = 1;
% samp.stamp = data.stamp';
% samp.ang = my_lowpass(data.ang,data.hz,1,cut_freq_hz);
% samp.t = data.stamp;
% Rd   = get_R_d(samp,eye(3));
% d.R_in = Rd;
% d.t = samp.stamp;
% data=d;
% 


num = size(data.Rdelta,2);

out = zeros(num,3);

for i=1:num
    
    out(i,:) = (rot2rph(data.Rdelta{i}))';
    
end
data_rph = out*180/pi;

figure;
hold on;

% data.t = data.t-data.t(1);
% ground.t = ground.t - data.t(1);


subplot(3,1,1);
hold on;
plot(data.t,data_rph(:,1),'-b');
title('x');
ylabel('Degrees');
xlabel('Time [s]');
xlim([data.t(1),data.t(end)]);
grid on;

subplot(3,1,2);
hold on;
plot(data.t,data_rph(:,2),'-b');



title('y');
ylabel('Degrees');
xlabel('Time [s]');
xlim([data.t(1),data.t(end)]);
grid on;

subplot(3,1,3);
hold on;

plot(data.t,data_rph(:,3),'-b');
title('z');
ylabel('Degrees');
xlabel('Time [s]');
xlim([data.t(1),data.t(end)]);
grid on;


