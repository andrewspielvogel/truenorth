function f = plotatt(kvh,ground)

plotrph(kvh,ground);

figure; plot(kvh.t,R2rph(kvh.Rb)*180/pi); grid on;
legend('roll','pitch','heading');
title('Rbar');

figure; plot(kvh.t,kvh.east_est_n); grid on;
legend('x','y','z');
title('east in ned');

% figure; plot(kvh.t,kvh.acc-ground.acc); grid on;
% legend('x','y','z');
% title('acc est - acc true');


figure; plot(kvh.t,kvh.g_error); grid on;
title('g error');

figure; plot(kvh.t,kvh.east_error); grid on;
title('east error');