function f = plotbias(data)

figure; plot(data.t,data.dacc); grid on;
title('acc dot');
legend('x','y','z');

figure; plot(data.t,data.acc); grid on;
title('acc');
legend('x','y','z');


figure; plot(data.t,data.dwb); grid on;
title('wb dot');
legend('x','y','z');


figure; plot(data.t,data.wb); grid on;
title('wb');
legend('x','y','z');


figure; plot(data.t,data.R); grid on;
title('R');
legend('R11','R21','R31','R12','R22','R32','R13','R23','R33');

figure; plot(data.t,data.dR); grid on;
title('R dot');
legend('dR11','dR21','dR31','dR12','dR22','dR32','dR13','dR23','dR33');
