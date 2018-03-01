function bias = calc_bias(data,lat)

lat = lat*pi/180;
Ren = [-sin(lat),0,-cos(lat);0,1,0;cos(lat),0,-sin(lat)];
r = 6371*1000;
a_e = [cos(lat);0;sin(lat)] - (7.292150/100000)^2*cos(lat)*[r;0;0]/9.81;
a_n = Ren'*a_e;
w_E_e = [0;0;1]*7.292150/100000;
w_E_n = Ren'*w_E_e;

bias.w_b = mean(data(:,1:3))' - rph2R([pi;0;pi/4])'*rph2R(data(end,29:31))'*w_E_n;
bias.a_b = mean(data(:,4:6))' - rph2R([pi;0;pi/4])'*rph2R(data(end,29:31))'*a_n;
