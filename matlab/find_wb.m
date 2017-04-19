function out = find_wb(samp)

lat = 39.32*pi/180;

ka = 1;
kw = 0.03;

dt = 1/samp.hz;

Ren = [-sin(lat),0,-cos(lat);0,1,0;cos(lat),0,-sin(lat)];
w_E = Ren'*[0;0;15.04*pi/180/3600];
a_g = Ren'*[cos(lat)-6371*1000*cos(lat)*15.04*pi/180/3600*15.04*pi/180/3600/9.81;0;sin(lat)];

a_hat = zeros(3,size(samp.t,2));
a_hat_dot = zeros(3,size(samp.t,2));
wb_dot = zeros(3,size(samp.t,2));
wb = zeros(3,size(samp.t,2));

for i=2:size(samp.t,2)
    
    Rni = samp.Rni{i}*rph2R([0;0;0]*pi/180);

    dacc = a_hat(:,i-1) - (samp.acc(:,i)-Rni'*a_g);
    da = (samp.acc(:,i) - samp.acc(:,i-1))*samp.hz;
    
    a_hat_dot(:,i) = skew(samp.ang(:,i)-Rni'*w_E-wb(:,i-1))*Rni'*a_g + da - ka*dacc;
    wb_dot(:,i) = kw*skew(Rni'*a_g)*dacc;
    
    a_hat(:,i) = a_hat(:,i-1) + dt*a_hat_dot(:,i);
    wb(:,i)    = wb(:,i-1)    + dt*wb_dot(:,i);
    
    
end

out.a = a_hat;
out.wb = wb;
out.a_dot = a_hat_dot;
out.wb_dot = wb_dot;
