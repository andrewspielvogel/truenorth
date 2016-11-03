function out = int_bias(samp,lat,R_align)
% int_bias(samp,lat)
% samp - data
% lat  - latitude in degrees

% gains
kdacc = 1;
kwb = 1/10;
kR = 1000;

out.t = samp.t;
lat = lat*pi/180;
num = size(samp.t,2);

% init
out.da = zeros(3,num);
out.acc = zeros(3,num);
out.wb = zeros(3,num);

Rsz = get_Rsn(lat,0)*R_align;
out.R = zeros(9,num);
out.R(:,1) = reshape(Rsz',9,1);
out.dR = zeros(9,num);
out.Rd{1} = eye(3);
out.Rd{num} = eye(3);
out.dacc = zeros(3,num);
out.dwb = zeros(3,num);
samp.t = samp.t-samp.t(1);
out.acc(:,1) = samp.acc(:,1);
r = 6371*1000;

% calc e_n
w_e = [0;0;1]*15*pi/180/3600;
a_e = [cos(lat);0;sin(lat)] - (15*pi/180/3600)^2*cos(lat)*[r;0;0]/9.81;
e_e = skew(w_e)*a_e;

out.etot = zeros(3,num);
out.ee = zeros(3,num);
% do estimation
for i=2:num
    
    dt        = samp.t(i) - samp.t(i-1);
    out.Rd{i} = out.Rd{i-1}*expm(skew(samp.ang(:,i-1)-samp.bias.ang)*dt);

    R_se        = get_Rse(samp.t(i));
    out.da(:,i) = out.acc(:,i-1) - samp.acc(:,i);
    
    kronecker = kron(R_se*e_e,out.Rd{i});
    
    out.etot(:,i) = -skew(samp.ang(:,i)-out.wb(:,i-1))*samp.acc(:,i);
    out.ee(:,i) = out.Rd{i}'*Rsz'*R_se*e_e;
    out.dacc(:,i) = -skew(samp.ang(:,i)-out.wb(:,i-1))*samp.acc(:,i) + out.Rd{i}'*Rsz'*R_se*e_e - kdacc*out.da(:,i);
    %out.dacc(:,i) = -skew(samp.ang(:,i)-out.wb(:,i-1))*samp.acc(:,i) + kronecker'*out.R(:,i-1) - kdacc*out.da(:,i);
    out.dwb(:,i)  = -skew(samp.acc(:,i))*out.da(:,i);
    out.dR(:,i) = kronecker*out.da(:,i);

    out.R(:,i)   = out.R(:,i-1)   + dt*kR*out.dR(:,i);
    out.acc(:,i) = out.acc(:,i-1) + dt*out.dacc(:,i);
    out.wb(:,i)  = out.wb(:,i-1)  + dt*kwb*out.dwb(:,i);
    
end



function R = get_Rse(t)

rate = 15*pi/180/3600;

Rse = expm(skew([0,0,1])*rate*t);

R = Rse;

function R = get_Rsn(lat,t)

Ren = [-sin(lat),0,-cos(lat);0,1,0;cos(lat),0,-sin(lat)];
Rse = get_Rse(t);

R = Rse*Ren;




