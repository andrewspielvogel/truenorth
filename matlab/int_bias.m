function out = int_bias(samp,lat)


kdacc = 10;
kwb = 2;
kR = 0;

out.t = samp.t;
lat = lat*pi/180;
num = size(samp.t,2);


out.da = zeros(3,num);
out.acc = zeros(3,num);
out.wb = zeros(3,num);

out.R = zeros(9,num);
out.R(:,1) = reshape(diag([1;1;1]),9,1);
out.Rd{1} = eye(3);
out.Rd{num} = eye(3);
out.dacc = zeros(3,num);
out.dwb = zeros(3,num);
out.dR = zeros(9,num);

Rne = [-sin(lat),0,-cos(lat);0,1,0;cos(lat),0,-sin(lat)]';
w_e = [0;0;1]*15/pi/180/3600;
samp.t = samp.t-samp.t(1);
out.acc(:,1) = samp.acc(:,1);
r = 6371*1000;
a_e = [cos(lat);0;sin(lat)] - (15*pi/180/3600)^2*cos(lat)*[r;0;0]/9.81;
a_n = Rne*a_e;
w_n = Rne*w_e;
e_n = skew(w_n)*a_n;


for i=2:num
    
    dt = samp.t(i) - samp.t(i-1);
    out.Rd{i} = out.Rd{i-1}*expm(skew(samp.ang(:,i-1))*dt);

    R_sn = get_Rsn(lat,samp.t(i));
    kronecker = kron(R_sn*e_n,out.Rd{i});
    out.da(:,i) = out.acc(:,i-1) - samp.acc(:,i);
    out.dacc(:,i) = -skew(samp.ang(:,i)-out.wb(:,i-1))*samp.acc(:,i) + kronecker'*out.R(:,i-1) - kdacc*out.da(:,i);
    out.dR(:,i) = kronecker*out.da(:,i);
    out.dwb(:,i)   = -skew(samp.acc(:,i))*out.da(:,i);
    
    
    out.acc(:,i) = out.acc(:,i-1) + dt*out.dacc(:,i);
    out.R(:,i)   = out.R(:,i-1)   + dt*kR*out.dR(:,i);
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




