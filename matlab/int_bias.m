function out = int_bias(samp,lat)


kacc = .01;
kdacc = 10;
kwb = diag([.01,.01,.01])*50;
kR = 10;

out.t = samp.t;
lat = lat*pi/180;
num = size(samp.t,2);


out.da = zeros(3,num);
out.acc = zeros(3,num);
out.wb = zeros(3,num);
out.R = zeros(9,num);
out.R(:,1) = reshape(diag([1;-1;-1]),9,1);
out.Rd{1} = eye(3);
out.Rd{num} = eye(3);
out.dacc = zeros(3,num);
out.dwb = zeros(3,num);
out.dR = zeros(9,num);

Rne = [-sin(lat),0,-cos(lat);0,1,0;cos(lat),0,-sin(lat)]';
we_n = Rne*[0;0;1];
a_n = [0;0;-1];
samp.t = samp.t-samp.t(1);

for i=2:num
    
    dt = samp.t(i) - samp.t(i-1);
    out.Rd{i} = out.Rd{i-1}*expm(skew(samp.ang(:,i-1))*dt);

    out.da(:,i) = out.acc(:,i-1) - samp.acc(:,i);
    R_sn = get_Rsn(lat,samp.t(i));
    out.dacc(:,i) = -skew(samp.ang(:,i)-out.wb(:,i-1))*samp.acc(:,i) + (kron((R_sn*skew(we_n)*a_n)',out.Rd{i}'))*out.R(:,i-1) - kdacc*out.da(:,i);
    out.dR(:,i)   = -kron(R_sn*skew(we_n)*a_n,out.Rd{i})*out.da(:,i);
    out.dwb(:,i)   = -skew(samp.acc(:,i))*out.da(:,i);
    
    
    out.acc(:,i) = out.acc(:,i-1) + dt*kacc*out.dacc(:,i);
    out.R(:,i)   = out.R(:,i-1)   + dt*kR/kacc*out.dR(:,i);
    out.wb(:,i)  = out.wb(:,i-1)  + dt*kwb/kacc*out.dwb(:,i);
    
    
end



function R = get_Rse(t)

rate = 15*pi/180/3600;

Rse = expm(skew([0,0,1])*rate*t);

R = Rse;

function R = get_Rsn(lat,t)

Ren = [-sin(lat),0,-cos(lat);0,1,0;cos(lat),0,-sin(lat)];
Rse = get_Rse(t);

R = Rse*Ren;




