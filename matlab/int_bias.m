function out = int_bias(samp,lat,R_align,bang)
% int_bias(samp,lat)
% samp - data
% lat  - latitude in degrees

% gains
kdacc = 1;
kwb = diag([1,1,1])*0.04;

out.t = samp.t;
lat   = lat*pi/180;
num   = size(samp.t,2);


% init
out.da       = zeros(3,num);
out.acc      = zeros(3,num);
out.wb       = zeros(3,num);
out.wb(:,1) = bang;
out.dacc     = zeros(3,num);
out.dwb      = zeros(3,num);
samp.t       = samp.t-samp.t(1);
out.acc(:,1) = samp.acc(:,1);
r = 6371*1000;

% calc e_n
w_e = [0;0;1]*15*pi/180/3600;
a_e = [cos(lat);0;sin(lat)] - (15*pi/180/3600)^2*cos(lat)*[r;0;0]/9.81;
e_e = skew(w_e)*a_e;
Ren = [-sin(lat),0,-cos(lat);0,1,0;cos(lat),0,-sin(lat)];
e_n = Ren'*e_e;
Rd = eye(3);
Rb = R_align;%*expm(skew([0;0;5]*pi/180));

% do estimation
for i=2:num
    
    dt        = samp.t(i) - samp.t(i-1);
    
    %R_se        = get_Rse(samp.t(i));
    out.da(:,i) = out.acc(:,i-1) - samp.acc(:,i);
    
    %Rd = Rd*expm(skew((samp.ang(:,i-1)-out.wb(:,i-1))*dt));
    Rd = Rd*expm(skew((samp.ang(:,i-1)-samp.bias.ang)*dt));
    
    out.dacc(:,i) = -skew(samp.ang(:,i)-out.wb(:,i-1))*samp.acc(:,i) + Rd'*Rb'*e_n - kdacc*out.da(:,i);
    out.dwb(:,i)  = -kwb*skew(samp.acc(:,i))*out.da(:,i);

    out.acc(:,i) = out.acc(:,i-1) + dt*out.dacc(:,i);
    out.wb(:,i)  = out.wb(:,i-1)  + dt*out.dwb(:,i);
    
end



function R = get_Rse(t)

rate = 15*pi/180/3600;

Rse = expm(skew([0,0,1])*rate*t);

R = Rse;

function R = get_Rsn(lat,t)

Ren = [-sin(lat),0,-cos(lat);0,1,0;cos(lat),0,-sin(lat)];
Rse = get_Rse(t);

R = Rse*Ren;




