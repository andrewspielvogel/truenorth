function samp = gensamples(lat,hz,t_end,R_align,bias)

tic

lat = lat*pi/180;

if nargin<5
   
    bias.acc = zeros(3,1);
    bias.ang = zeros(3,1);
    
end


% Define t
dt = 1/hz;
t = 0:dt:t_end;

r = 6371*1000;

% noise
w_sig = 6.32 * 10^(-3)*pi/180;  % measured 1775, units are rad/sec
a_sig = 0.0037;            % measured 1775, units are g, not m/s^2

num = size(t,2);


samp.ang = zeros(3,num);
samp.acc = zeros(3,num);
samp.att = zeros(3,num);

samp.Rni{1} = eye(3);

Ren = [-sin(lat),0,-cos(lat);0,1,0;cos(lat),0,-sin(lat)];
a_e = [cos(lat);0;sin(lat)] - (15*pi/180/3600)^2*cos(lat)*[r;0;0]/9.81;
a_e = a_e/norm(a_e);
a_n = Ren'*a_e;

Rsz = get_Rsn(lat,0)*R_align;
samp.Rsz=Rsz;
w_earth_z = Rsz'*[0;0;1]*15*pi/180/3600;
    
for i=1:num


    
    w_veh = get_w(t(i));
    Rsn = get_Rsn(lat,t(i));
    
    if i~=num
        samp.Rni{i + 1} = samp.Rni{i}*expm(skew(w_veh)*dt);
    end
    samp.Rsi{i} = Rsn*samp.Rni{i};
    samp.Rd{i} = Rsz'*Rsn*samp.Rni{i};
    samp.Rsn{i} = Rsn;

    samp.att(:,i) = rot2rph(samp.Rni{i});  
    
    samp.ang(:,i) =  samp.Rni{i}'*w_veh + samp.Rsi{i}'*Rsz*w_earth_z + w_sig*randn(3,1) + bias.ang;
    samp.acc(:,i) =  samp.Rni{i}'*a_n + a_sig*randn(3,1) + bias.acc;
    samp.acc_z(:,i) = samp.Rd{i}*samp.acc(:,i);
   
        samp.w_v(:,i) = w_veh;
samp.e_v(:,i) = skew(w_veh)*samp.acc(:,i);
    % print progress
    if ~mod(t(i),30)
        str = sprintf('Made %i:%i0 of data at %i hz',floor(t(i)/60),mod(t(i),60)/10,hz);
        disp(str);
    end
        
end

% save
samp.t = t;
samp.stamp = t;
samp.hz = hz;
samp.bias = bias;
samp.noise.w_sig = w_sig;
samp.noise.a_sig = a_sig;
toc

function R = get_Rse(t)

rate = 15*pi/180/3600;

Rse = expm(skew([0,0,1])*rate*t);

R = Rse;

function R = get_Rsn(lat,t)

Ren = [-sin(lat),0,-cos(lat);0,1,0;cos(lat),0,-sin(lat)];
Rse = get_Rse(t);

R = Rse*Ren;

function w = get_w(t)

if t<5*60*0
    w=[0;0;0];
else

w = [cos(t)/7;sin(t*2.3)/4;0];
w = [cos(t/2)/20;sin(t)/10;cos(t/5)/40];
end


