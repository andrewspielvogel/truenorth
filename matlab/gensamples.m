function samp = gensamples(lat,hz,t_end,R_align,bias)

tic

lat = lat*pi/180;

if nargin<5
   
    bias.acc = zeros(3,1);
    bias.ang = zeros(3,1);
    
end

Rb = [-sin(lat),0,-cos(lat);0,1,0;cos(lat),0,-sin(lat)]*R_align;


% Define t
dt = 1/hz;
t = 0:dt:t_end;

% noise
w_sig = 6.32 * 10^(-3)*pi/180;  % measured 1775, units are rad/sec
a_sig = 0.0037;            % measured 1775, units are g, not m/s^2

num = size(t,2);


samp.ang = zeros(3,num);
samp.acc = zeros(3,num);

samp.Rd{1} = eye(3);

for i=1:num

    w = get_w(t(i));
    samp.Rd{i+1} = samp.Rd{i}*expm(skew(w*dt));
    samp.ang(:,i) = samp.Rd{i}'*(Rb'*[0;0;1]*15*pi/180/3600 + w) + w_sig*randn(3,1) + bias.ang;
    samp.acc(:,i) = samp.Rd{i}'*Rb'*[cos(lat);0;sin(lat)]  + a_sig*randn(3,1) + bias.acc;
    
    % print progress
    if ~mod(t(i),30)
        str = sprintf('Made %i:%i0 of data at %i hz',floor(t(i)/60),mod(t(i),60)/10,hz);
        disp(str);
    end
    
end

% save
samp.Rb = Rb;
samp.t = t;
samp.stamp = t;
samp.hz = hz;
samp.bias = bias;
samp.noise.w_sig = w_sig;
samp.noise.a_sig = a_sig;
toc

