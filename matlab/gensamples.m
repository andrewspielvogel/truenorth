function samp = gensamples(hz,t_end,bias)

tic

if nargin<3
   
    bias.acc = zeros(3,1);
    bias.ang = zeros(3,1);
    
end

% Define t
t = 0:1/hz:t_end;

% noise
w_sig = 2.1 * 10^(-4);  % measured 1775, units are rad/sec
a_sig = 0.0037;            % measured 1775, units are g, not m/s^2

% initialize R at t0
R{1} = eye(3);

num = size(t,2);

R{num} = zeros(3,3);

samp.ang = zeros(3,num);
samp.acc = zeros(3,num);
samp.true.ang = zeros(3,num);
samp.true.acc = zeros(3,num);

for i=1:num

    % get w at t
    w = get_w(t(i));
    
    % generate R at t
    if i>1
        R{i} = R{i-1}*expm(skew(w)*(t(i)-t(i-1))); 
    end
    
    % save true signal
    samp.true.ang(:,i) = w;
    samp.true.acc(:,i) = R{i}*[0;0;1];
    
    % generate ang and acc samples at t
    samp.ang(:,i) = R{i}*[1/sqrt(2);0;1/sqrt(2)]*15*pi/180/3600 + w + w_sig*randn(3,1) + bias.ang;
    
    samp.acc(:,i) = R{i}*[0;0;1]  + a_sig*randn(3,1) + bias.acc;% + [sin(t(i)/7);cos(t(i)/10)/5;-sin(t(i)/15)/3]/10;
    
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
samp.true.bias = bias;
samp.noise.w_sig = w_sig;
samp.noise.a_sig = a_sig;
toc

