function samp = gensamples(hz,t_end,bias)
tic
% Define t
t = 0:1/hz:t_end;

% noise
w_sig = 8.4178 * 10^(-5);  % measured 1775, units are rad/sec
a_sig = 0.0023;            % measured 1775, units are g, not m/s^2
v_sig = 0.002/9.81;

% initialize R at t0
R{1} = eye(3);

num = size(t,2);

R{num} = zeros(3,3);

samp.ang = zeros(3,num);
samp.acc = zeros(3,num);
samp.true.ang = zeros(3,num);
samp.true.acc = zeros(3,num);
samp.vel = zeros(3,t_end);

for i=1:num

    % get w at t
    w = get_w(t(i));
    
    % generate R at t
    if i>1
        R{i} = R{i-1}*expm(skew(w)*(t(i)-t(i-1))); 
    end
    
    % save true signal
    samp.true.ang(:,i) = w;
    samp.true.acc(:,i) = R{i}'*[0;0;1];
    
    % generate ang and acc samples at t
    samp.ang(:,i) = w + w_sig*randn(3,1) + bias.ang;
    
    if ~mod(i-1,hz)
        samp.vel(:,floor(t(i))+1) = [0;sin(t(i)/20);cos(t(i)/15)-1]/10 + v_sig*randn(3,1);
    end
    
    samp.acc(:,i) = R{i}'*[0;0;1] + a_sig*randn(3,1) + bias.acc ;%+ [0;cos(t(i))/20;-sin(t(i))/15]/10;
    
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

