function samp = gensamples(hz,t_end,bias)

% Define t
t = 0:1/hz:t_end;

% noise
w_sig = 0;%8.4178 * 10^(-5);  % measured 1775, units are rad/sec
a_sig = 0;%0.0023;            % measured 1775, units are g, not m/s^2


for i=1:size(t,2)
   
    w = [sin(t(i)/100);sin(t(i)/200);cos(t(i)/150)];
    R = expm(skew([1-cos(t(i)/100);1-cos(t(i)/200);sin(t(i)/150)]));
    
    samp.ang(:,i) = w + w_sig*rand(3,1) + bias.ang;
    samp.acc(:,i) = R'*[0;0;1] + a_sig*rand(3,1) + bias.acc;
    
    if ~mod(t(i),30)
        disp(t(i));
    end
    
end

samp.t = t;
samp.stamp = t;
samp.hz = hz;
samp.true.bias = bias;

