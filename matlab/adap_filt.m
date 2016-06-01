function out = adap_filt(hz,t_end,bias)

% Define t
t = 0:1/hz:t_end;

% Define parameters
S.k1 = 1;
S.k2 = .005;
S.k3 = .001;
S.k4 = .00005;
S.bias = bias;
% noise
S.w_sig = 8.4178 * 10^(-5);  % measured 1775, units are rad/sec
S.a_sig = 0.0023;            % measured 1775, units are g, not m/s^2

% initial conditions
x0 = [[0;0;1];zeros(9,1);[1;0;0;0;1;0;0;0;1]];

% solve
[ts, xs] = ode45(@filt_ode,t,x0,[],S);

% assign outputs
out.acc = xs(:,1:3)';
out.bias.ang = xs(:,4:6)';
out.bias.acc = xs(:,7:9)';
out.bias.z = xs(:,10:12)';
out.t = ts';
out.hz = hz;
out.k = [S.k1;S.k2;S.k3;S.k4];
out.true.bias = bias;
out.noise.w_sig = S.w_sig;
out.noise.a_sig = S.a_sig;

for i=1:size(ts,1)
    
    out.true.ang(:,i) = get_w(ts(i));
    out.true.acc(:,i) = reshape(xs(i,13:21),3,3)'*[0;0;1];
    
end



function dx = filt_ode(t,x,S)

% w at time t
w = get_w(t);
ang = w + S.w_sig*randn(3,1) + S.bias.ang;

% get R and dR at time t
R = reshape(x(13:21),3,3);
dR = R*skew(w);

% get acc at time t
acc = R'*[0;0;1] + S.a_sig*randn(3,1) + S.bias.acc;

% define delta acc
da = x(1:3) - acc;

% define dx
dx = [skew(-ang)*x(1:3) + skew(ang)*x(7:9)+skew(-acc)*x(4:6)-x(10:12)-S.k1*da;
     S.k3*skew(-acc)*da;
     S.k2*(skew(ang)*da);
     S.k4*(da);
     reshape(dR,9,1)];
 