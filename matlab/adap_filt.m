function out = adap_filt(hz,t_end,bias)

% Define t
t = 0:1/hz:t_end;

% Define parameters
S.k1 = 1000;
S.k2 = 10;
S.k3 = 10;
S.k4 = .01;
S.bias = bias;

% initial conditions
x0 = [[0;0;1];zeros(9,1);[1;0;0;0;1;0;0;0;1]];

% solve
[ts, xs] = ode45(@filt_ode,t,x0,[],S);

% assign outputs
out.acc = xs(:,1:3);
out.bias.ang = xs(:,4:6);
out.bias.acc = xs(:,7:9);
out.bias.z = xs(:,10:12);
out.t = ts;
out.hz = hz;
out.x = xs;
out.true.bias = bias;

for i=1:size(ts,1)
    
    out.true.ang(:,i) = get_w(ts(i));
    out.true.acc(:,i) = reshape(xs(i,13:21),3,3)'*[0;0;1];
    
end
% plot bias output
plot_adap(out);


% define w at time t
function w = get_w(t)

w = [sin(t/17);cos(t/14);sin(t/12)];


function dx = filt_ode(t,x,S)

% noise
w_sig = 8.4178 * 10^(-5);  % measured 1775, units are rad/sec
a_sig = 0.0023;            % measured 1775, units are g, not m/s^2

% w at time t
w = get_w(t);
ang = w + w_sig*randn(3,1) + S.bias.ang;

% get R and dR at time t
R = reshape(x(13:21),3,3);
dR = R*skew(w);

% get acc at time t
acc = R'*[0;0;1] + a_sig*randn(3,1) + S.bias.acc;

% define delta acc
da = x(1:3) - acc;

% define dx
dx = [skew(-ang)*x(1:3) + skew(ang)*x(7:9)+skew(-acc)*x(4:6)-x(10:12)-S.k1*da;
     S.k2*skew(-acc)*da;
     S.k3*(skew(ang)*da);
     S.k4*(da);
     reshape(dR,9,1)];
 