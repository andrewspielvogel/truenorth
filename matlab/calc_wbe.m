function out = calc_wbe(samp,hz)

lat = 39.32*pi/180;

ka = 10;
kE = 0.05;
kb = 0.5;
kab = 1;
ka2 = 1;

kg = .05;
ke = 50;

dt = 1/hz;
n = size(samp.t,2);

acc = zeros(3,n);
dacc = zeros(3,n);
acc2 = zeros(3,n);
dacc2 = zeros(3,n);
wb = zeros(3,n);
dwb = zeros(3,n);
wed = zeros(3,n);
dwe = zeros(3,n);
wen = zeros(3,n);
dwen = zeros(3,n);
da  = zeros(3,n);
dab = zeros(3,n);
ab = zeros(3,n);
da2  = zeros(3,n);
att = zeros(3,n);
e_tilde = zeros(3,n);
V = zeros(3,n);

R = eye(3)*expm(skew([1;1;10]*pi/180));


Ren = [-sin(lat),0,-cos(lat);0,1,0;cos(lat),0,-sin(lat)];

w_E_e = [0;0;1]*15.04*pi/180/3600;
w_E_n = Ren'*w_E_e;

acc(:,1) = samp.acc(:,1);
wen(:,1) = R'*[1;0;0];
wen(:,1) = wen(:,1)*w_E_n(1)/norm(wen(:,1));



r = 6371*1000;
a_e = [cos(lat);0;sin(lat)] - (15.04*pi/180/3600)^2*cos(lat)*[r;0;0]/9.81;
a_n = Ren'*a_e;

alpha = norm(w_E_n(1))/norm(a_e);

for i=2:n
    
    da2(:,i) = acc2(:,i-1) - samp.acc(:,i);
    dacc2(:,i) = -skew(samp.ang(:,i))*(samp.acc(:,i)-ab(:,i-1)) - ka2*da2(:,i);

    
    samp.acc(:,i) = samp.acc(:,i) - ab(:,i-1);%[0.003;-0.005;0.002];%ab(:,i-1);
    
    P = R'*(a_n*a_n')*R/(norm(a_n)^2);
    
    a_tilde = kg*R'*(skew(R*(samp.acc(:,i)))*a_n);
    e_tilde(:,i) = P*ke*skew(wen(:,i-1)/norm(wen(:,i-1)))*R'*w_E_n;
    R = R*expm(skew(a_tilde+e_tilde(:,i)+samp.ang(:,i)-wb(:,i-1)-wen(:,i-1)-wed(:,i-1))*dt);
    att(:,i) = rot2rph(R);
    
    
    samp.acc(:,i) = R'*a_n;
    da(:,i) = acc(:,i-1) - samp.acc(:,i);
    
    dacc(:,i) = -skew(samp.ang(:,i) - wen(:,i-1) - wb(:,i-1))*samp.acc(:,i) - ka*da(:,i);
    dwen(:,i) = -skew(samp.ang(:,i))*wen(:,i-1) - skew(samp.acc(:,i))*(alpha*wen(:,i-1) + kE*acc(:,i-1));
    dwb(:,i)  = -skew(samp.acc(:,i))*acc(:,i-1);
    
    
    dab(:,i) = skew(samp.ang(:,i))*da2(:,i);
    
    acc(:,i) = acc(:,i-1) + dt*dacc(:,i);
    %wen(:,i) = (wen(:,i-1) + dt*dwen(:,i));
    wen(:,i) = (eye(3) - P)*(wen(:,i-1) + dt*dwen(:,i));
    wen(:,i) = wen(:,i)*w_E_n(1)/norm(wen(:,i));
    wed(:,i) = -samp.acc(:,i)*w_E_n(3)/norm(samp.acc(:,i));
    wb(:,i)  = wb(:,i-1) + dt*diag([kb,kb,20*kb])*dwb(:,i);
    
    acc2(:,i) = acc2(:,i-1) + dt*dacc2(:,i);
    ab(:,i)   = ab(:,i-1) + dt*diag([kab,kab,2*kab])*dab(:,i);
    
    V(i) = (da(:,i)'*da(:,i) + (wen(:,i)-samp.w_E_n(:,i))'*(wen(:,i)-samp.w_E_n(:,i)) + (wb(:,i) - samp.bias.ang)'*(wb(:,i) - samp.bias.ang))/2;
    
end
out.wb = wb;
out.acc = acc;
out.dacc = dacc;
out.da = da;
out.wen = wen;
out.dwen = dwen;
out.wed=wed;
out.acc2 = acc2;
out.ab = ab;
out.samp.acc = samp.acc;
out.att = att;
out.e_tilde = e_tilde;
out.V = V;
