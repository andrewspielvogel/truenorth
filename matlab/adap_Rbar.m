function out = adap_Rbar(samp,R_align,lat)

tic 

lat = pi/180*lat;

num_samp  = size(samp.stamp,2);

data = samp;
data.acc = samp.acc';
data.ang = samp.ang';

disp('Computing Rd');
Rd   = get_R_d(data,eye(3));


disp('Computing Rb');


u_a = zeros(3,num_samp);
y_a = zeros(3,num_samp);
y_w = zeros(3,num_samp);
u_w = samp.ang;

Rsn{num_samp} = zeros(3,3);

for i=1:num_samp
    
    
    Rsn{i} = get_Rsn(lat,samp.stamp(i));
    y_a(:,i) = Rsn{i}*[0;0;-1];
    u_a(:,i) = Rd{i}*samp.acc(:,i);
    y_w(:,i) = [-sin(lat),0,cos(lat)];
    %u_w(:,i) = samp.ang(:,i);
    
end

R0 = [-sin(lat),0,-cos(lat);0,1,0;cos(lat),0,-sin(lat)]*R_align;
wrand = [3,10,10];

R_distr = rph2R(wrand*pi/180);
R0 = eye(3); %R0*R_distr;
Rb = adap_so3(y_a,u_a,y_w,u_w,samp.stamp,R0,Rd);

disp('Computing att');

out.u_a = u_a;
out.y_a = y_a;
out.y_w = y_w;
out.u_w = u_w;
out.Rb = Rb;
out.Rsn = Rsn;
out.Rd = Rd;
out.Rsi = cellfun(@(A,B) A*B,Rb.R,Rd,'UniformOutput',false);
out.Rin = cellfun(@(A,B) R_align*B'*A,Rsn,out.Rsi,'UniformOutput',false);
out.att = rph(out.Rin);
out.t = samp.stamp;

toc 


function R = get_Rse(t)

rate = 15*pi/180/3600;

Rse = expm(skew([0,0,1])*rate*t);

R = Rse;

function R = get_Rsn(lat,t)

Ren = [-sin(lat),0,-cos(lat);0,1,0;cos(lat),0,-sin(lat)];
Rse = get_Rse(t);

R = Rse*Ren;


