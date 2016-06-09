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


u = zeros(3,num_samp);
y = zeros(3,num_samp);
Rsn{num_samp} = zeros(3,3);

for i=1:num_samp
    
    
    Rsn{i} = get_Rsn(lat,samp.stamp(i));
    y(:,i) = Rsn{i}*[0;0;-1];
    u(:,i) = Rd{i}*samp.acc(:,i);
    
    
end

R0 = [-sin(lat),0,-cos(lat);0,1,0;cos(lat),0,-sin(lat)]*R_align;
Rb = adap_so3(y,u,samp.stamp,R0);

disp('Computing att');

out.u = u;
out.y = y;
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


