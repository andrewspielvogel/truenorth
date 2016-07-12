function out = adap_Rbar(samp,R_align,lat)

tic 

lat = pi/180*lat;

data = samp;
data.acc = samp.acc';
data.ang = samp.ang';

disp('Computing Rd');
Rd   = get_R_d(data,eye(3));


disp('Computing Rb');


R0 = [-sin(lat),0,-cos(lat);0,1,0;cos(lat),0,-sin(lat)]*R_align;
wrand = [10,5,5];

R_distr = rph2R(wrand*pi/180);
R0 = R0*R_distr;
Rb = adap_so3(lat,samp,R0,Rd);

disp('Computing att');


out.Rb = Rb;
out.Rd = Rd;
out.Rsn = get_Rsn(lat,samp.t);
out.Rsi = cellfun(@(A,B) A*B,Rb.R,Rd,'UniformOutput',false);
out.Rin = cellfun(@(A,B) R_align*B'*A,out.Rsn,out.Rsi,'UniformOutput',false);
out.att = rph(out.Rin);
out.t = samp.t;
out.stamp = samp.stamp;

toc 



function Rse = get_Rse(t)

rate = 15*pi/180/3600;

num = size(t,2);
Rse{num} = eye(3);

for i = 1:num
   
    Rse{i} = expm(skew([0,0,1])*rate*t(i));

end


function R = get_Rsn(lat,t)

Ren = [-sin(lat),0,-cos(lat);0,1,0;cos(lat),0,-sin(lat)];
Rse = get_Rse(t);

R = cellfun(@(A) A*Ren,Rse,'UniformOutput',false);
