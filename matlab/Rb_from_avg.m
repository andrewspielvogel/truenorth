function out = Rb_from_avg(samp,bias,lat)

samp.stamp = samp.stamp';

w = mean(samp.ang,1)'-bias.ang';
a = mean(samp.acc,1)'-bias.acc';

scale = norm(w)/(15.04*pi/(180*60*60));

fac = [1;1;1];


w = w.*fac;

num_samples = size(samp.ang,1);

samp.ang = samp.ang - repmat(bias.ang,num_samples,1);
samp.acc = samp.acc - repmat(bias.acc,num_samples,1);

samp.acc = repmat(fac',num_samples,1).*my_lowpass(samp.acc,samp.hz,1,5);
samp.ang = repmat(fac',num_samples,1).*my_lowpass(samp.ang,samp.hz,1,5);


samp.ang = repmat(w',num_samples,1);

Rd   = get_R_d(samp,eye(3));

d = -a;
e = cross(d,w);
n = cross(e,d);
d = d/norm(d);
e = e/norm(e);
n = n/norm(n);

d = cellfun(@(A) A*d, Rd,'UniformOut',false);
e = cellfun(@(A) A*e, Rd,'UniformOut',false);
n = cellfun(@(A) A*n, Rd,'UniformOut',false);

lat  = lat*pi/180; % convert to radians
w_se = [0;0;15.04*pi/(3600*180)]; % earth rate in rad/sec in space frame

% generate R_se (space to Earth frame) matrices for ts
R_se = cellfun(@(A) expm(skew(w_se)*(A-samp.stamp(1))),num2cell(samp.stamp),'UniformOutput',false);

% generate analytical east and north vectors for case at equator (still need to generalize to all lat) 
d_a = cellfun(@(A) A*(-1)*r_lat_long(lat),R_se,'UniformOutput',false);
e_a = cellfun(@(A) A*[0;1;0],R_se,'UniformOutput',false);
n_a = cellfun(@(A,B) cross(A,B),e_a,d_a,'UniformOutput',false);



% generate R_en (earth to NED frame) based on lat
R_en = [-sin(lat),0,-cos(lat);0,1,0;cos(lat),0,-sin(lat)];

% generate R_sn (Space to NED frame) matrices
R_sn = cellfun(@(A) A*R_en,R_se,'UniformOutput',false);

% R_si (space to instrument frame) matrices recovered from data



out.Rb = fit_2_sets([cell2mat(n_a),cell2mat(e_a),cell2mat(d_a)],[cell2mat(n),cell2mat(e),cell2mat(d)]);

out.R_si = cellfun(@(A) out.Rb*A,Rd,'UniformOutput',false);

% recovered (instrument to ned frame) matrices from data
out.R_in = cellfun(@(A,B) A'*B,out.R_si,R_sn,'UniformOutput',false);

out.att = rph(out.R_in);
out.Rdelta = Rd;
out.t = samp.stamp;