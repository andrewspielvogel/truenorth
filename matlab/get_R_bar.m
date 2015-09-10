function reconstruct = get_R_bar(samp,lat)

earth_rad   = 6371*1000;
lat  = lat*pi/180;
w_se = [0;0;15*pi/(3600*180)];

% get R_delta (pass in samples and the initial condition)
Rd   = get_R_d(samp,eye(3));

% define g and earth radius in earth frame
g    = 9.81*r_lat_long(lat);
r_e  = earth_rad*r_lat_long(lat);

% g_s = Rbar*g_d
% generate analytical g in star frame using lat
%g_s = cellfun(@(A) expm(skew(w_se)*A)*g,num2cell(samp.t),'UniformOutput',false);
g_s  = cellfun(@(A) skew(w_se)*skew(w_se)*expm(skew(w_se)*A)*r_e+expm(skew(w_se)*A)*g,num2cell(samp.t),'UniformOutput',false);

% transform measured acc into g vectors in zero from (gd = Rdelta*a_m)
g_d = cellfun(@(A,B) A*B',Rd,num2cell(samp.acc,2)','UniformOutput',false);


reconstruct.gs = cell2mat(g_s)/9.81; %normalize to be in units of g
reconstruct.gd = cell2mat(g_d);
reconstruct.Rdelta = Rd;

% find Rbar using svd method
reconstruct.Rbar = fit_2_sets(reconstruct.gs,reconstruct.gd);

% generate east approx from samples e(i) = Rdelta(i)*(-skew(a_m(i))*w_m(i)
e1 = cellfun(@(A,B) (-1)*A*skew(B),reconstruct.Rdelta,num2cell(samp.acc,2)','UniformOutput',false);
e = cellfun(@(A,B) A*B',e1,num2cell(samp.ang,2)','UniformOutput',false);

% generate north approx from samples n(i) =
% Rdelta(i)*(-skew(a_m(i))*skew(a_m(i)))*w_m(i)
n1 = cellfun(@(A,B) (-1)*A*skew(B)*skew(B),reconstruct.Rdelta,num2cell(samp.acc,2)','UniformOutput',false);
n = cellfun(@(A,B) A*B', n1,num2cell(samp.ang,2)','UniformOut',false);

% generate analytical east and north vectors for case at equator (still need to generalize to all lat) 
e_a = cellfun(@(A) A*[0;1;0],samp.R_si,'UniformOutput',false);
n_a = cellfun(@(A) A*[1;0;0],samp.R_si,'UniformOutput',false);

% normalize and create nde matrices for measured and analytical
reconstruct.as = [cell2mat(n_a)./repmat(sqrt(sum(cell2mat(n_a).^2,1)),3,1),(-1)*reconstruct.gs./repmat(sqrt(sum(reconstruct.gs.^2,1)),3,1),cell2mat(e_a)./repmat(sqrt(sum(cell2mat(e_a).^2,1)),3,1)];

reconstruct.ms = [cell2mat(n)./repmat(sqrt(sum(cell2mat(n).^2,1)),3,1),(-1)*reconstruct.gd./repmat(sqrt(sum(reconstruct.gd.^2,1)),3,1),cell2mat(e)./repmat(sqrt(sum(cell2mat(e).^2,1)),3,1)];

% find Rbar with svd method
reconstruct.rb = fit_2_sets(reconstruct.as,reconstruct.ms);

end