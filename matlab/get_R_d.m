function R = get_R_d(samp,R0)

num  = size(samp.ang,1); % number of samples
R = cell(1,num); % pre allocate
R{1} = R0;

for i=2:num
    
    dR = expm(skew(samp.ang(i-1,:))*(samp.stamp(i)-samp.stamp(i-1)));
    R{i} = R{i-1}*dR; 
    
end