function R = get_R_d(samp,R0)

num  = size(samp.ang,1);
R = cell(1,num);
R{1} = R0;

for i=2:num
    
    R{i} = R{i-1}*expm(skew(samp.ang(i-1,:))*(samp.t(i)-samp.t(i-1))); 
    
end