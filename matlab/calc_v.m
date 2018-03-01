function v = calc_v(out)

num = size(out.Rb_rph,2);

v = zeros(num,1);

for i=1:num
    
   
    v(i) = norm(unskew(logm(rph2R(out.att(:,i))'*rph2R(out.Rb_rph(:,i)))))^2;
    
end