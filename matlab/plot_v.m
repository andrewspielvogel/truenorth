function out = plot_v(samp,proc)
out.t = proc.t;
num = size(proc.t,2);

out.da = proc.east_n - (samp.acc_v(:,1:num) + repmat(samp.bias.acc,1,num));


out.dw = proc.Rb_rph - repmat(samp.bias.ang,1,num);


out.da_term = zeros(num,1);
out.dw_term = zeros(num,1);
for i=1:num
   
    out.da_term(i) = (norm(out.da(:,i)))^2;
    out.dw_term(i) = (norm(out.dw(:,i)))^2;
    
end