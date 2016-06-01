function out = adap_Rbar(samp,R_align,bias)

data = samp;
data.acc = samp.acc' - bias.bias.acc';
data.ang = samp.ang' - bias.bias.ang';

Rd   = get_R_d(data,eye(3));

num_samp  = size(samp.stamp,2);
u = zeros(3,num_samp);
y1 = zeros(2,num_samp);
y2 = ones(1,num_samp);
y = [y1;-y2];

for i=1:num_samp
    
    u(:,i) = Rd{i}*samp.acc(:,i);
    
end

out.t = samp.stamp;
out.R = adap_so3(y,u,samp.stamp);
out.Rd = Rd;
out.Rin = cellfun(@(A,B) R_align*B'*A',out.R,Rd,'UniformOutput',false);
out.att = rph(out.Rin);