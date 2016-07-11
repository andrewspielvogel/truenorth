function out = R2rph(R)

num = size(R,2);

out = zeros(num,3);

for i=1:num
    
    out(i,:) = (rot2rph(R{i}))';
    
end
