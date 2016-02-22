function out = rph(Rin)


num = size(Rin,2);

out = zeros(num,3);

for i=1:num
    
    out(i,:) = (rot2rph(Rin{i}'))';
    
end
