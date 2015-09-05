function R = fit_2_sets(A,B)

samples = size(A,2);

A_avg = sum(A,2)/samples;
B_avg = sum(B,2)/samples;

qa = A - repmat(A_avg,1,samples);
ba = B - repmat(B_avg,1,samples);

H = zeros(3,3);

for i=1:samples
    H = H+qa(:,i)*ba(:,i)';
end


[U,S,V] = svd(H);

R = V*U';

if (det(R)<0)
    
    V(:,3) = -V(:,3);
    R=V*U';
    
end