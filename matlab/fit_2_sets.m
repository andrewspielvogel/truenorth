function R = fit_2_sets(A,B)
% least squares fitting of rotation using svd
% from Arun

samples = size(A,2);
% find sample averages
A_avg = sum(A,2)/samples;
B_avg = sum(B,2)/samples;

% compute diffs between samples and avgs
qa = A - repmat(A_avg,1,samples);
ba = B - repmat(B_avg,1,samples);

H = zeros(3,3);

% sum 3x3 matrices from multiplying samples diffs together
for i=1:samples
    H = H+qa(:,i)*ba(:,i)';
end

% get svd of H
[U,~,V] = svd(H);

% construct rotation
S = eye(3);
S(3,3) = det(U*V');
R = U*S*V';
