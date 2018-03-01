function out = project_mag(data)

n = zeros(size(data,1),3);
angle = zeros(size(data,1),1);
phins = zeros(size(data,1),3);

n_v = [1,1,0];
n_v = n_v/norm(n_v);

for i=1:size(data,1)
    
    n_ = data(i,7:9) - (data(i,7:9)*data(i,4:6)')*data(i,4:6);
    n(i,:) = n_/norm(n_);
    angle(i) = asin(norm(cross(n(i,:),n_v)))*180/pi;
    phins(i,:) = rot2rph(reshape(data(i,20:28),3,3))';
end

out.n = n;
out.angle = angle;
out.phins = phins;