function bias = get_bias( xy, xy_flip, z , z_flip,num)
%function for calculating the bias
%
% Andrew Spielvogel
% andrewspielvogel@gmail.com
%
% July 2015
%

xy_num = size(xy.acc,1);
xyf_num = size(xy_flip.acc,1);
z_num = size(z.acc,1);
zf_num = size(z_flip.acc,1);
num_samp = min([xy_num, xyf_num, z_num,zf_num]);

if (num_samp < num)
    num = num_samp;
    display('using all');
end

bias.acc(1:2) = (mean(xy.acc(1:2,1:num),2)+mean(xy_flip.acc(1:2,1:num),2))/2;
bias.acc(3)   = (mean(z_flip.acc(3,1:num),2)+mean(z.acc(3,1:num),2))/2;

bias.ang(1:2) = (mean(xy.ang(1:2,1:num),2)+mean(xy_flip.ang(1:2,1:num),2))/2;
bias.ang(3)   = (mean(z_flip.ang(3,1:num),2)+mean(z.ang(3,1:num),2))/2;


bias.ang = bias.ang';
bias.acc = bias.acc';

end

