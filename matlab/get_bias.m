function bias = get_bias( xy, xy_flip, z )

bias.acc(1:2) = (mean(xy.acc(:,1:2))+mean(xy_flip.acc(:,1:2)))/2;
bias.acc(3)   = (mean(xy_flip.acc(:,3))+mean(z.acc(:,3)))/2;

bias.ang(1:2) = (mean(xy.ang(:,1:2))+mean(xy_flip.ang(:,1:2)))/2;
bias.ang(3)   = ((mean(xy_flip.ang(:,3))+mean(xy.ang(:,3)))/2+mean(z.ang(:,3)))/2;


end

