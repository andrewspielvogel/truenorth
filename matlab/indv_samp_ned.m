function [NED,w,deg] = indv_samp_ned(data,bias)
%function to calculate NED, w for eash sample
%
% Andrew Spielvogel
% andrewspielvogel@gmail.com
%
% July 2015
%

num_samples = size(data.ang);

w = data.ang(:,1:3)-repmat(bias.ang,num_samples(1),1);

NED.d = -(data.acc(:,1:3)-repmat(bias.acc,num_samples(1),1));
NED.e = cross(NED.d,w);
NED.n = cross(NED.e,NED.d);

%normalized vectors
wn = w./repmat(sqrt(sum(abs(w).^2,2)),1,3);
d = NED.d./repmat(sqrt(sum(abs(NED.d).^2,2)),1,3);
e = NED.e./repmat(sqrt(sum(abs(NED.e).^2,2)),1,3);
n = NED.n./repmat(sqrt(sum(abs(NED.n).^2,2)),1,3);

%get deg of north
rad = atan2(NED.n(:,2),NED.n(:,1));
deg = rad*180/pi;


%plot data
plot_comp(d,500,'down');
plot_comp(e,500,'east');
plot_comp(n,500,'north');
plot_comp(wn,500,'w');

% figure;
% hist(deg,500);
% grid;
% title('deg');