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
figure;

subplot(1,3,1);
hist(d(:,1),500);
title('down x');

subplot(1,3,2);
hist(d(:,2),500);
title('down y');

subplot(1,3,3);
hist(d(:,3),500);
title('down z');

figure;

subplot(1,3,1);
hist(e(:,1),500);
title('east x');

subplot(1,3,2);
hist(e(:,2),500);
title('east y');

subplot(1,3,3);
hist(e(:,3),500);
title('east z');

figure;

subplot(1,3,1);
hist(n(:,1),500);
title('north x');

subplot(1,3,2);
hist(n(:,2),500);
title('north y');

subplot(1,3,3);
hist(n(:,3),500);
title('north z');

figure;

subplot(1,3,1);
hist(wn(:,1),500);
title('w x');

subplot(1,3,2);
hist(wn(:,2),500);
title('w y');

subplot(1,3,3);
hist(wn(:,3),500);
title('w z');

% figure;
% hist(deg,500);
% title('deg');