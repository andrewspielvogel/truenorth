function f = plot_comp(data,n_bins,name)
% plot components of a vector
%
% Andrew Spielvogel
% andrewspielvogel@gmail.com
%
% August 2015
%

if ~exist('name','var')
    name = '';
end

figure;
subplot(3,1,1);
hist(data(:,1),n_bins);
str = sprintf('%s X mean=%.3g std=%.3g',name,mean(data(:,1)),std(data(:,1)));
xlabel(str);
grid;

subplot(3,1,2);
hist(data(:,2),n_bins);
str = sprintf('%s Y mean=%.3g std=%.3g',name,mean(data(:,2)),std(data(:,2)));
xlabel(str);
grid;

subplot(3,1,3);
hist(data(:,3),n_bins);
str = sprintf('%s Z mean=%.3g std=%.3g',name,mean(data(:,3)),std(data(:,3)));
xlabel(str);
grid;