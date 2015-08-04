function f = plot_comp(data,n_bins)

figure;
subplot(3,1,1);
hist(data(:,1),n_bins);
xlabel('X');
grid;

subplot(3,1,2);
hist(data(:,2),n_bins);
xlabel('Y');
grid;

subplot(3,1,3);
hist(data(:,3),n_bins);
xlabel('Z');
grid;