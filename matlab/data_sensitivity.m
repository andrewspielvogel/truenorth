function [out_data,out_sim] = data_sensitivity(data,bias,N,num_samples_2_avg,freq)

num = size(num_samples_2_avg);

w_sig = 2/(10000*sqrt(1/freq));
a_sig = .12*sqrt(3)/(1000*sqrt(1/freq));

s_bias.ang = [0,0,0];
s_bias.acc = [0,0,0];

sim_samples = gen_samp([1,1,sind(39.28)],[0,0,1],60*60*1000,w_sig,a_sig);


for i=1:num(2)
    
    [~,~,d{i}] = plot_north(data,bias,N,num_samples_2_avg(i),0);
    [~,~,ds{i}] = plot_north(sim_samples,s_bias,N,num_samples_2_avg(i),0);

    data_mean(i) = mean(d{i});
    data_std(i)  = std(d{i});
    
    sim_mean(i) = mean(ds{i});
    sim_std(i)  = std(ds{i});
    
end

figure;
hold on;
plot(num_samples_2_avg/freq,data_std,'-b^');
plot(num_samples_2_avg/freq,data_mean,'-r*');
plot(num_samples_2_avg/freq,sim_std,'-k^');
plot(num_samples_2_avg/freq,sim_mean,'-g*');
ylabel('Degrees');
xlabel('Time Averaged');
legend('data std','data mean','sim std','sim mean');

out_data.mean = data_mean;
out_data.std  = data_std;

out_sim.mean = sim_mean;
out_sim.std  = sim_std;
