function out = plot_errors(samp,out)

figure;plot(samp.t,out.wb,samp.t,repmat(samp.bias.ang,[1,size(samp.t,2)]));grid on;title('w_b');xlabel('Seconds');
figure;plot(samp.t,out.att*180/pi-samp.att*180/pi);grid on;title('Att Error');xlabel('Seconds');ylabel('Degrees');
figure;plot(samp.t,out.ab,samp.t,repmat(samp.bias.acc,[1,size(samp.t,2)]));grid on; title('a_b');xlabel('Seconds');
figure;plot(samp.t,out.wen-samp.w_E_n);grid on;title('W_E_n Error');xlabel('Seconds');
%figure;plot(samp.t,out.V);grid on;title('V');xlabel('Seconds');