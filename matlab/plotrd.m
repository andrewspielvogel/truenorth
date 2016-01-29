function f = plotrd(data)

cut_freq_hz = 1;
samp.stamp = data.stamp';
samp.acc = my_lowpass(data.acc,data.hz,1,cut_freq_hz);
samp.ang = my_lowpass(data.ang,data.hz,1,cut_freq_hz);

Rd   = get_R_d(samp,eye(3));
d.R_in = Rd;
d.t = samp.stamp;
plotrph(d,d);