function samp = gen_samp(w_dir,a_dir,num_samples,w_sig,a_sig)
%function for generating samples
%
% Andrew Spielvogel
% andrewspielvogel@gmail.com
%
% August 2015
%

% normalize w, a directions
w_pure = w_dir/norm(w_dir);
a_pure = a_dir/norm(a_dir);

% earth rate degrees/second
earth_rate = 15/3600;

% bias sigs
w_bias_sig = .1/3600;
a_bias_sig = .05/1000;


% generate samples
samp.ang = earth_rate*repmat(w_pure,num_samples,1) + normrnd(0,w_bias_sig,num_samples,3) + normrnd(0,w_sig,num_samples,3);
samp.acc =            repmat(a_pure,num_samples,1) + normrnd(0,a_bias_sig,num_samples,3) + normrnd(0,a_sig,num_samples,3);

% switch to rad/s
samp.ang = samp.ang*pi/180;