function samp = gen_samp(w_dir,a_dir,num_samples,w_sig,a_sig)

w_pure = w_dir/norm(w_dir);
a_pure = a_dir/norm(a_dir);
earth_rate = 15/3600;
    
samp.ang = earth_rate*repmat(w_pure,num_samples,1) + normrnd(0,.1/3600,num_samples,3)  + normrnd(0,w_sig,num_samples,3);
samp.acc =            repmat(a_pure,num_samples,1) + normrnd(0,.05/1000,num_samples,3) + normrnd(0,a_sig,num_samples,3);