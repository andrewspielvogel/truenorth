function foo = my_lowpass(x, sample_freq_hz, filter_order, cutoff_freq_hz)

%
% function ans = lowpass(x, sample_freq_hz, filter_order, cutoff_freq_hz)
%
% Function for lowpass filtering signal X, of frequency "sample_freq_hz"
% with a lowpass filter of order "filter_order", with
% cutoff frequency of cutoff_freq_hz
%
% Data in X are arranged as column vectors
%
% 10 Feb 2004 LLW Created and Written


wn =  cutoff_freq_hz / (0.5 * sample_freq_hz);

if( wn > 1.0)
    fprintf(1,'LOWPASS.M: Error: the cutoff frequency cannot exceed 1/2 the sample frequency.\n');
    fprintf(1,'LOWPASS.M: Error: you have requested a cutoff frequency of %f HZ.\n',cutoff_freq_hz);
    fprintf(1,'LOWPASS.M: Error: for a sample frequency of %f HZ.\n', sample_freq_hz);
    return;
end

% create nth order butterworth filter
[B,A] = butter(filter_order,  wn, 'low');


% filter the colums of X
num_rows = length(x(:,1));
num_cols = length(x(1,:));

fprintf(1,'LOWPASS.M: %d order butterworth filter. samples at %g HZ, cutoff freq at %g HZ\n',filter_order, sample_freq_hz, cutoff_freq_hz);
fprintf(1,'LOWPASS.M: input data has %d samples in %d columns, wn=%g\n',num_rows, num_cols,wn);
 
% pre-allocate output matrix
foo = zeros(num_rows,num_cols);
for(col = 1:num_cols)
  foo(:,col) = filtfilt(B,A,x(:,col));
  %     foo(:,col) = filter(B,A,x(:,col));
end