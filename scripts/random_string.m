function ans = random_string( str_length )
% 2018-08-31 LLW
% source credit:
% https://stackoverflow.com/questions/8918051/how-can-i-generate-a-random-string-in-matlab
%
% available chars
s = 'ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789';

%find number of random characters to choose from
numRands = length(s); 

%generate random string
ans = s( ceil(rand(1,str_length)*numRands) );
