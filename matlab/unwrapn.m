function ans = unwrapn(x,n)

% function ans = unwrapn(x, n)
% argument: x    is an nx1 vector of bit unsigned integer encoder values
% argument: n    is the  word length of the encoder.
%                e.g. for a 16 bit encoder, use n=16.
% return:    ans is a vector whose first element is the first element of x.
%                and remaining elements are unwrapped absolute values for
%                the vector x
%
% 29 May 98 LLW
%
% example:
% example:  » z = (1:10)' * 20000
% example:  
% example:  z =
% example:  
% example:         20000
% example:         40000
% example:         60000
% example:         80000
% example:        100000
% example:        120000
% example:        140000
% example:        160000
% example:        180000
% example:        200000
% example:  
% example:  » z = mod(z,2^16)
% example:  
% example:  z =
% example:  
% example:         20000
% example:         40000
% example:         60000
% example:         14464
% example:         34464
% example:         54464
% example:          8928
% example:         28928
% example:         48928
% example:          3392
% example:  
% example:  » unwrapn(z,16)
% example:  
% example:  ans =
% example:  
% example:         20000
% example:         40000
% example:         60000
% example:         80000
% example:        100000
% example:        120000
% example:        140000
% example:        160000
% example:        180000
% example:        200000
% example:  
% example:  
% example:   end of example


len          = length(x);

MODULUS      = 2^n;

% MAX_UNSIGNED = (2^n)-1;     % is  65,535 for 16 bit unsigned int
% MIN_UNSIGNED = 0;           % is  0      for 16 bit unsigned int

MAX_SIGNED   = (2^(n-1)-1); % is  32,767 for 16 bit signed int
MIN_SIGNED   = -(2^(n-1));  % is -32,768 for 16 bit signed int

delta = diff(x);

delta = delta  - (((delta > MAX_SIGNED) - (delta < MIN_SIGNED)) * MODULUS);

ans(1:1)     = x(1);
ans(2:len,1) = delta;
ans          = cumsum(ans);

