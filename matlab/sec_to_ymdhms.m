function [year,month,day,hour,minute,second] = sec_to_ymdhms(secs_from_1970) 
%
% This function calculates the number of seconds from Jan-01-1970 to the 
% time specified by the args
% 
% Matlab datenum and datevec use year 0 as ref, but we will use Jan 1 1970 
% as ref to be compatible w/ unix convention.
% 
% Note: returns year as 1997, not 97, and 2002, not 02 
%
%  August 1997 G. Lerner, cteated and written
%
%

% t1 = datenum(1970,1,1);

t1=719529;   % this is the date number for Jan 1 1970

[year,month,day,hour,minute,second]  = datevec(t1 + (secs_from_1970/(24*3600)));

