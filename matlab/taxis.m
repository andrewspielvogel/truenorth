function ans=taxis(unix_time)
% Mar 23 2007 LLW Computes time axis for plotting
%                 in units of time since midnight on first 
%                 day of the data
%  If data is less than 10 min long, x axis is seconds since midnight
%  otherwise 
%  if data is less than 120 min long, x axis is minutes since midnight
%  otherwise 
%  if data is less than 2 days long, x axis is hours since midnight
%  otherwise 
%  x axis is days since midnight

% compute second vector since midnight
dt = max(unix_time) - min(unix_time);

if (dt < (60*10))
    ans =  ssm_noclash(unix_time);
else if (dt < (60*120))
        ans = msm(unix_time);
    else if (dt < (60*60*24))
            ans = hsm(unix_time);
        else
                ans = dsm(unix_time);
        end    
    end
end
        
