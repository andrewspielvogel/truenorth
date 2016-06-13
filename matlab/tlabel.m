function ans=taxis(unix_time)
% Mar 23 2007 LLW Companion to taxis.
%                 Gives unit label for labeling axis
% Example:
% plot(taxis(dvl.unix_time),dvl.position(:,1))
% xlabel(tlabel(dvl.unix_time))
%

dt = max(unix_time) - min(unix_time);

if (dt < (60*10))
    ans =  'SECONDS';
else if (dt < (60*120))
        ans = 'MINUTES';
    else if (dt < (60*60*24))
            ans = 'HOURS';
        else
                ans = 'DAYS';
        end    
    end
end
        
