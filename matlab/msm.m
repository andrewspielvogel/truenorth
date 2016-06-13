function ans = msm(unix_time)
% Mar 23 2007 LLW Computes monotinic minutes since midnight

ans = ssm_noclash(unix_time) * (1/60);
