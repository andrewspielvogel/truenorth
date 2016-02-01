function R = get_R_delta(samp,R0)
R0 = reshape(R0,[9,1]);
w_m = samp.ang;
t = samp.t;

S.w=w_m;
S.t=t;


[~, Rs] = ode45(@R_delta_ode, t, R0, [], S);

R = cellfun(@(A) reshape(A,3,3),num2cell(Rs,2),'UniformOutput',false)';






function dR = R_delta_ode(t, R, S)

w_est = zeros(3,1);
w_est(1) = interp1(S.t,S.w(:,1),t);
w_est(2) = interp1(S.t,S.w(:,2),t);
w_est(3) = interp1(S.t,S.w(:,3),t);
R = reshape(R,[3,3]);
dR = R*skew(w_est);
dR = reshape(dR,[9,1]);

