function [q_minus,dq_minus,alphas45] = ExtractAtEOS(a,q1Switch)
q1=a(1); q2=-a(1); q3=a(2);
dq1=a(3); dq2=a(4); dq3=a(5);
M = 5;

% q and dq
q_minus = [q1 q2 q3]';
dq_minus = [dq1 dq2 dq3]';

th_plus = -q1Switch ; % q1Switch is stance leg angle at end of step
th_minus = q1Switch ;
const = 1/(th_minus - th_plus);
s = [const 0 0]*q_minus + (-th_plus)*const;
ds = [const 0 0]*dq_minus;

% Alphas
% hd_dot = dbi_ds*ds_dt = M(a_M - a_M-1)*ds_dt = dq_i (@ s = 1)
a5_h1 = q3;
a4_h1 = a5_h1 - dq3/(M*ds);

a5_h2 = q2;
a4_h2 = a5_h2 - dq2/(M*ds);

alphas45 = [a4_h1 a5_h1 a4_h2 a5_h2]';

end