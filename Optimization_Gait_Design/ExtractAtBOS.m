function [alphas01] = ExtractAtBOS(x_plus,q1Switch)
q1=x_plus(1); q2=x_plus(2); q3=x_plus(3);
dq1=x_plus(4); dq2=x_plus(5); dq3=x_plus(6);
M=5;

q_plus = [q1 q2 q3]';
dq_plus = [dq1 dq2 dq3]';

%q1Switch is the stancle leg angle at the end of step (an initial
%condition)
th_plus = -q1Switch ;
th_minus = q1Switch ;
const = 1/(th_minus - th_plus);
s = [const 0 0]*q_plus + (-th_plus)*const;
ds = [const 0 0]*dq_plus;

% h1 alphas
% hd_dot = dbi_ds*ds_dt = M(a_1 - a_0)*ds_dt = dq_i (@ s = 0)
a0_h1 = q3;
a1_h1 = a0_h1 + dq3/(M*ds);

% h2 alphas
a0_h2 = q2;
a1_h2 = a0_h2 + dq2/(M*ds);

alphas01 = [a0_h1 a1_h1 a0_h2 a1_h2]';

end