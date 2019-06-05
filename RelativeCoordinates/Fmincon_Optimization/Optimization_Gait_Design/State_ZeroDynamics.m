function [q_init,dq_init] = State_ZeroDynamics(x_plus,q1Switch,h_alpha)
% State_ZeroDynamics - calculates what x_init should be in order to keep
% the output on the zero dynamics manifold

%% [OPTIMIZATION] Extract Bezier Coefficients
if isempty(h_alpha)
    pause
else
    h1_alpha = h_alpha(1:6);
    h2_alpha = h_alpha(7:length(h_alpha));
end

% Phasing variable is zero at beginning of step
s = 0 ;
dimq = 3;
q_plus = x_plus(1:dimq); dq_plus = x_plus(dimq+1:end) ;
th_begin = -q1Switch ;
th_end = q1Switch ;
const = 1/(th_end - th_begin);
s = [const 0 0]*q_plus + (-th_begin)*const;


% Set y = dy = 0 so that state is on zero dynamics manifold
y1=0; y2=0; dy1=0; dy2=0;

% hd_h1 - desired h1 output trajectory ("Torso")
hd_h1 = bezier(h1_alpha',s); % bezier coefficients must be row vectors
dhd_h1 = dbezier(h1_alpha',s)*[const 0 0]*dq_plus;   %chain rule

% hd_h2 - desired h2 output trajectory ("q2")
hd_h2 = bezier(h2_alpha',s);
dhd_h2 = dbezier(h2_alpha',s)*[const 0 0]*dq_plus;   % h_alpha should be 1x2M vector

% Calculate the initial states
q3 =hd_h1;
dq3 =dhd_h1;

q2 =hd_h2;
dq2 =dhd_h2;

% Rearrange input states
q_init = [-q2; q2; q3];
dq_init = [-dq2; dq2; dq3];

end
















