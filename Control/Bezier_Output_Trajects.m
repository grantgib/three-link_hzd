function [y,dy,s] = Bezier_Output_Trajects(x,q1Switch,h_alpha)
% BEZIER_OUTPUT_TRAJECTS - defines the output trajectory as a function of
% Bezier coefficients and the current state

%% Define coordinates
dimq = 3;
q = x(1:dimq); dq = x(dimq+1:end) ;
q1 = q(1) ; q2 = q(2) ; q3 = q(3) ;
dq1 = dq(1) ; dq2 = dq(2) ; dq3 = dq(3) ;

%% [OPTIMIZATION] Extract Bezier Coefficients
if isempty(h_alpha)
    pause
else
    h1_alpha = h_alpha(1:6);
    h2_alpha = h_alpha(7:length(h_alpha));
end

%% Compute phasing variable s
th_plus = -q1Switch ; % q1Switch is the stance leg angle at EOS
th_minus = q1Switch ;
const = 1/(th_minus - th_plus);
s = [const 0 0]*q + (-th_plus)*const;
ds = [const 0 0]*dq;

%% hd_h1 - desired h1 output trajectory ("Torso")
hd_h1 = bezier(h1_alpha',s); % bezier coefficients must be row vectors

%% hd_h2 - desired h2 output trajectory ("q2")
hd_h2 = bezier(h2_alpha',s);

%% h0 calculation - output directly from the state
h0_h1 = q3 ;        % Torso Angle
h0_h2 = q2 ;        % q2

%% Output results
% Calculate y
y1 = h0_h1 - hd_h1;
y2 = h0_h2 - hd_h2;
y = [y1 ; y2];

% Calculate dy
dh0_h1 = dq3;
dh0_h2 = dq2;

dhd_h1 = dbezier(h1_alpha',s)*ds;   %chain rule
dhd_h2 = dbezier(h2_alpha',s)*ds;   % h_alpha should be 1x2M vector

dy1 = dh0_h1 - dhd_h1;
dy2 = dh0_h2 - dhd_h2;

dy = [dy1; dy2];

end

