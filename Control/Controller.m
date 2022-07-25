function [u,y,dy,s] = Controller(x,q1Switch,h_alpha)
% Controller - Compute input-output linearization control values

%% Initialize Variables
kp = 66;
kd = 13;
% kp = 0;
% kd = 0;
[D,Cdq,G,B] = LagrangeModel_3link(x);

q1=x(1); q2=x(2); q3=x(3); dq1=x(4); dq2=x(5); dq3=x(6);
q = [q1; q2; q3];
dq = [dq1; dq2; dq3];
dim_q = length(x)/2 ;
h1_alpha = h_alpha(1:6);
h2_alpha = h_alpha(7:length(h_alpha));

%% Define y and dy
% Compute phasing variable s
th_plus = -q1Switch ;                   % q1Switch is the stance leg angle at EOS
th_minus = q1Switch ;
const = 1/(th_minus - th_plus);
s = [const 0 0]*q + (-th_plus)*const;   % s = (q1-th_plus)/(th_minus - th_plus)
dsdq = [const 0 0];
ds = dsdq*dq;

% hd_h1 - desired h1 output trajectory ("Torso")
hd_h1 = bezier(h1_alpha',s); % bezier coefficients must be row vectors

% hd_h2 - desired h2 output trajectory ("q2")
hd_h2 = bezier(h2_alpha',s);

% h0 calculation - output directly from the state
h0_h1 = q3 ;        % Torso Angle
h0_h2 = q2 ;        % q2

% Calculate y
y1 = h0_h1 - hd_h1;
y2 = h0_h2 - hd_h2;
y = [y1 ; y2];

% Calculate dy
dh0_h1 = dq3;
dh0_h2 = dq2;

dhd_h1 = dbezier(h1_alpha',s)*ds;   % chain rule
dhd_h2 = dbezier(h2_alpha',s)*ds;   % h_alpha should be 1x2M vector

dy1 = dh0_h1 - dhd_h1;
dy2 = dh0_h2 - dhd_h2;

dy = [dy1; dy2];

%% Computed control torques
H = Cdq + G;

x1=x(1); x2=x(2); x3=x(3); x4=x(4); x5=x(5); x6=x(6);
h_alpha1=h_alpha(1); h_alpha2=h_alpha(2); h_alpha3=h_alpha(3); h_alpha4=h_alpha(4);
h_alpha5=h_alpha(5); h_alpha6=h_alpha(6); h_alpha7=h_alpha(7); h_alpha8=h_alpha(8);
h_alpha9=h_alpha(9); h_alpha10=h_alpha(10); h_alpha11=h_alpha(11); h_alpha12=h_alpha(12);

[Jh,J_Jhdq] = Symb_Jh(x1,x2,x3,x4,x5,x6,q1Switch,h_alpha1,h_alpha2,h_alpha3,h_alpha4,h_alpha5,h_alpha6,h_alpha7,h_alpha8,h_alpha9,h_alpha10,h_alpha11,h_alpha12);

Ju = Jh*inv(D)*B;
fdfwd = inv(Ju)*(Jh*inv(D)*H - J_Jhdq);
fdbk = inv(Ju);
ddy = -kd*dy - kp*y;        % PD control on the output dynamics
v = ddy;
u =  fdfwd + fdbk*v;        % Input-Output feedback linearization

end
