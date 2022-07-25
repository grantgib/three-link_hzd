% Symb_Controller_Generator 

%% Define Symbolic Variables
D = sym('D',[3 3],'real');
Cdq = sym('Cdq',[3 1],'real');
G = sym('G',[3 1],'real');
B = sym('B',[3 2],'real');
h_alpha = sym('h_alpha', [12 1],'real');
x = sym('x',[6 1],'real');
syms q1Switch real 

q1=x(1); q2=x(2); q3=x(3); dq1=x(4); dq2=x(5); dq3=x(6);
q = [q1; q2; q3];
dq = [dq1; dq2; dq3];
h1_alpha = h_alpha(1:6);
h2_alpha = h_alpha(7:length(h_alpha));

%% Define y and dy
% Compute phasing variable s
th_plus = -q1Switch ; % q1Switch is the stance leg angle at EOS
th_minus = q1Switch ;
const = 1/(th_minus - th_plus);
s = [const 0 0]*q + (-th_plus)*const;
dsdq = [const 0 0];
dsdt = dsdq*dq;

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

dhd_h1 = dbezier(h1_alpha',s)*dsdt;   %chain rule
dhd_h2 = dbezier(h2_alpha',s)*dsdt;   % h_alpha should be 1x2M vector

dy1 = dh0_h1 - dhd_h1;
dy2 = dh0_h2 - dhd_h2;

dy = [dy1; dy2];

%% Compute control
h = y;
Jh = jacobian(h,q);
H = Cdq + G;
alpha = Jh*inv(D)*B;
fdfwd = inv(alpha)*(Jh*inv(D)*H - (jacobian(Jh*dq,q))*dq);
fdbck = inv(alpha);

matlabFunction(y,dy,s,fdfwd,fdbck,'file','Symb_Controller.m')

% matlabFunction([y,dy,s,fdfwd,fdbck],'file','Symb_Controller.m',...
%     'vars',{'D','Cdq','G','B','x','q1Switch','h_alpha'})


