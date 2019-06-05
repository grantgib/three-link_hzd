% SYMB_JH_GEN - Generate symbolic code for the controller. Symbolics are
% related to jacobians

%% Symbolic Input Variables
h_alpha = sym('h_alpha', [12 1],'real');
x = sym('x',[6 1],'real');
syms q1Switch real 

% Initialize Variables
q1=x(1); q2=x(2); q3=x(3); dq1=x(4); dq2=x(5); dq3=x(6);
q = [q1; q2; q3];
dq = [dq1; dq2; dq3];
h1_alpha = h_alpha(1:6);
h2_alpha = h_alpha(7:length(h_alpha));

%% Compute phasing variable s
th_plus = -q1Switch ; % q1Switch is the stance leg angle at EOS
th_minus = q1Switch ;
const = 1/(th_minus - th_plus);
s = [const 0 0]*q + (-th_plus)*const;
dsdq = [const 0 0];
dsdt = dsdq*dq;

%% Compute y and dy
% Calculate y
h0_h1 = q3 ;                    % observed output
hd_h1 = bezier(h1_alpha',s);    % desired output 
y1 = h0_h1 - hd_h1;

h0_h2 = q2 ;                    % observed output
hd_h2 = bezier(h2_alpha',s);    % bezier coefficients must be row vectors
y2 = h0_h2 - hd_h2;

y = [y1 ; y2];

% Calculate dy
dh0_h1 = dq3;
dhd_h1 = dbezier(h1_alpha',s)*dsdt;   %chain rule
dy1 = dh0_h1 - dhd_h1;

dh0_h2 = dq2;
dhd_h2 = dbezier(h2_alpha',s)*dsdt;   % h_alpha should be 1x2M vector
dy2 = dh0_h2 - dhd_h2;

dy = [dy1; dy2];

%% Compute Jacobian Symbolic Expressions
h = y;
Jh = jacobian(h,q);

J_Jhdq = jacobian(Jh*dq,q)*dq;
matlabFunction(Jh,J_Jhdq,'file','Symb_Jh.m','vars',...
    {'x1','x2','x3','x4','x5','x6','q1Switch','h_alpha1','h_alpha2',...
    'h_alpha3','h_alpha4','h_alpha5','h_alpha6','h_alpha7','h_alpha8',...
    'h_alpha9','h_alpha10','h_alpha11','h_alpha12',})



