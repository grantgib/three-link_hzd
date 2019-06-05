function [h,dh,s] = h_dh_optim(x)

%% Global Variables
GlobalVar

% alphas = [-0.01;-0.01;0;0] ;       % Constant alphas condition 

%% Define coordinates
q = x(1:dimq); dq = x(dimq+1:end) ;
q1 = q(1) ; q2 = q(2) ; q3 = q(3) ;
dq1 = dq(1) ; dq2 = dq(2) ; dq3 = dq(3) ;
% Define virtual displacements
thT = q3 ;          dthT = dq3 ;
thSt = q1 + q3 ;    dthSt = dq1 + dq3 ;
thSw = q2 + q3 ;    dthSw = dq2 + dq3 ;  

%% Compute phasing variable s
th_begin = 17*pi/24 + pi/6 ;
th_end = 23*pi/24 + pi/6 ;
s = (thSt - th_begin)/(th_end - th_begin) ;

%% Extract Alpha values
a2_1 = alphas(1) ;
a3_1 = alphas(2) ;
a2_2 = alphas(3) ;
a3_2 = alphas(4) ;

%% h calculation
% h should be of the form h(q) = h0 - b_i. 
% Where b_i is the bezier polynomial

% h1 - "Torso"
% Virtual constraint output
h0_1 = thT - pi/6 ;
% Bezier Polynomial
slope = 0 ; M = 5 ;
a0 = 0 ;                a5 = a0 ;
a1 = (slope/M) + a0 ;   a4 = a5 - (slope/M) ; 
a2 = a2_1 ;             a3 = a3_1 ;
hd_1 = bezier([a0 a1 a2 a3 a4 a5],s) ;
% h1 output
h1 = h0_1 - hd_1 ;

% h2 - "Mirror Law"
% Virtual constraint output
h0_2 = wrapToPi(thSt + thSw) ;
% Bezier Polynomial
slope = -.1 ; M = 5 ;
a0 = 0 ;                a5 = a0 ;
a1 = (slope/M)+a0 ;     a4 = a5 - (slope/M) ; 
a2 = a2_2 ;             a3 = a3_2 ;
hd_2 = bezier([a0 a1 a2 a3 a4 a5],s) ;

% h2 output
h2 = h0_2 - hd_2 ;

%% Results
h = [h1 ; h2] ;
dh = [dthT ; dthSt + dthSw] ;

end