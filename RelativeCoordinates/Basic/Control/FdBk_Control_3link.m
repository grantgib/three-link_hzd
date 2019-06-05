function [u,y,dy,s] = FdBk_Control_3link(x,kp,kd)
% FDBK_CONTROL_3LINK - Compute input-output linearization control values

%% Initialize Variables
myGlobalVariables

[g,m,m_H,m_T,r,r_T] = ModelParam_3link() ;
dim_q = length(x)/2 ;
q = x(1:dim_q) ;
dq = x(dim_q+1:end) ;

q1 = q(1) ;
q2 = q(2) ;
q3 = q(3) ;
dq1 = dq(1) ;
dq2 = dq(2) ;
dq3 = dq(3) ;

%% Define output goals
% Mirror Law
% y = [th3 - th3d ; th1 + th2] ; % Goal y -> [0 ; 0]
% dy = [dth3 ; dth1 + dth2] ;
% s = [];
% Bezier Output Trajectories
[y,dy,s] = Bezier_Output_Trajects(x);

%% Compute some more control terms
A = zeros(2,2) ;
A(1,1) = (3*m*r + 4*m_H*r + 4*m_T*r + 4*m_T*r_T*cos(q1 - q3) - 2*m*r*cos(2*q1 - 2*q2))/(m_T*r*r_T^2*(3*m + 4*m_H + 2*m_T - 2*m*cos(2*q1 - 2*q2) - 2*m_T*cos(2*q1 - 2*q3))) ;
A(1,2) = (3*m*r + 4*m_H*r + 4*m_T*r + 4*m_T*r_T*cos(q2 - q3) - 2*m*r*cos(2*q1 - 2*q2) + 4*m_T*r_T*cos(q2 - 2*q1 + q3))/(m_T*r*r_T^2*(3*m + 4*m_H + 2*m_T - 2*m*cos(2*q1 - 2*q2) - 2*m_T*cos(2*q1 - 2*q3))) ;
A(2,1) = -(4*r_T + 4*r*cos(q2 - 2*q1 + q3) + 4*r*cos(q1 - q3) + 4*r*cos(q2 - q3) + 8*r_T*cos(q1 - q2))/(r^2*r_T*(3*m + 4*m_H + 2*m_T - 2*m*cos(2*q1 - 2*q2) - 2*m_T*cos(2*q1 - 2*q3))) ;
A(2,2) = -(20*m*r_T + 16*m_H*r_T + 8*m_T*r_T + 4*m*r*cos(q1 - q3) + 4*m*r*cos(q2 - q3) + 8*m*r_T*cos(q1 - q2) - 8*m_T*r_T*cos(2*q1 - 2*q3) + 4*m*r*cos(q2 - 2*q1 + q3))/(m*r^2*r_T*(3*m + 4*m_H + 2*m_T - 2*m*cos(2*q1 - 2*q2) - 2*m_T*cos(2*q1 - 2*q3))) ;

xi = zeros(2,1) ;
xi(1,1) = (g*m*sin(q3) + 2*g*m_H*sin(q3) + 2*g*m_T*sin(q3) - ...
    g*m*sin(2*q1-2*q2+q3) - 2*g*m*sin(2*q1-q3) + g*m*sin(2*q2-q3) -...
    2*g*m_H*sin(2*q1-q3) - 2*g*m_T*sin(2*q1-q3) + 3*dq1^2*m*r*...
    sin(q1-q3) + dq2^2*m*r*sin(q2-q3) + 4*dq1^2*m_H*r*sin(q1-q3) +...
    4*dq1^2*m_T*r*sin(q1-q3) + 2*dq3^2*m_T*r_T*sin(2*q1-2*q3) +...
    2*dq1^2*m*r*sin(q1-2*q2+q3) + dq2^2*m*r*sin(q2-2*q1+q3))/...
    (r_T*(3*m+4*m_H+2*m_T-2*m*cos(2*q1-2*q2)-2*m_T*cos(2*q1-2*q3))) ;
xi(2,1) = (4*g*m*sin(q1) - 4*g*m*sin(q2) + 4*g*m_H*sin(q1) - ...
    4*g*m_H*sin(q2) + 2*g*m_T*sin(q1) - 2*g*m_T*sin(q2) + ...
    2*g*m_T*sin(2*q1+q2-2*q3) + 2*g*m*sin(q1-2*q2) + ...
    2*g*m_T*sin(q1-2*q3) + 2*g*m_T*sin(q2-2*q3) + 6*g*m*sin(2*q1-q2) +...
    4*g*m_H*sin(2*q1-q2) + 2*g*m_T*sin(2*q1-q2) - ...
    10*dq1^2*m*r*sin(q1-q2) + 2*dq2^2*m*r*sin(q1-q2) - ...
    8*dq1^2*m_H*r*sin(q1-q2) - 4*dq1^2*m_T*r*sin(q1-q2) - ...
    4*dq3^2*m_T*r_T*sin(q1-q3) - 4*dq3^2*m_T*r_T*sin(q2-q3) - ...
    2*dq1^2*m*r*sin(2*q1-2*q2) + 2*dq2^2*m*r*sin(2*q1-2*q2) - ...
    2*dq1^2*m_T*r*sin(2*q1-2*q3) - 4*dq1^2*m_T*r*sin(q1+q2-2*q3) +...
    4*dq3^2*m_T*r_T*sin(q2-2*q1+q3))/(r*(3*m+4*m_H+2*m_T-2*m*...
    cos(2*q1-2*q2)-2*m_T*cos(2*q1-2*q3))) ;
% Symbolic equations calculated before hand

%% Computed control torques
u = inv(A)*(-xi-kd*dy-kp*y) ;  
%u = [0 ; 0] ; % For testing model without a controller
end