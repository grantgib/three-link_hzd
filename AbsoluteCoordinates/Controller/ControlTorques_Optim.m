function [u,h,dh,s] = ControlTorques_Optim(x)
% ControlTorques_Optim - return the control torque as a function of the
% state and bezier coefficients

%% Global Variables
GlobalVar

%% Model Parameters
[g,m,m_H,m_T,r,r_T] = ModelParam_3link() ;

%% Coordinates
q = x(1:dimq); dq = x(dimq+1:end) ;
q1 = q(1) ; q2 = q(2) ; q3 = q(3) ;
dq1 = dq(1) ; dq2 = dq(2) ; dq3 = dq(3) ;

%% Define output goals
[h,dh,s] = h_dh_optim(x) ;     % Gait optimization condition

%% Compute some more control terms
LgLf = zeros(2,2) ;
LgLf(1,1) = (5*m*r + 4*m_H*r + 4*m_T*r - 4*m_T*r_T*cos(q1) - 4*m*r*cos(q1 - q2)^2)/(m_T*r*r_T^2*(5*m + 4*m_H + 4*m_T - 4*m_T*cos(q1)^2 - 4*m*cos(q1 - q2)^2)) ;
LgLf(1,2) = (5*m*r + 4*m_H*r + 4*m_T*r - 4*m*r*cos(q1 - q2)^2 - 8*m_T*r_T*cos(q1 - q2)*cos(q1))/(m_T*r*r_T^2*(5*m + 4*m_H + 4*m_T - 4*m_T*cos(q1)^2 - 4*m*cos(q1 - q2)^2)) ;
LgLf(2,1) = (4*r*cos(2*q1 - q2) - 4*r_T + 4*r*cos(q1) + 4*r*cos(q2) - 8*r_T*cos(q1 - q2))/(r^2*r_T*(3*m + 4*m_H + 2*m_T - 2*m_T*cos(2*q1) - 2*m*cos(2*q1 - 2*q2))) ;
LgLf(2,2) = -(20*m*r_T + 16*m_H*r_T + 8*m_T*r_T - 4*m*r*cos(q1) - 4*m*r*cos(q2) + 8*m*r_T*cos(q1 - q2) - 8*m_T*r_T*cos(2*q1) - 4*m*r*cos(2*q1 - q2))/(m*r^2*r_T*(3*m + 4*m_H + 2*m_T - 2*m_T*cos(2*q1) - 2*m*cos(2*q1 - 2*q2))) ;

Lf2h = zeros(2,1) ;
Lf2h(1,1) = -(g*m*sin(2*q1 - 2*q2 + q3) - g*m*sin(q3) - 2*g*m_H*sin(q3) - 2*g*m_T*sin(q3) + 2*g*m*sin(2*q1 + q3) - g*m*sin(2*q2 + q3) + 2*g*m_H*sin(2*q1 + q3) + 2*g*m_T*sin(2*q1 + q3) + 2*dq1^2*m*r*sin(q1 - 2*q2) + 2*dq3^2*m*r*sin(q1 - 2*q2) - 2*dq3^2*m_T*r_T*sin(2*q1) - dq2^2*m*r*sin(2*q1 - q2) - dq3^2*m*r*sin(2*q1 - q2) + 3*dq1^2*m*r*sin(q1) + dq2^2*m*r*sin(q2) + 3*dq3^2*m*r*sin(q1) + dq3^2*m*r*sin(q2) + 4*dq1^2*m_H*r*sin(q1) + 4*dq3^2*m_H*r*sin(q1) + 4*dq1^2*m_T*r*sin(q1) + 4*dq3^2*m_T*r*sin(q1) + 4*dq1*dq3*m*r*sin(q1 - 2*q2) - 2*dq2*dq3*m*r*sin(2*q1 - q2) + 6*dq1*dq3*m*r*sin(q1) + 2*dq2*dq3*m*r*sin(q2) + 8*dq1*dq3*m_H*r*sin(q1) + 8*dq1*dq3*m_T*r*sin(q1))/(r_T*(3*m + 4*m_H + 2*m_T - 2*m_T*cos(2*q1) - 2*m*cos(2*q1 - 2*q2))) ;
Lf2h(2,1) = -(6*g*m*sin(2*q1 - q2 + q3) - 2*g*m*sin(2*q2 - q1 + q3) + 4*g*m_H*sin(2*q1 - q2 + q3) + 2*g*m_T*sin(2*q1 - q2 + q3) + 2*g*m_T*sin(q1 - q3) + 2*g*m_T*sin(q2 - q3) + 2*g*m_T*sin(2*q1 + q2 + q3) + 4*g*m*sin(q1 + q3) - 4*g*m*sin(q2 + q3) + 4*g*m_H*sin(q1 + q3) - 4*g*m_H*sin(q2 + q3) + 2*g*m_T*sin(q1 + q3) - 2*g*m_T*sin(q2 + q3) + 10*dq1^2*m*r*sin(q1 - q2) - 2*dq2^2*m*r*sin(q1 - q2) + 8*dq3^2*m*r*sin(q1 - q2) + 8*dq1^2*m_H*r*sin(q1 - q2) + 8*dq3^2*m_H*r*sin(q1 - q2) + 4*dq1^2*m_T*r*sin(q1 - q2) + 4*dq3^2*m_T*r*sin(q1 - q2) + 2*dq1^2*m_T*r*sin(2*q1) + 2*dq3^2*m_T*r*sin(2*q1) + 2*dq1^2*m*r*sin(2*q1 - 2*q2) - 2*dq2^2*m*r*sin(2*q1 - 2*q2) - 4*dq3^2*m_T*r_T*sin(2*q1 - q2) + 4*dq1^2*m_T*r*sin(q1 + q2) + 4*dq3^2*m_T*r*sin(q1 + q2) - 4*dq3^2*m_T*r_T*sin(q1) - 4*dq3^2*m_T*r_T*sin(q2) + 20*dq1*dq3*m*r*sin(q1 - q2) - 4*dq2*dq3*m*r*sin(q1 - q2) + 16*dq1*dq3*m_H*r*sin(q1 - q2) + 8*dq1*dq3*m_T*r*sin(q1 - q2) + 4*dq1*dq3*m_T*r*sin(2*q1) + 4*dq1*dq3*m*r*sin(2*q1 - 2*q2) - 4*dq2*dq3*m*r*sin(2*q1 - 2*q2) + 8*dq1*dq3*m_T*r*sin(q1 + q2))/(r*(3*m + 4*m_H + 2*m_T - 2*m_T*cos(2*q1) - 2*m*cos(2*q1 - 2*q2))) ;

%% Computed control torques
ddh = -kd*dh-kp*h ;
u = inv(LgLf)*(-Lf2h + ddh) ;  
%u = [0 ; 0] ; % For testing model without a controller
end