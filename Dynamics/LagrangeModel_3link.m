function [D,Cdq,G,B] = LagrangeModel_3link(x)
% Model Parameters
[g,m,m_H,m_T,r,r_T,m_Total] = ModelParam_3link() ;

% Define the generalized coordinates
dim_q = length(x)/2 ;
q = x(1:dim_q) ;
dq = x(dim_q+1:end) ;
q1 = q(1) ;
q2 = q(2) ;
q3 = q(3) ;
dq1 = dq(1) ;
dq2 = dq(2) ;
dq3 = dq(3) ;

% D
D = zeros(3,3) ;
D(1,1) = r^2*((5/4)*m + m_H + m_T);
D(1,2) = -(1/2)*m*r^2*cos(q1 - q2);
D(2,1) = D(1,2) ;
D(2,2) = (1/4)*m*r^2;
D(1,3) = m_T*r*r_T*cos(q1 - q3);
D(3,1) = D(1,3) ;
D(2,3) = 0;
D(3,2) = D(2,3) ;
D(3,3) = m_T*r_T^2;

% Cdq
Cdq = zeros(3,1) ;
Cdq(1,1) = m_T*r*r_T*sin(q1-q3)*dq3^2 - (dq2^2*m*r^2*sin(q1-q2))/2 ;
Cdq(2,1) = (dq1^2*m*r^2*sin(q1-q2))/2 ;
Cdq(3,1) = -dq1^2*m_T*r*r_T*sin(q1-q3) ;

% G
G = zeros(3,1) ;
G(1,1) = -(1/2)*g*(2*m_H + 3*m + 2*m_T)*r*sin(q1) ;
G(2,1) = (1/2)*g*m*r*sin(q2) ;
G(3,1) = -g*m_T*r_T*sin(q3) ;

% B
B = [-1 0 ; 0 -1 ; 1 1] ;

end