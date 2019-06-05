function [De, Ce_dqe, Ge, Be] = LagrangeModelextend_3link(x)
% LagrangeModelextend_3link - Computes the extended coefficients of the
% Lagrange model for the 3-link walker

% Model Parameters
[g,m,m_H,m_T,r,r_T,m_Total] = ModelParam_3link() ;

% Define the generalized coordinates
dim_q = length(x)/2 ;
q = x(1:dim_q) ;
dq = x(dim_q+1:end) ; 
th1 = q(1) ;
th2 = q(2) ;
th3 = q(3) ;
dth1 = dq(1) ;
dth2 = dq(2) ;
dth3 = dq(3) ;

% Compute extended mass-inertia matrix
De = zeros(5,5) ;
De(1,1) = (r^2*(5*m + 4*m_H + 4*m_T))/4 ;
De(1,2) = -(m*r^2*cos(th1 - th2))/2 ;
De(1,3) = m_T*r*r_T*cos(th1 - th3) ;
De(1,4) = (r*cos(th1)*(3*m + 2*m_H + 2*m_T))/2 ;
De(1,5) = -(r*sin(th1)*(3*m + 2*m_H + 2*m_T))/2 ;
De(2,2) = (m*r^2)/4;
De(2,3) = 0;
De(2,4) = -(m*r*cos(th2))/2 ;
De(2,5) = (m*r*sin(th2))/2 ;
De(3,3) = m_T*r_T^2 ;
De(3,4) = m_T*r_T*cos(th3) ;
De(3,5) = -m_T*r_T*sin(th3) ;
De(4,4) = 2*m + m_H + m_T ;
De(4,5) = 0 ;
De(5,5) = 2*m + m_H + m_T ;
De(2,1) = De(1,2) ;
De(3,1) = De(1,3) ;
De(3,2) = De(2,3) ;
De(4,1) = De(1,4) ;
De(4,2) = De(2,4) ;
De(4,3) = De(3,4) ;
De(5,1) = De(1,5) ;
De(5,2) = De(2,5) ;
De(5,3) = De(3,5) ;
De(5,4) = De(4,5) ;

% Compute extended Ce*dqe term
Ce_dqe = zeros(5,1) ;
Ce_dqe(1,1) = m_T*r*r_T*sin(th1 - th3)*dth3^2 - (dth2^2*m*r^2*sin(th1 - th2))/2 ;
Ce_dqe(2,1) = (dth1^2*m*r^2*sin(th1 - th2))/2 ;
Ce_dqe(3,1) = -dth1^2*m_T*r*r_T*sin(th1 - th3) ;
Ce_dqe(4,1) = (m*r*sin(th2)*dth2^2)/2 - m_T*r_T*sin(th3)*dth3^2 - (dth1^2*r*sin(th1)*(3*m + 2*m_H + 2*m_T))/2 ;
Ce_dqe(5,1) = (m*r*cos(th2)*dth2^2)/2 - m_T*r_T*cos(th3)*dth3^2 - (dth1^2*r*cos(th1)*(3*m + 2*m_H + 2*m_T))/2 ;


% G extended
Ge = zeros(5,1) ;
Ge(1,1) = -(g*r*sin(th1)*(3*m + 2*m_H + 2*m_T))/2 ;
Ge(2,1) = (g*m*r*sin(th2))/2 ;
Ge(3,1) = -g*m_T*r_T*sin(th3) ;
Ge(4,1) = 0;
Ge(5,1) = g*(2*m + m_H + m_T) ;

% B extended
Be = [-1 0 ; 0 -1 ; 1 1 ; 0 0 ; 0 0] ;

end

