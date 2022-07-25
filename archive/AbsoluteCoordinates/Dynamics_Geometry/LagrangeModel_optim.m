function [D,Cdq,G,B] = LagrangeModel_optim(q, dq)
%% Model Parameters
[g,m,m_H,m_T,r,r_T,m_Total] = ModelParam_3link() ;

%% Define the generalized coordinates
q1 = q(1) ; q2 = q(2) ; q3 = q(3) ;
dq1 = dq(1) ; dq2 = dq(2) ; dq3 = dq(3) ;
q = [q1 q2 q3].'; dq = [dq1 dq2 dq3].';

%% D
D = zeros(3,3) ;
D(1,1) = (r^2*(5*m + 4*m_H + 4*m_T))/4 ;
D(1,2) = -(m*r^2*cos(q1 - q2))/2 ;
D(2,1) = D(1,2) ;
D(2,2) = (m*r^2)/4 ;
D(1,3) = (5*m*r^2)/4 + m_H*r^2 + m_T*r^2 - (m*r^2*cos(q1 - q2))/2 - m_T*r*r_T*cos(q1) ;
D(3,1) = D(1,3) ;
D(2,3) = -(m*r^2*(2*cos(q1 - q2) - 1))/4 ;
D(3,2) = D(2,3) ;
D(3,3) = (3*m*r^2)/2 + m_H*r^2 + m_T*r^2 + m_T*r_T^2 - m*r^2*cos(q1 - q2) - 2*m_T*r*r_T*cos(q1) ;

%% C
C = zeros(3,3) ;
C(1,1) = 0 ;
C(1,2) = -(m*r^2*sin(q1 - q2)*(dq2 + dq3))/2 ;
C(1,3) = - (dq2*m*r^2*sin(q1 - q2))/2 - (dq3*m*r^2*sin(q1 - q2))/2 - dq3*m_T*r*r_T*sin(q1) ;
C(2,1) = (m*r^2*sin(q1 - q2)*(dq1 + dq3))/2 ;
C(2,2) = 0 ;
C(2,3) = (m*r^2*sin(q1 - q2)*(dq1 + dq3))/2 ;
C(3,1) = (r*(dq1 + dq3)*(2*m_T*r_T*sin(q1) + m*r*sin(q1 - q2)))/2 ;
C(3,2) =  -(m*r^2*sin(q1 - q2)*(dq2 + dq3))/2 ;
C(3,3) = (dq1*m*r^2*sin(q1 - q2))/2 - (dq2*m*r^2*sin(q1 - q2))/2 + dq1*m_T*r*r_T*sin(q1) ;

Cdq = C*dq ;
%% G
G = zeros(3,1) ;
G(1,1) = (g*r*sin(q1 + q3)*(3*m + 2*m_H + 2*m_T))/2 ;
G(2,1) = -(g*m*r*sin(q2 + q3))/2 ;
G(3,1) = (g*(3*m*r*sin(q1 + q3) - 2*m_T*r_T*sin(q3) - m*r*sin(q2 + q3) + 2*m_H*r*sin(q1 + q3) + 2*m_T*r*sin(q1 + q3)))/2 ;

%% B
B = [-1 0 ; 0 -1 ; 0 0] ;

end