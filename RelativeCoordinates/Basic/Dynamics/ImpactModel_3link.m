function [q_plus,dq_plus] = ImpactModel_3link(x_minus)
% IMPACTMODEL_3LINK - computes the values of x after impact
% Note: Remember the general system has 7 equations and 7 unknowns

%% Initialize variables
dim_q = length(x_minus)/2 ;
q_minus = x_minus(1:dim_q) ;
q1 = q_minus(1) ; q2 = q_minus(2) ; q3 = q_minus(3) ;
dq_min = x_minus(dim_q+1:end) ;
[g,m,m_H,m_T,r,r_T] = ModelParam_3link() ;

%% Compute mapping matrix
del_q = [0 1 0; 1 0 0; 0 0 1] ;

del_dq = zeros(3,3) ;
den = -3*m-4*m_H-2*m_T+2*m*cos(2*q1-2*q2)+2*m_T*cos(-2*q2+2*q3) ;
del_dq(1,1) = (1/den)*(2*m_T*cos(-q1+2*q3-q2) - ...
    (2*m+4*m_H+2*m_T)*cos(q1-q2)) ;
del_dq(1,2) = m/den ;
del_dq(1,3) = 0 ;
del_dq(2,1) = (1/den)*(m - (4*m+4*m_H+2*m_T)*cos(2*q1-2*q2) + ...
    2*m_T*cos(2*q1-2*q3)) ;
del_dq(2,2) = (1/den)*2*m*cos(q1 - q2) ;
del_dq(2,3) = 0 ;
del_dq(3,1) = (r/(r_T*den))*((2*m+2*m_H+2*m_T)*cos(q3+q1-2*q2) - ...
    (2*m+2*m_H+2*m_T)*cos(-q1+q3) + m*cos(-3*q1+2*q2+q3)) ;
del_dq(3,2) = -(r/(r_T*den))*m*cos(-q2+q3) ;
del_dq(3,3) = 1 ;

%% Output
q_plus = del_q*q_minus ;
dq_plus = del_dq*dq_min ;

x_plus = [q_plus;dq_plus] ;

end

