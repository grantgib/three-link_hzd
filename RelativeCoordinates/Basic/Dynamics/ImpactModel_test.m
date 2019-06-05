function [q_plus,dq_plus] = ImpactModel_test(x_minus)
% IMPACTMODEL_3LINK - computes the values of x after impact
% Note: Remember the general system has 7 equations and 7 unknowns

%% Initialize variables
dim_q = length(x_minus)/2 ;
q_min = x_minus(1:dim_q) ;
th1 = q_min(1) ; th2 = q_min(2) ; th3 = q_min(3) ;
dq_min = x_minus(dim_q+1:end) ;
[g,m,m_H,m_T,r,r_T] = ModelParam_3link() ;

%% Compute mapping matrix
del_q = [0 1 0; 1 0 0; 0 0 1] ;

del_dq = zeros(3,3) ;
del_dq(1,1) = -(m^3*cos(3*th1 - 3*th2) - 8*m_H^3*cos(th1 - th2) - m^3*cos(th1 - th2) + 2*m^2*m_H*cos(3*th1 - 3*th2) + m^2*m_T*cos(3*th1 - 3*th2) + 6*m*m_T^2*cos(th1 + th2 - 2*th3) + 4*m^2*m_T*cos(th1 + th2 - 2*th3) + 4*m_H*m_T^2*cos(th1 + th2 - 2*th3) + 4*m_H^2*m_T*cos(th1 + th2 - 2*th3) + m*m_T^2*cos(th1 - 3*th2 + 2*th3) + m*m_T^2*cos(th2 - 3*th1 + 2*th3) - m*m_T^2*cos(th1 + 3*th2 - 4*th3) - m*m_T^2*cos(3*th1 + th2 - 4*th3) - 20*m*m_H^2*cos(th1 - th2) - 10*m^2*m_H*cos(th1 - th2) - 6*m*m_T^2*cos(th1 - th2) - 5*m^2*m_T*cos(th1 - th2) - 4*m_H*m_T^2*cos(th1 - th2) - 12*m_H^2*m_T*cos(th1 - th2) + 12*m*m_H*m_T*cos(th1 + th2 - 2*th3) + 2*m*m_H*m_T*cos(th1 - 3*th2 + 2*th3) + 2*m*m_H*m_T*cos(th2 - 3*th1 + 2*th3) - 20*m*m_H*m_T*cos(th1 - th2))/(m^3*cos(4*th1 - 4*th2) - 5*m^3*cos(2*th1 - 2*th2) + 22*m*m_H^2 + 16*m^2*m_H + 5*m*m_T^2 + 8*m^2*m_T + 4*m_H*m_T^2 + 12*m_H^2*m_T + 4*m^3 + 8*m_H^3 - 4*m*m_H^2*cos(2*th1 - 2*th2) - 12*m^2*m_H*cos(2*th1 - 2*th2) + m*m_T^2*cos(2*th1 - 2*th2) - 6*m^2*m_T*cos(2*th1 - 2*th2) - 2*m*m_T^2*cos(2*th1 - 2*th3) - m^2*m_T*cos(2*th1 - 2*th3) - 6*m*m_T^2*cos(2*th2 - 2*th3) - 4*m^2*m_T*cos(2*th2 - 2*th3) + m*m_T^2*cos(4*th2 - 4*th3) - 4*m_H*m_T^2*cos(2*th2 - 2*th3) - 4*m_H^2*m_T*cos(2*th2 - 2*th3) + 22*m*m_H*m_T + m*m_T^2*cos(2*th1 + 2*th2 - 4*th3) + 2*m^2*m_T*cos(2*th1 - 4*th2 + 2*th3) + m^2*m_T*cos(2*th2 - 4*th1 + 2*th3) - 4*m*m_H*m_T*cos(2*th1 - 2*th2) - 4*m*m_H*m_T*cos(2*th1 - 2*th3) - 12*m*m_H*m_T*cos(2*th2 - 2*th3)) ;
del_dq(1,2) = -(m*(4*m*m_H + 2*m*m_T + 2*m_H*m_T - m^2*cos(2*th1 - 2*th2) + m^2 + 2*m_H^2 - m*m_T*cos(2*th1 - 2*th3) - m*m_T*cos(2*th2 - 2*th3)))/(m^3*cos(4*th1 - 4*th2) - 5*m^3*cos(2*th1 - 2*th2) + 22*m*m_H^2 + 16*m^2*m_H + 5*m*m_T^2 + 8*m^2*m_T + 4*m_H*m_T^2 + 12*m_H^2*m_T + 4*m^3 + 8*m_H^3 - 4*m*m_H^2*cos(2*th1 - 2*th2) - 12*m^2*m_H*cos(2*th1 - 2*th2) + m*m_T^2*cos(2*th1 - 2*th2) - 6*m^2*m_T*cos(2*th1 - 2*th2) - 2*m*m_T^2*cos(2*th1 - 2*th3) - m^2*m_T*cos(2*th1 - 2*th3) - 6*m*m_T^2*cos(2*th2 - 2*th3) - 4*m^2*m_T*cos(2*th2 - 2*th3) + m*m_T^2*cos(4*th2 - 4*th3) - 4*m_H*m_T^2*cos(2*th2 - 2*th3) - 4*m_H^2*m_T*cos(2*th2 - 2*th3) + 22*m*m_H*m_T + m*m_T^2*cos(2*th1 + 2*th2 - 4*th3) + 2*m^2*m_T*cos(2*th1 - 4*th2 + 2*th3) + m^2*m_T*cos(2*th2 - 4*th1 + 2*th3) - 4*m*m_H*m_T*cos(2*th1 - 2*th2) - 4*m*m_H*m_T*cos(2*th1 - 2*th3) - 12*m*m_H*m_T*cos(2*th2 - 2*th3)) ;
del_dq(1,3) = 0 ;
del_dq(2,1) = -(2*m^3*cos(4*th1 - 4*th2) - 5*m^3*cos(2*th1 - 2*th2) - 8*m_H^3*cos(2*th1 - 2*th2) + 2*m*m_H^2 + 6*m^2*m_H - m*m_T^2 + 3*m^2*m_T + 3*m^3 - 24*m*m_H^2*cos(2*th1 - 2*th2) - 20*m^2*m_H*cos(2*th1 - 2*th2) + 2*m^2*m_H*cos(4*th1 - 4*th2) - 5*m*m_T^2*cos(2*th1 - 2*th2) - 10*m^2*m_T*cos(2*th1 - 2*th2) + 5*m*m_T^2*cos(2*th1 - 2*th3) + 3*m^2*m_T*cos(2*th1 - 2*th3) + m*m_T^2*cos(2*th2 - 2*th3) + m^2*m_T*cos(4*th1 - 4*th2) - m*m_T^2*cos(4*th1 - 4*th3) - 4*m_H*m_T^2*cos(2*th1 - 2*th2) - 12*m_H^2*m_T*cos(2*th1 - 2*th2) + 4*m_H*m_T^2*cos(2*th1 - 2*th3) + 4*m_H^2*m_T*cos(2*th1 - 2*th3) + 2*m*m_H*m_T - m*m_T^2*cos(2*th1 + 2*th2 - 4*th3) + m*m_T^2*cos(2*th1 - 4*th2 + 2*th3) + m*m_T^2*cos(2*th2 - 4*th1 + 2*th3) + 2*m^2*m_T*cos(2*th1 - 4*th2 + 2*th3) + m^2*m_T*cos(2*th2 - 4*th1 + 2*th3) - 24*m*m_H*m_T*cos(2*th1 - 2*th2) + 10*m*m_H*m_T*cos(2*th1 - 2*th3) + 2*m*m_H*m_T*cos(2*th2 - 2*th3) + 2*m*m_H*m_T*cos(2*th1 - 4*th2 + 2*th3) + 2*m*m_H*m_T*cos(2*th2 - 4*th1 + 2*th3))/(m^3*cos(4*th1 - 4*th2) - 5*m^3*cos(2*th1 - 2*th2) + 22*m*m_H^2 + 16*m^2*m_H + 5*m*m_T^2 + 8*m^2*m_T + 4*m_H*m_T^2 + 12*m_H^2*m_T + 4*m^3 + 8*m_H^3 - 4*m*m_H^2*cos(2*th1 - 2*th2) - 12*m^2*m_H*cos(2*th1 - 2*th2) + m*m_T^2*cos(2*th1 - 2*th2) - 6*m^2*m_T*cos(2*th1 - 2*th2) - 2*m*m_T^2*cos(2*th1 - 2*th3) - m^2*m_T*cos(2*th1 - 2*th3) - 6*m*m_T^2*cos(2*th2 - 2*th3) - 4*m^2*m_T*cos(2*th2 - 2*th3) + m*m_T^2*cos(4*th2 - 4*th3) - 4*m_H*m_T^2*cos(2*th2 - 2*th3) - 4*m_H^2*m_T*cos(2*th2 - 2*th3) + 22*m*m_H*m_T + m*m_T^2*cos(2*th1 + 2*th2 - 4*th3) + 2*m^2*m_T*cos(2*th1 - 4*th2 + 2*th3) + m^2*m_T*cos(2*th2 - 4*th1 + 2*th3) - 4*m*m_H*m_T*cos(2*th1 - 2*th2) - 4*m*m_H*m_T*cos(2*th1 - 2*th3) - 12*m*m_H*m_T*cos(2*th2 - 2*th3)) ;
del_dq(2,2) = -(m*(m^2*cos(th1 - th2) + 4*m_H^2*cos(th1 - th2) - m^2*cos(3*th1 - 3*th2) - m*m_T*cos(th1 - 3*th2 + 2*th3) - m*m_T*cos(th2 - 3*th1 + 2*th3) + 8*m*m_H*cos(th1 - th2) + 4*m*m_T*cos(th1 - th2) + 4*m_H*m_T*cos(th1 - th2) - 2*m*m_T*cos(th1 + th2 - 2*th3)))/(m^3*cos(4*th1 - 4*th2) - 5*m^3*cos(2*th1 - 2*th2) + 22*m*m_H^2 + 16*m^2*m_H + 5*m*m_T^2 + 8*m^2*m_T + 4*m_H*m_T^2 + 12*m_H^2*m_T + 4*m^3 + 8*m_H^3 - 4*m*m_H^2*cos(2*th1 - 2*th2) - 12*m^2*m_H*cos(2*th1 - 2*th2) + m*m_T^2*cos(2*th1 - 2*th2) - 6*m^2*m_T*cos(2*th1 - 2*th2) - 2*m*m_T^2*cos(2*th1 - 2*th3) - m^2*m_T*cos(2*th1 - 2*th3) - 6*m*m_T^2*cos(2*th2 - 2*th3) - 4*m^2*m_T*cos(2*th2 - 2*th3) + m*m_T^2*cos(4*th2 - 4*th3) - 4*m_H*m_T^2*cos(2*th2 - 2*th3) - 4*m_H^2*m_T*cos(2*th2 - 2*th3) + 22*m*m_H*m_T + m*m_T^2*cos(2*th1 + 2*th2 - 4*th3) + 2*m^2*m_T*cos(2*th1 - 4*th2 + 2*th3) + m^2*m_T*cos(2*th2 - 4*th1 + 2*th3) - 4*m*m_H*m_T*cos(2*th1 - 2*th2) - 4*m*m_H*m_T*cos(2*th1 - 2*th3) - 12*m*m_H*m_T*cos(2*th2 - 2*th3)) ;
del_dq(2,3) = 0 ;
del_dq(3,1) = (r*(2*m^3*cos(3*th1 - 4*th2 + th3) - 4*m^3*cos(2*th2 - 3*th1 + th3) + m^3*cos(4*th2 - 5*th1 + th3) + 7*m^3*cos(th1 - th3) + 8*m_H^3*cos(th1 - th3) - 6*m^3*cos(th1 - 2*th2 + th3) - 8*m_H^3*cos(th1 - 2*th2 + th3) - 2*m*m_T^2*cos(3*th1 - 3*th3) - m^2*m_T*cos(3*th1 - 3*th3) - 24*m*m_H^2*cos(th1 - 2*th2 + th3) - 22*m^2*m_H*cos(th1 - 2*th2 + th3) - 10*m*m_T^2*cos(th1 - 2*th2 + th3) - 15*m^2*m_T*cos(th1 - 2*th2 + th3) - 8*m_H*m_T^2*cos(th1 - 2*th2 + th3) - 16*m_H^2*m_T*cos(th1 - 2*th2 + th3) - 4*m*m_H^2*cos(2*th2 - 3*th1 + th3) - 10*m^2*m_H*cos(2*th2 - 3*th1 + th3) + 2*m^2*m_H*cos(3*th1 - 4*th2 + th3) + 2*m*m_T^2*cos(2*th2 - 3*th1 + th3) - 4*m^2*m_T*cos(2*th2 - 3*th1 + th3) + 2*m*m_T^2*cos(th1 - 4*th2 + 3*th3) + 2*m^2*m_T*cos(th1 - 4*th2 + 3*th3) + 3*m^2*m_T*cos(3*th1 - 4*th2 + th3) + 24*m*m_H^2*cos(th1 - th3) + 22*m^2*m_H*cos(th1 - th3) + 8*m*m_T^2*cos(th1 - th3) + 14*m^2*m_T*cos(th1 - th3) + 8*m_H*m_T^2*cos(th1 - th3) + 16*m_H^2*m_T*cos(th1 - th3) + m^2*m_T*cos(2*th2 - 5*th1 + 3*th3) - 2*m*m_H*m_T*cos(3*th1 - 3*th3) - 34*m*m_H*m_T*cos(th1 - 2*th2 + th3) - 2*m*m_H*m_T*cos(2*th2 - 3*th1 + th3) + 2*m*m_H*m_T*cos(th1 - 4*th2 + 3*th3) + 32*m*m_H*m_T*cos(th1 - th3)))/(2*r_T*(m^3*cos(4*th1 - 4*th2) - 5*m^3*cos(2*th1 - 2*th2) + 22*m*m_H^2 + 16*m^2*m_H + 5*m*m_T^2 + 8*m^2*m_T + 4*m_H*m_T^2 + 12*m_H^2*m_T + 4*m^3 + 8*m_H^3 - 4*m*m_H^2*cos(2*th1 - 2*th2) - 12*m^2*m_H*cos(2*th1 - 2*th2) + m*m_T^2*cos(2*th1 - 2*th2) - 6*m^2*m_T*cos(2*th1 - 2*th2) - 2*m*m_T^2*cos(2*th1 - 2*th3) - m^2*m_T*cos(2*th1 - 2*th3) - 6*m*m_T^2*cos(2*th2 - 2*th3) - 4*m^2*m_T*cos(2*th2 - 2*th3) + m*m_T^2*cos(4*th2 - 4*th3) - 4*m_H*m_T^2*cos(2*th2 - 2*th3) - 4*m_H^2*m_T*cos(2*th2 - 2*th3) + 22*m*m_H*m_T + m*m_T^2*cos(2*th1 + 2*th2 - 4*th3) + 2*m^2*m_T*cos(2*th1 - 4*th2 + 2*th3) + m^2*m_T*cos(2*th2 - 4*th1 + 2*th3) - 4*m*m_H*m_T*cos(2*th1 - 2*th2) - 4*m*m_H*m_T*cos(2*th1 - 2*th3) - 12*m*m_H*m_T*cos(2*th2 - 2*th3))) ;
del_dq(3,2) = -(m*r*(m^2*cos(2*th1 - 3*th2 + th3) - 2*m^2*cos(th2 - th3) - 4*m_H^2*cos(th2 - th3) + m^2*cos(th2 - 2*th1 + th3) + m*m_T*cos(2*th1 + th2 - 3*th3) - 8*m*m_H*cos(th2 - th3) - 3*m*m_T*cos(th2 - th3) - 4*m_H*m_T*cos(th2 - th3) + m*m_T*cos(3*th2 - 3*th3) + m*m_T*cos(th2 - 2*th1 + th3)))/(2*r_T*(m^3*cos(4*th1 - 4*th2) - 5*m^3*cos(2*th1 - 2*th2) + 22*m*m_H^2 + 16*m^2*m_H + 5*m*m_T^2 + 8*m^2*m_T + 4*m_H*m_T^2 + 12*m_H^2*m_T + 4*m^3 + 8*m_H^3 - 4*m*m_H^2*cos(2*th1 - 2*th2) - 12*m^2*m_H*cos(2*th1 - 2*th2) + m*m_T^2*cos(2*th1 - 2*th2) - 6*m^2*m_T*cos(2*th1 - 2*th2) - 2*m*m_T^2*cos(2*th1 - 2*th3) - m^2*m_T*cos(2*th1 - 2*th3) - 6*m*m_T^2*cos(2*th2 - 2*th3) - 4*m^2*m_T*cos(2*th2 - 2*th3) + m*m_T^2*cos(4*th2 - 4*th3) - 4*m_H*m_T^2*cos(2*th2 - 2*th3) - 4*m_H^2*m_T*cos(2*th2 - 2*th3) + 22*m*m_H*m_T + m*m_T^2*cos(2*th1 + 2*th2 - 4*th3) + 2*m^2*m_T*cos(2*th1 - 4*th2 + 2*th3) + m^2*m_T*cos(2*th2 - 4*th1 + 2*th3) - 4*m*m_H*m_T*cos(2*th1 - 2*th2) - 4*m*m_H*m_T*cos(2*th1 - 2*th3) - 12*m*m_H*m_T*cos(2*th2 - 2*th3))) ;
del_dq(3,3) = 1 ;

%% Output
q_plus = del_q*q_min ;
dq_plus = del_dq*dq_min ;

x_plus = [q_plus;dq_plus] ;

end