function [g,m,m_H,m_T,r,r_T,m_Total] = ModelParam_3link()
g = 9.81 ;  % m/s^2
m = 5 ;     % kg
m_H = 15 ;  % kg
m_T = 10 ;  % kg
m_Total = 2*m + m_H + m_T ;   % kg
r = 1.0 ;   % m
r_T = 0.5 ; % m

end


