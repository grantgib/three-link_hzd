function [value,isterminal,direction] = EventImpact_3link(t,x,impactAngle)
% Detects Event when Stance leg has reached a certain angle. This function
%   is called when setting the options with the odeset function

dim_q = length(x)/2 ;
q = x(1:dim_q) ;
q1 = q(1) ; q2 = q(2) ; q3 = q(3) ;
q = [q1 q2 q3].';
      
% Compute phasing variable s
th_begin = impactAngle ;
th_end = -impactAngle ;
s = (q1 - th_begin)/(th_end - th_begin) ;
value(1) = s - 1 ;

isterminal(1) = 1;  % 1 if the integration is to terminate at a zero
direction(1) = 1;   % A value of +1 locates only zeros where the event
                    %   function is increasing
end

