function [value,isterminal,direction] = OptimEvent_3Link(t,x,q1Switch)
% Detects Event when Stance leg has reached a certain angle. This function
%   is called when setting the options with the odeset function
                    
%% Method 2
dim_q = length(x)/2 ;
q = x(1:dim_q) ;
q1 = q(1) ; q2 = q(2) ; q3 = q(3) ;
q = [q1 q2 q3].';
      
% Compute phasing variable s
th_begin = -q1Switch ;
th_end = q1Switch ;
s = (q1 - th_begin)/(th_end - th_begin) ;
value(1) = s - 1 ;

% if (s == 1) OR (p4L <= 0)
%     value(1) = 0
% else
%     value(0) = 1 ;
% end                                                                  
                                       
%% Other options                                    
isterminal(1) = 1;  % 1 if the integration is to terminate when value = 0
direction(1) = 0;   % A value of +1 locates only zeros where the event
                    %   function is increasing
end