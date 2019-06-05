function [value,isterminal,direction] = Event_EOS(t,x)
% Detects Event when Stance leg has reached a certain angle. This function
%   is called when setting the options with the odeset function

%% Method 1
% dim_q = length(x)/2 ;
% q = x(1:dim_q) ;
% 
% thSt = q(1) ;    % Should start out as -pi/8 and increase over time
% angleSwitch = 23*pi/24 ;    % Switch stance with swing when stance reaches this
%                         %   angle
% value(1) = thSt - angleSwitch ; % Stop integration when the stance
%                                        %    leg reaches angleSwitch
                                       
%% Method 2
dim_q = length(x)/2 ;
q = x(1:dim_q) ;
q1 = q(1) ; q2 = q(2) ; q3 = q(3) ;
q = [q1 q2 q3].';
% Define virtual displacements
thT = q3 ;         
thSt = q1 + q3 ;   
thSw = q2 + q3 ;      
% Compute phasing variable s
th_begin = 17*pi/24 + pi/6 ;
th_end = 23*pi/24 + pi/6 ;
s = (thSt - th_begin)/(th_end - th_begin) ;
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