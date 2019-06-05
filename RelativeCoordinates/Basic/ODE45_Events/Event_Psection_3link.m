function [value,isterminal,direction] = Event_Psection_3link(t,x,Poincare_angle)
% Detects Event when Stance leg has reached a certain angle. This function
%   is called when setting the options with the odeset function. When this
%   event is called the integration should not stop since it is not an 
%   impact event. This event is used to gather data for the Poincare 
%   section for th1 = constant

dim_q = length(x)/2 ;
q = x(1:dim_q) ;
th1_current = q(1) ;    % Should start out as -pi/8 and increase over time
% angleSwitch = 0 ;   % Switch stance with swing when stance reaches this
                    %   angle
value(1) = th1_current - Poincare_angle ; % Stop integration when the stance
                                       % leg reaches angleSwitch
isterminal(1) = 1 ; % 1 if the integration is to terminate at a zero.
direction(1) = 1 ;  % A value of +1 locates only zeros where the event
                    %   function is increasing

end

