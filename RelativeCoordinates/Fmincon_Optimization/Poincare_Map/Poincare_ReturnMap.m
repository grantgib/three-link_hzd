function [P_x] = Poincare_ReturnMap(P_init,mapped_state,pStanceFoot,kp,kd,tSpan,options1,options2)
% Poincare_ReturnMap - computes the return map value of the state given an
% initial condition

% Use ode45 solver (IC at section) until impact has occured
[t2,x2,t_minus,x_minus] = ode45(@(t,x) StateSpaceModel_3link(t,x,pStanceFoot,kp,kd), tSpan, P_init, options2) ; 
x_minus = x_minus.' ;        % state space values right before impact

% Impact Map
[q_plus,dq_plus] = ImpactModel_3link(x_minus) ;
x_init = [q_plus ; dq_plus] ;

% Continue solver to compute dynamics until reaching the Poincare
% section
[t1,x1,t_Px,P_x] = ode45(@(t,x) StateSpaceModel_3link(t,x,pStanceFoot,kp,kd), tSpan, x_init, options1) ;

end