function [dP] = PoincareJacobian(x,mapped_state,perturb,pStanceFoot,kp,kd,tSpan,options1,options2)
    
dP = [];    % Initialize jacobian
x_small = x;
x_small(mapped_state) = [];              % This is the smaller state vector that doesn't contain the mapped state
for pos = 1:length(x_small) 
    delta_vector = zeros(length(x),1) ;  % Initialize delta vector
    delta_vector(pos) = perturb ;           % Redefine vector for each loop iteration
    [dP_i] = SymDerivative_Poincare(x,x_small,mapped_state,delta_vector,perturb,pStanceFoot,kp,kd,tSpan,options1,options2) ;  % Solve for element of dP
    dP = [dP dP_i] ;                        % Store in dP vector
end
dP(mapped_state,:)=[];
end