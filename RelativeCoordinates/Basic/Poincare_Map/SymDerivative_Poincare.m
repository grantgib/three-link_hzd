function [dP_i] = SymDerivative_Poincare(x_FP,x_small,mapped_state,delta_vector,perturb,pStanceFoot,kp,kd,tSpan,options1,options2) 
% SymDerivative_Poincare - calculate the symmetric partial derivative of
% the Poincare return map given a fixed point and perturbation

% Create perturbation arguments for derivative
x_plus = x_FP + delta_vector ;
x_minus = x_FP - delta_vector ;

% Compute terms in the numerator (function shown below)
[P_xplus] = Poincare_ReturnMap(x_plus,mapped_state,pStanceFoot,kp,kd,tSpan,options1,options2) ;
[P_xminus] = Poincare_ReturnMap(x_minus,mapped_state,pStanceFoot,kp,kd,tSpan,options1,options2) ;

% Compute derivative and orient vector

dP_i = (P_xplus - P_xminus);

if dP_i == 0
    dP_i = dP_i';
else
    dP_i = dP_i / (2*perturb) ;
    dP_i = dP_i.' ;
end

end
