function [x_FP] = NewtonRaphson_Poincare(x_initial,mapped_state,pStanceFoot,kp,kd,tSpan,options1,options2)
% NewtonRaphson_Poincare - computes the fixed points of the Poincare Return
% Map for a set section

% Note that x_initial is a 6x1 vector. It is missing the state that is constant
% for each Poincare section

% Initialize Variables
global angleSwitch
tol = 1e-4;
e = 0.8;            % For Damped Newton Raphson algorithm step size control
x_old = x_initial;  % Should be the size of the state
perturb = 0.01;
LargestDiff = 1000;  % Just initialize to some large value so while loop will start

% Iterate
while LargestDiff > tol
    % Compute the Jacobian of the Poincare about the old state
    J_Poincare = PoincareJacobian(x_old,mapped_state,perturb,pStanceFoot,kp,kd,tSpan,options1,options2);   % Returns a 5x5
    dFdx = eye(length(x_old)-1) - J_Poincare;
    
    % Compute the Poincare map on the old state
    P_xold = Poincare_ReturnMap(x_old,mapped_state,pStanceFoot,kp,kd,tSpan,options1,options2);
    Fx = x_old - P_xold';
    Fx(mapped_state) = [];
    
    % Compute new state
    x_old_small = x_old; x_old_small(mapped_state)=[];
    x_new = x_old_small - e*inv(dFdx)*Fx;
    
    % Compute largest diff between states
    LargestDiff = 0;
    for i=1:length(x_new)
        tempDiff = abs(x_new(i)-x_old_small(i));
        if (tempDiff > LargestDiff)
           LargestDiff = tempDiff; 
        end
    end
    
    % Update the state
    x_old_small = x_new ; 
    x_old = [angleSwitch;x_old_small]; % Resize to make it a 6x1
end

x_FP = x_new;

end

