function [q_plus,dq_plus] = ImpactModel_Simple(x_minus)

%% Initialize variables
dim_q = length(x_minus)/2 ;
q_min = x_minus(1:dim_q) ;
th1 = q_min(1) ; th2 = q_min(2) ; th3 = q_min(3) ;
dq_min = x_minus(dim_q+1:end) ;

%% Output
Swap_Matrix = [0 1 0 0 0 0; 1 0 0 0 0 0; 0 0 1 0 0 0; 0 0 0 1 0 0; 0 0 0 0 1 0; 0 0 0 0 0 1];
x_plus = Swap_Matrix*x_minus;

q_plus = x_plus(1:3);
dq_plus = x_plus(4:6);




end

