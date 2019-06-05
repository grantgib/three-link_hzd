function [Cost,Gineq,Geq,u] = CostFunc(state_input)
%COSTFUNC - Output the cost function for optimization


tic
%% Global Variables
global Cost Geq Gineq x0 alphas
global pStanceFoot
global kp kd OptimCondition
global dimq
dimq = 3 ;

%% Initial Conditions
% Extract info from fmincon output
q = state_input(1:dimq) ;
dq = state_input(dimq+1:2*dimq) ;
alphas = state_input(2*dimq+1:end) ;
x_init = [q ; dq] ;
tSpan = 0:.01:2 ;
pStanceFoot = [0 ; 0] ;
kp = 67 ;
kd = 14.5 ;

Time = []; X = []; H = []; DH = []; 
Torques = []; Forces = []; pSwingFoot = []; GRF = [];
T_ReturnMap = [] ; X_ReturnMap = [] ; 
StanceFoot = [] ; S = [] ;

%% ODE solver option
opts = odeset('Events',@Event_EOS,'MaxStep',0.01,'Refine',4,...
    'RelTol',1e-4,'AbsTol',1e-6) ;

%% Use odeSolver to run through entire step
disp(x_init)
[t1,x1,t_end,x_end] = ode45(@(t,x) StateSpaceModel_optim(x),tSpan,x_init,opts) ;
[pHip,pTorso,pSwingFoot] = Points_optim(x_end) ;
StepLength = pSwingFoot(1) - pStanceFoot(1) ;

[q_plus,dq_plus] = ImpactModel_optim(x_end) ;
x_postImpact = [q_plus;dq_plus] ;
X = [X ; x1] ;
Time = [Time ; t1] ;


%% Reconstruct Dynamics - to compute cost
dt = diff(t1) ;      % Needed for IntegralSqTorque -> cost
IntegralSqTorque = 0 ;
for k = 1:length(t1)-1
    %Sum integral of torque^2 for cost function
    xiter = x1(k,:) ;
    [u,h,dh,s] = ControlTorques_Optim(xiter) ;
    IntegralSqTorque = IntegralSqTorque + norm(u)^2*dt(k) ;
    [pHip,pTorso,pSwingEnd] = Points_optim(x_end) ;
    
    %% Ground Reaction Forces (GRF)
%     [De, Ce_dqe, Ge, Be] = LagrangeModelextend_optim(x) ;
%     E1 = [0 0 0 1 0 ; 0 0 0 0 1] ;
%     GRF = GRF_optim(De,Ce_dqe,Ge,Be,E1,u) ;
%     Ft = GRF(1) ; Fn = GRF(2) ;
    %% Store Data
    H = [H h] ;
    DH = [DH dh] ;
    Torques = [Torques u] ;
    S = [S s] ;

end
% [u_Impact,h_Impact,dh_Impact,s_Impact] = ControlTorques_Optim(x_postImpact);
% IntegralSqTorque = IntegralSqTorque + norm(u_Impact)^2*.01 ;
% H = [H h_Impact] ;
% DH = [DH dh_Impact] ;
% Torques = [Torques u_Impact] ;
% S = [S s_Impact] ;
%% Constraints
Geq = [] ;
Gineq = [];
% Gineq(1) = abs(u(1)) - 30 ; % Torque constrain ui < 5
% Gineq(2) = abs(pSwingEnd(2)) - 0.01 ;
% Gineq(3) = abs(q(2) - q(1)) - pi/4 ; % q2 - q1 < pi/2
% Gineq(3) = u(2) - 5 ;
% Gineq(4) = max(abs(Ft/Fn)) - 0.6 ; % Nonslip |F_T| <= 0.6*|F_N|

%% Cost
Cost =  IntegralSqTorque/max(0.1,StepLength) ;

%% Animation Check
% figure(1) 
% StanceFoot = zeros(length(x1),2) ;
% Animation_3link(x1,StanceFoot) ;

%% Clean and Plot data
H = H.' ;
DH = DH.' ;
Torques = Torques.' ;
S = S.' ;

% figure(1)
% subplot(2,3,1) ;
% plot(Time,H(:,1)) ;
% title('y1-torso') ;
% hold on
% subplot (2,3,2) ;
% plot(Time, Torques(:,1)) ;
% title('u1') ;
% hold on
% subplot (2,3,4) ;
% plot(Time, H(:,2)) ;
% title('y2-legs') ;
% hold on
% subplot (2,3,5) ;
% plot(Time, Torques(:,2)) ;
% title('u2') ;
% hold on
toc
end

