function [Cost,Gineq,Geq] = Cost_Torque(a)
% COST_TORQUE - Computes the energy required for the actuated motor torques

%% INITIALIZE VARIABLES
% Global Variables
myGlobalVariables
q1Switch = a(1);    % Used to end ode45 solver

% General Variables
dimq = 3;
pStanceFoot = [0;0];
[g,m,m_H,m_T,r,r_T,m_Total] = ModelParam_3link();

% See how many times the function is called
counter = counter+1;

% ODE options
options = odeset('Events',@(t,x) OptimEvent_3Link(t,x,q1Switch),'MaxStep',0.1,'Refine',4,'RelTol',10^-3,'AbsTol',10^-4);

% Simulation Parameters
tspan = [0 5];

% Variables to be collected and save
Time = []; X = []; Y = []; DY = []; 
Torques = []; Forces = []; pSwingFoot = []; GRF = [];
StanceFoot = [0;0] ; S = [] ; 
FT =[]; FN = [];
FR = [];
% tic
%% SIMULATE STEP (INCLUDING IMPACT)
% Extract Parameters at End of Step
[q_minus,dq_minus,h_alpha45] = ExtractAtEOS(a,q1Switch);
x_minus = [q_minus; dq_minus];
pStanceFoot = [0 0]';
StanceFoot = [StanceFoot pStanceFoot];
[pHip,pTorso,pSwingFoot_end] = Points_3link(x_minus,pStanceFoot) ;
X = [X x_minus];
X = X';

% Apply Impact
[q_plus,dq_plus] = ImpactModel_3link(x_minus);
x_plus = [q_plus; dq_plus];
pStanceFoot = pSwingFoot_end;
StanceFoot = [StanceFoot pStanceFoot];

h_alpha01 = ExtractAtBOS(x_plus,q1Switch);
h_alpha23 = a(6:length(a));
h_alpha = ReconstructHalpha(h_alpha01,h_alpha23,h_alpha45);
x_init = [q_plus; dq_plus];

% ODE Solver
[t1,x1,t_end,x_end] = ode45(@(t,x) StateSpaceModel_3link(t,x,pStanceFoot,q1Switch,h_alpha),tspan,x_init,options);

[pHip,pTorso,pSwingEnd] = Points_3link(x_end,pStanceFoot) ;
StepLength = pSwingEnd(1) - pStanceFoot(1) ;
avgSpeed = abs(StepLength/t_end);  % velocity >= speed_desired
X = [X ; x1] ;
Time = [Time ; t1] ;

%% RECONSTRUCT DYNAMICS 
dt = diff(t1);
IntegralSqTorque = 0;
pSwing = zeros(length(t1)-1,2);
for k = 1:length(t1)-1
    xiter = x1(k,:);
    [dx,q,dq,y,dy,u,pSwingiter,GRF_iter,s] = StateSpaceModel_3link(t1(k),xiter',pStanceFoot,q1Switch,h_alpha);
    pSwing(k,:) = pSwingiter';
    IntegralSqTorque = IntegralSqTorque + (norm(u)^2*dt(k)/max(0.1,StepLength));          % minimization of torques only
%     IntegralSqTorque = IntegralSqTorque + (norm(u)^2*dt(k) / max(0.1,StepLength)) + 1e5/avgSpeed;   % Find a maximum speed for the robot
   
    %% Ground Reaction Forces
    [De,Ce_dqe,Ge,Be] = LagrangeModelextend_3link(xiter);
    E1 = [0 0 0 1 0 ; 0 0 0 0 1] ;
    GRF = GRF_3link(De,Ce_dqe,Ge,Be,E1,u) ;
    Ft = GRF(1);
    Fn = GRF(2);
   
    %% Froude number ( ratio of the centripetal force around the center of motion, the foot, and the weight of the animal walking)
    % Fr = v^2/(g*l)
    [pHip,pTorso,pSwingFoot] = Points_3link(xiter,pStanceFoot);
    Fr = avgSpeed / (9.81*pHip(2));
    
   %% Store Data
   Y = [Y y] ;
   DY = [DY dy] ;
   Torques = [Torques u];
   S = [S s] ;
   FT = [FT Ft];
   FN = [FN Fn];
   FR = [FR Fr];
end

%% CLEAN DATA
Y = Y.' ;
DY = DY.' ;
Torques = Torques.' ;
S = S.' ;
FT = FT';
FN = FN';
pSwing = [pSwing ;pSwingEnd'];
FR = FR';

%% CONSTRAINTS
% Nonlinear Inequalities
Gineq = [];
maxT = 40;

Gineq(1) = max(abs(Torques(:,1))) - maxT; % Torque Constraint
Gineq(2) = max(abs(Torques(:,2))) - maxT;
% Gineq(1) = -1; % Torque Constraint
% Gineq(2) = -1;
Gineq(3) = 0.5 - avgSpeed ;        % Velocity constraint. can currently go up to 0.6
% Gineq(3) = -1;
Gineq(4) = abs(Ft/Fn) - 0.6;                     % GRF friction
% Gineq(5) = -Fn;
% Gineq(6) = Fn - 0.3*m_Total*g;

% Nonlinear Equalities
Geq = [];
Geq(1) = x_end(1) - a(1);   % Periodicity equalities
Geq(2) = x_end(3) - a(2);
Geq(3) = x_end(4) - a(3);
Geq(4) = x_end(5) - a(4);
Geq(5) = x_end(6) - a(5);
Geq(6) = pSwingEnd(2);      % Swing Foot ground equality
% Geq(7) = StepLength - 0.58; % Step length equality

% Save constraints to global variables
const_Gineq = [const_Gineq; Gineq];
const_Geq = [const_Geq; Geq];

%% COST
Cost = IntegralSqTorque;
disp(['Cost = ',num2str(Cost),'  Max Gineq = ', num2str(max(Gineq)),' norm2(Geq) = ',num2str(norm(Geq)),' norm_inf(Geq) = ',num2str(norm(Geq,inf)),'  norm2_mean(Geq) = ',num2str(norm(Geq)/length(Geq))])
% toc

%% ANIMATION + PLOTS
% Animation_3link(t1,X,StanceFoot)

end

