% SIMULATION_3LINK_OPTIMIZATION - animated simulation showing one step of
% the optimized gait to minimize energy from joint torques

clear all
close all
restoredefaultpath;
addpath(genpath('.'));

%% INITIALIZE VARIABLES
% Global Variables
myGlobalVariables % Cost Geq Gineq alphas X0 angleSwitch=pi/16 counter

% Starting with end of step (x_minus)
x_ic = [pi/8 pi/10 1.6 -1.4 0]';    % q2 not included and is set = -q1 in cost

% load the optimized gait
% load('OptimGait');
% X0 = X_SAVE;

% Cost = 305.174
% X0 = [0.3038
%     0.2383
%     1.1260
%    -0.6731
%    -0.0268
%     0.1652
%     0.1525
%     0.6283
%    -0.0132];

% Cost = 240.0974. satisfies torque <50. Speed > 0.5. Ft/Fn < 0.6.
X0=[0.2980
    0.2466
    1.2839
   -0.1215
   -0.7131
    0.1462
    0.0090
    0.4718
    0.3510]

% General Variables
dimq = 3;
q1Switch = X0(1);

% ODE solver parameters
options = odeset('Events',@(t,x) OptimEvent_3Link(t,x,q1Switch),'MaxStep',0.1,'Refine',4,'RelTol',10^-5,'AbsTol',10^-6);
tspan = [0 2];

% Variables to be collected and save
Time = []; X = []; Y = []; DY = []; 
Torques = []; Forces = []; pSwingFoot = []; GRF = [];
StanceFoot = [] ; S = [] ; 
FT =[]; FN = [];

tic
%% SIMULATE STEP (INCLUDING IMPACT)

% Extract Parameters at End of Step
[q_minus,dq_minus,h_alpha45] = ExtractAtEOS(X0,q1Switch);
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

% Set alpha parameters
h_alpha01 = ExtractAtBOS(x_plus,q1Switch);
h_alpha23 = X0(6:length(X0));
h_alpha = ReconstructHalpha(h_alpha01,h_alpha23,h_alpha45);
% BezierPlot(X0);

% Initial Condition
x_init = [q_plus; dq_plus];

% ODE Solver
[t1,x1,t_end,x_end] = ode45(@(t,x) StateSpaceModel_3link(t,x,pStanceFoot,q1Switch,h_alpha),tspan,x_init,options);

[pHip,pTorso,pSwingEnd] = Points_3link(x_end,pStanceFoot) ;
StepLength = pSwingEnd(1) - pStanceFoot(1) ;
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
    IntegralSqTorque = IntegralSqTorque + norm(u)^2*dt(k) ;
   
    %% Ground Reaction Forces
    [De,Ce_dqe,Ge,Be] = LagrangeModelextend_3link(xiter);
    E1 = [0 0 0 1 0 ; 0 0 0 0 1] ;
    GRF = GRF_3link(De,Ce_dqe,Ge,Be,E1,u) ;
    Ft = GRF(1);
    Fn = GRF(2);
   
   %% Store Data
   Y = [Y y] ;
   DY = [DY dy] ;
   Torques = [Torques u] ;
   S = [S s] ;
   FT = [FT Ft];
   FN = [FN Fn];
   StanceFoot = [StanceFoot pStanceFoot] ;

end

%% CLEAN DATA
Y = Y.' ;
DY = DY.' ;
Torques = Torques.' ;
S = S.' ;
FT = FT';
FN = FN';
pSwing = [pSwing ;pSwingEnd'];
StanceFoot = StanceFoot.' ;

%% COST
Cost = IntegralSqTorque / max(0.1/StepLength);

toc
%% ANIMATION + PLOTS
Animation_3link(t1,X,StanceFoot)
