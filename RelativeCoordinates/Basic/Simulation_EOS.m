%% Main Simulation_3link 
% by Grant
% Simulation for the simple 3-link walker with straight knees.
% Simulates starting with beginning of step

clear all
close all
addpath(genpath('Animation_Plots'))
addpath(genpath('Control'))
addpath(genpath('Dynamics'))
addpath(genpath('ODE45_Events'))

% Starting with end of step (x_minus)
x_ic = [pi/8 pi/6 1.6 -1.4 0]';    % q2 not included and is set = -q1 in cost
freeAlphas = [0 0 0 0]'; % alpha 2,3 for y1,y2 are free can be changed by optimization
X0 = [x_ic; freeAlphas];

X0 = [0.2274
    0.1558
    0.9720
   -1.3943
    0.0582
    0.0347
    0.1797
    0.4782
   -0.1097]
% 
% Cost = 57.0823
X0 = [0.1555
    0.0581
    0.6011
   -1.0009
    0.0666
   -0.0324
    0.0649
    0.4488
   -0.0393]


% General Variables
dimq = 3;
q1Switch = X0(1);

% ODE options
options = odeset('Events',@(t,x) OptimEvent_3Link(t,x,q1Switch),'MaxStep',0.1,'Refine',4,'RelTol',10^-5,'AbsTol',10^-6);

% Simulation Parameters
tspan = 0:0.01:5;

% Variables to be collected and save
Time = []; X = []; Y = []; DY = []; 
Torques = []; Forces = []; pSwing = []; GRF = [];
StanceFoot = [] ; S = [] ; 
FT =[]; FN = [];

tic
%% SIMULATE STEP (INCLUDING IMPACT)
n = 5;  % # of steps
q_minus = [X0(1); -X0(1); X0(2)];
dq_minus = [X0(3); X0(4); X0(5)];
x_minus = [q_minus; dq_minus];
pStanceFoot = [0 0]';
StanceFoot = [StanceFoot pStanceFoot];
X = [X x_minus];
for i = 1:n
% Extract Parameters at End of Step
[pHip,pTorso,pSwingFoot_end] = Points_3link(x_minus,pStanceFoot) ;
% Apply Impact
[q_plus,dq_plus] = ImpactModel_3link(x_minus);
x_plus = [q_plus; dq_plus];
pStanceFoot = pSwingFoot_end;
StanceFoot = [StanceFoot pStanceFoot];

% Initial Condition
x_init = [q_plus; dq_plus];

% ODE Solver
q1Switch = x_minus(1);
[t1,x1,t_end,x_end] = ode45(@(t,x) StateSpaceModel_3link(t,x,pStanceFoot,q1Switch),tspan,x_init,options);

[pHip,pTorso,pSwingEnd] = Points_3link(x_end,pStanceFoot) ;
StepLength = pSwingEnd(1) - pStanceFoot(1) ;
X = [X x1'];
Time = [Time t1'] ;

%% RECONSTRUCT DYNAMICS 
dt = diff(t1);
IntegralSqTorque = 0;
% pSwing = zeros(length(t1)-1,2);
    for k = 1:length(t1)-1
        xiter = x1(k,:);
        [u,y,dy,s] = Controller(xiter',q1Switch); 
        [pHip,pTorso,pSwingiter] = Points_3link(xiter,pStanceFoot) ;
%         pSwing(k,:) = pSwingiter';
        IntegralSqTorque = IntegralSqTorque + norm(u)^2*dt(k) ;

        %% Ground Reaction Forces
        [De,Ce_dqe,Ge,Be] = LagrangeModelextend_3link(xiter);
        E1 = [0 0 0 1 0 ; 0 0 0 0 1] ;
        GRF = GRF_3link(De,Ce_dqe,Ge,Be,E1,u) ;
        Ft = GRF(1);
        Fn = GRF(2);

       %% Store Data
       S = [S s];
       Y = [Y y];
       DY = [DY dy];
       Torques = [Torques u];
       FT = [FT Ft];
       FN = [FN Fn];
       StanceFoot = [StanceFoot pStanceFoot];
       pSwing = [pSwing pSwingiter];
    end
%% Prepare for next loop
x_minus = x_end';

end

%% CLEAN DATA
X = X.';
Time = [Time Time(end)+0.05];
Time = Time.';
Y = Y.' ;
DY = DY.' ;
Torques = Torques.' ;
S = S.' ;
FT = FT';
FN = FN';
pSwing = pSwing';
StanceFoot = StanceFoot.' ;

%% COST
Cost = IntegralSqTorque / max(0.1/StepLength);

toc
%% ANIMATION + PLOTS
Animation_3link(t1,X,StanceFoot)


% Plot state space variables
ss = true ;
PlotOutput_3link(ss,Time,X,Y,DY,S,Torques,pSwing,GRF) ;

% Plot other output data
ss = false ;
PlotOutput_3link(ss,Time,X,Y,DY,S,Torques,pSwing,GRF) ;


