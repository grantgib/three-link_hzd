function [pHip,pTorso,pSwingFoot] = Points_optim(x)
% POINTS_3LINK - computes the points of the linkage model based on the
%       current generalized coordinates

global pStanceFoot
%% Model Parameters
[g,m,m_H,m_T,r,r_T,m_Total] = ModelParam_3link() ;

%% Coordinate Transformation
q = x(1:length(x)/2) ;
q1 = q(1) ; q2 = q(2) ; q3 = q(3) ;
q = [q1 q2 q3].';

% Perform coordinate transformation so that you can reuse old code
th = zeros(3,1) ;
th = [1 0 1; 0 1 1; 0 0 1]*q - [pi ; pi ; 0] ;
th1 = th(1); th2 = th(2) ; th3 = th(3) ;

%% Compute positions based on geometry
pHip = pStanceFoot + r*[sin(th1) ; cos(th1)] ;  % "" hip
pTorso = pHip + r_T*[sin(th3) ; cos(th3)] ;     % "" torso
pSwingFoot = pHip + r*[-sin(th2) ; -cos(th2)] ; % "" swing (2) foot
end

