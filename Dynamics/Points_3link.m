function [pHip,pTorso,pSwingFoot] = Points_3link(x,pStanceFoot)
% POINTS_3LINK - computes the points of the linkage model based on the
%       current generalized coordinates

%% Initialize variables
[g,m,m_H,m_T,r,r_T,m_Total] = ModelParam_3link() ;
dimq = 3;
q = x(1:dimq); dq = x(dimq+1:end) ;
q1 = q(1) ; % stance
q2 = q(2) ; % swing 
q3 = q(3) ; % torso
dq1 = dq(1) ; dq2 = dq(2) ; dq3 = dq(3) ;

%% Compute positions based on geometry
pHip = pStanceFoot + r*[sin(q1) ; cos(q1)] ;  % "" hip
pTorso = pHip + r_T*[sin(q3) ; cos(q3)] ;     % "" torso
pSwingFoot = pHip + r*[-sin(q2) ; -cos(q2)] ; % "" swing (2) foot
end

