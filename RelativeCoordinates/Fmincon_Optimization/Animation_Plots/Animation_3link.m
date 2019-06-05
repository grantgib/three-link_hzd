function [] = Animation_3link(t,x,StanceFoot)
% Animation_3link - plots the 3-link walker moving with lines drawn between
%   different positions (i.e. stance leg, swing leg, etc.)

%% Set defaults for plot
clf
color_Stance = 'r' ; color_Swing = 'b' ; color_Torso = 'g' ;
xLow = -1 ; xUp = 20 ; yLow = -.2 ; yUp = 1.5 ;
xlim([xLow xUp]) ; ylim([yLow yUp]) ;
hline = refline(0,0);
set(hline,'LineWidth',3) ;
%% Initialize the linkage lines
[xRow,xCol]=size(x) ;
dim_q=xCol/2 ;
q = x(1,1:dim_q) ;
pStanceFoot = StanceFoot(1,:).' ;
[pHip,pTorso,pSwingFoot] = Points_3link(x(1,:),pStanceFoot) ;
link_Stance = line([pStanceFoot(1) pHip(1)],[pStanceFoot(2) pHip(2)]) ;
link_Swing = line([pHip(1) pSwingFoot(1)],[pHip(2) pSwingFoot(2)]) ;
link_Torso = line([pHip(1) pTorso(1)],[pHip(2) pTorso(2)]) ;

set(link_Stance,'LineWidth',2,'Color',color_Stance) ;
set(link_Swing,'LineWidth',2,'Color',color_Swing) ;
set(link_Torso,'LineWidth',2,'Color',color_Torso) ;

%% Draw the linkages for each state space iteration
for j = 1:xRow
    
    % Calculate points for each iteration
    q=x(j,1:dim_q);
    dq=x(j,dim_q+1:end);
    pStanceFoot = StanceFoot(j,:).' ;
    [pHip,pTorso,pSwingFoot] = Points_3link(x(j,:),pStanceFoot) ;
    
    % Using the set method allows you to redraw the lines without erasing
    % the previous lines on the screen
    set(link_Stance,'XData',[pStanceFoot(1) pHip(1)],...
        'YData',[pStanceFoot(2) pHip(2)]);    
    set(link_Swing,'XData',[pHip(1) pSwingFoot(1)],...
        'YData',[pHip(2) pSwingFoot(2)]);    
    set(link_Torso,'XData',[pHip(1) pTorso(1)],...
        'YData',[pHip(2) pTorso(2)]);
    
    drawnow ;       % Draws the newly updated lines onto the figure
%     pause(0.005) ;
%     pause
end
end