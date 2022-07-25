function [] = BezierPlot(a,q1Switch)
X = []; StanceFoot = [];
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
h_alpha = ReconstructHalpha(h_alpha01,h_alpha23,h_alpha45)

% h1d
h1d = h_alpha(1:6)'; % must be row vector
s = linspace(0,1);
for i = 1:100
    bi(i) = bezier(h1d,s(i)) ;
end
figure(1)
plot(s,bi)
title('q2')
hold on
scatter([0,1/5,2/5,3/5,4/5,5/5],h1d,'g')
title('h1 desired: tracks q3')
xlabel('s'); ylabel('Alpha values');
grid on

% h2d
h2d = h_alpha(7:12)'; % must be row vector
s = linspace(0,1);
for i = 1:100
    bi(i) = bezier(h2d,s(i)) ;
end
figure(2)
plot(s,bi)
title('q2')
hold on
scatter([0,1/5,2/5,3/5,4/5,5/5],h2d,'g')
title('h2 desired: tracks q2')
xlabel('s'); ylabel('Alpha values');
grid on


end




















