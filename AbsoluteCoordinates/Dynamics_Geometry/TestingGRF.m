% Testing GRF
clear
clc

xcheck = [0;0;0;0;0;0] ;
[De, Ce_dqe, Ge, Be] = LagrangeModelextend_3link(xcheck) ;
E1 = [0 0 0 1 0 ; 0 0 0 0 1] ;
u = [0;0];
GRF_iter = GRF_3link(De,Ce_dqe,Ge,Be,E1,u) 