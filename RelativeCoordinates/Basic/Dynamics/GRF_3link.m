function [GRF_iter] = GRF_3link(De,Ce_dqe,Ge,Be,E1,u)
%GRF_3link - Computes the Ground Reaction Force of the 3-link walker during
%the swing phase

GRF_iter1 = inv(E1*inv(De)*E1.') ;
GRF_iter2 = E1*(inv(De)*(Ce_dqe + Ge - Be*u)) ;
GRF_iter3 = 0 ; % The third term is zero due to how the world coordinates are defined

GRF_iter = GRF_iter1*(GRF_iter2 - GRF_iter3) ;
end

