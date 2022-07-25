function [h_alpha] = ReconstructHalpha(halpha01,halpha23,halpha45)
    h1_alpha = [halpha01(1:2); halpha23(1:2); halpha45(1:2)];
    h2_alpha = [halpha01(3:4); halpha23(3:4); halpha45(3:4)];
    h_alpha = [h1_alpha; h2_alpha];
end