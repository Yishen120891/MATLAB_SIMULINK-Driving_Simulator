function [ Kplus ] = Kplus_val( Kplus_r , Kplus_psiL , Kplus_yL , Kplus_deltad , x )
%KPLUS_VAL Summary of this function goes here
%   Detailed explanation goes here
Kplus = [ polyval(Kplus_r , x) polyval(Kplus_psiL , x) polyval(Kplus_yL , x) polyval(Kplus_deltad , x)];

end

