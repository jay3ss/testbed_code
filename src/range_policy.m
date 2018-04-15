function velocity = range_policy(headway, hst, hgo, v_max)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
if 0 <= headway && headway <= hst
  velocity = 0;
elseif hst < headway && headway < hgo
  velocity = 0.5 * (1- cos(pi*(headway - hst)/(hgo - hst)));
else
  velocity = v_max;
end

