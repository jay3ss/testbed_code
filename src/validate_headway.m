function [headway_val, vel_val] = validate_headway(headway, prev_headway, lower_limit, vel, dt)
  if headway < lower_limit
    headway_val = lower_limit;
    vel_val = (prev_headway - lower_limit)/dt;
  else
    headway_val = headway;
    vel_val = vel;
end
