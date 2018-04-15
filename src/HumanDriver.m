% Human driver model, taken from
% Optimal control of connected vehicle systems
% Jin I. Ge, Gabor Orosz
%
%% Example usage
% Plot the range policy
%
% alpha = 1; beta = 1;
% v_max = 15; h_stop = 5; h_go = 25;
% hv = HumanDriver(v_max, h_stop, h_go, alpha, beta);
% max_test_headway = 30;
% step_size = 0.01;
% headways = 0:step_size:max_test_headway;
% num_its = max_test_headway / step_size + 1;
% ranges = zeros(1, num_its);
%
% for i = 1:num_its
%   hv.headway_ = headways(i);
%   ranges(i) = hv.range_policy();
% end
%
% plot(headways, ranges)


classdef HumanDriver
  properties %(Access = private)
  % Underscore denotes that the property is private
    alpha_
    beta_
    h_go_
    h_stop_
    headway_
    v_max_
  end

  methods
    function obj = HumanDriver(v_max, h_stop, h_go, alpha, beta)
      if nargin == 5
        if isnumeric(v_max) && isnumeric(h_stop) && isnumeric(h_go)
          if h_stop < h_go
            obj.h_go_ = h_go;
            obj.h_stop_ = h_stop;
            obj.v_max_ = v_max;
            obj.alpha_ = alpha;
            obj.beta_ = beta;
          else
            error('h_stop must be less than h_go')
          end
        else
          error('All values must be numeric')
        end
      else
        error('v_max, h_stop, h_go, alpha, & beta must be specified')
      end
    end

    function hdot = h_dot(obj, vel_1, vel)
      % vel_1 (float): velocity of vehicle in front of this vehicle
      % vel (float): current velocity of this vehicle (should probably calculate
      % this within the class)
      hdot = vel_1 - vel;
    end

    function vdot = v_dot(obj, vel_1, vel)
      % vel_1 (float): velocity of vehicle in front of this vehicle
      % vel (float): current velocity of this vehicle (should probably calculate
      % this within the class)
      alpha = obj.alpha_;
      beta_ = obj.beta_;
      h_go_ = obj.h_go_;
      h_stop_ = obj.h_stop_;
      headway_ = obj.headway_;
      v_max_ = obj.v_max_;

      vdot = alpha*(obj.range_policy - vel);
      vdot = vdot + beta * (vel_1 - vel);
    end

    function vel = range_policy(obj)
      % Calculates a heading-based (desired) velocity
      v_max = obj.v_max_;
      h_stop = obj.h_stop_;
      h_go = obj.h_go_;
      headway = obj.headway_;

      if 0 <= headway && headway <= h_stop
        vel = 0;
      elseif h_stop < headway && headway < h_go
        vel = v_max / 2 * (1 - cos(pi*(headway - h_stop)/(h_go - h_stop)));
      else
        vel = v_max;
      end
    end

    function set_headway(obj, headway)
      obj.headway_ = headway;
    end

    function headway = get_headway(obj)
      headway = obj.headway_;
    end
  end
end
