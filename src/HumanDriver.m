% Human driver model, taken from
% Optimal control of connected vehicle systems
% Jin I. Ge, Gabor Orosz
%
% % Example usage
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

% Start with two human drivers
%
% alpha = 1; beta = 1;
% v_max = 15; h_stop = 5; h_go = 25;
%
% hv1 = HumanDriver(v_max, h_stop, h_go, alpha, beta);
% hv2 = HumanDriver(v_max, h_stop, h_go, alpha, beta);
%
% run_time = 30 % seconds
% delta_t = 0.001 % 1 ms
% num_its =  run_time/delta_t + 1;
% X0 = [30; 0; 0];
% X = zeros(3, num_its);
% X_dot = X;
% X(:, 1) = X0;
%
% vel1 = 0; % velocity of hv1
% vel2 = 0; % velocity of hv2
%
% for i = 1:delta_t:run_time+1
%   vel1 = vel1 + X(2, i) * delta_t;
%   vel2 = vel2 + X(3, i) * delta_t;
%   h_dot = X(3, i) - X(2, i);
%   X_dot(1, i) = X_dot(1, i) + h_dot * delta_t;
%   X_dot(2, i) = X_dot(2, i) + hv1.v_dot(vel2) * delta_t;
%   X_dot(3, i) = X_dot(2, i) + hv2.v_dot(vel1) * delta_t;
% end


classdef HumanDriver
  properties (Access = private)
  % Underscore denotes that the property is private
    alpha_
    beta_
    h_go_
    h_stop_
    headway_
    v_max_
    velocity_
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

    function hdot = h_dot(obj, vel_1)
      % vel_1 (float): velocity of vehicle in front of this vehicle
      % vel (float): current velocity of this vehicle (should probably calculate
      % this within the class)
      hdot = vel_1 - obj.velocity_;
    end

    function vdot = v_dot(obj, vel_1)
      % vel_1 (float): velocity of vehicle in front of this vehicle
      % vel (float): current velocity of this vehicle (should probably calculate
      % this within the class)
      alpha = obj.alpha_;
      beta = obj.beta_;
      h_go = obj.h_go_;
      h_stop = obj.h_stop_;
      headway = obj.headway_;
      v_max = obj.v_max_;
      velocity = obj.velocity_;

      vdot = alpha * (obj.range_policy - velocity);
      vdot = vdot + beta * (vel_1 - velocity);
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

    function set_velocity(obj, velocity)
      obj.velocity_ = velocity;
    end

    function velocity = get_velocity(obj)
      velocity = obj.velocity_;
    end
  end
end
