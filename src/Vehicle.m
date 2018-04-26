% Defines a road to simulate trafic according to M. Treiber and A. Kesting. Much
% of this has been borrowed from/inspired by
% https://github.com/movsim/traffic-simulation-de

classdef Vehicle < handle
  properties (Access=public)
  end
  properties (Access=private)
  % length_: length of the vehicle [m]
  % u:       longitudinal coordinate (arc-length) [m]
  % speed:   speed of vehicle [m/s]
    length_
    u
    speed
  end

  methods (Access=public)
    function obj = Vehicle(vel_length)
      % if nargin == 3
      %   obj.length_ = vel_length;
      %   obj.u = u;
      %   obj.speed_ = speed;
      % end
      obj.length_ = vel_length;
      obj.u = u;
      obj.speed = speed;
    end

    function set.u(obj, u)
    end

    function u = get.u(obj)
    end

    function set.speed(obj, u)
    end

    function speed = get.speed(obj)
    end
  end
  methods (Access=private)
  end
end
