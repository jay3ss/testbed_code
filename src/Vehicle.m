% Defines a road to simulate trafic according to M. Treiber and A. Kesting. Much
% of this has been borrowed from/inspired by
% https://github.com/movsim/traffic-simulation-de

classdef Vehicle < handle
%   properties (Access=public)
%   end
  properties %(Access=private)
  % length: length of the vehicle [m]
  % u:       longitudinal coordinate (arc-length) [m]
  % speed:   speed of vehicle [m/s]
    length
    u
    speed
  end

  methods
    function obj = Vehicle(vel_length, u)
      if nargin == 3
        obj.length = vel_length;
        obj.u = u;
        obj.speed = speed;
      end
      % obj.length = vel_length;
      % obj.u = u;
      % obj.speed = speed;
    end

    function set.u(obj, u)
      obj.u = u;
    end

    function u = get.u(obj)
        u = obj.u;
    end

    function set.speed(obj, s)
      obj.speed = s;
    end

    function speed = get.speed(obj)
        speed = obj.speed;
    end
    
    function set.length(obj, l)
      obj.length = l;
    end

    function l = get.length(obj)
      l = obj.length;
    end
  end
  methods (Access=private)
  end
end
