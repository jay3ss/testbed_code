% Defines a road to simulate trafic according to M. Treiber and A. Kesting. Much
% of this has been borrowed from/inspired by
% https://github.com/movsim/traffic-simulation-de

classdef Vehicle < handle
%   properties (Access=public)
%   end
  properties %(Access=private)
  % accel:   acceleration [m/s^2]
  % length:  length of the vehicle [m]
  % u:       longitudinal coordinate (arc-length) [m]
  % speed:   speed of vehicle [m/s]
  % id:      id of the vehicle
  % lead_id: id of the lead vehicle
  % driver:  model of the driver
    accel
    length
    u
    speed
    id
    lead_id
    driver
  end

  methods
    function obj = Vehicle(vel_length, u, speed, driver)
      if nargin == 4
        obj.length = vel_length;
        obj.u = u;
        obj.speed = speed;
        obj.driver = driver;
      end
      % obj.length = vel_length;
      % obj.u = u;
      % obj.speed = speed;
      % obj.id = floor(100000*rand()+200)
    end

    function set.accel(obj, a)
      obj.accel = a;
    end

    function a = get.accel(obj)
      a = obj.accel;
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

    function set.id(obj, n)
      obj.id = n;
    end

    function n = get.id(obj)
      n = obj.id;
    end

    function set.lead_id(obj, n)
      obj.lead_id = n;
    end

    function n = get.lead_id(obj)
      n = obj.lead_id;
    end

    function set.driver(obj, d)
      obj.driver = d;
    end

    function d = get.driver(obj)
      d = obj.driver;
    end
  end
  methods (Access=private)
  end
end
