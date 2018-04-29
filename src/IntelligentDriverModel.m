% Defines an intelligent driver model (IDM) according to M. Treiber and A.
% Kesting. Much of this has been borrowed from/inspired by
% https://github.com/movsim/traffic-simulation-de

classdef IntelligentDriverModel < handle
% basis model for the longitudinal dynamics, i.e., accelerations and braking
% decelerations of the drivers
  properties (Access=public)
  % v0: desired speed [m/s]
  % T: desired time gap [s]
  % s0: minimum gap [m]
  % a: maximum acceleration [m/s^2]
  % b: maximum deceleration [m/s^2]
    v0
    T
    s0
    a
    b
  end

  properties (Access=private)
  % speedlimit_
  % speedmax_
  % bmax_
  % alpha_v0: temporary multiplier
    speedlimit_
    speedmax_
    bmax_
    alpha_v0
  end

  methods
    function obj = IntelligentDriverModel(v0, T, s0, a, b)
      % Constructor
      % Set some "sensible" defaults (taken from traffic-simulation-de) for
      % speedlimit_, speedmax_, bmax_, & alphav0
      if nargin == 5
        obj.v0 = v0;
        obj.T = T;
        obj.s0 = s0;
        obj.a = a;
        obj.b = b;
      end
      obj.speedlimit_ = 1000;
      obj.speedmax_ = 1000;
      obj.bmax_ = 16;
      obj.alpha_v0 = 1;
    end

    function accel = calcAccel(obj, s, v, vl, al)
      % Calculates the acceleration for the IDM
      % dv/dt = a(1 - (v/v0)^4 - (s_star/s)^)
      % s_star = s0 + v*T + v*del_v/(2*sqrt(a*b))
      % @param s:     actual gap [m]
      % @param v:     actual speed [m/s]
      % @param vl:    leading speed [m/s]
      % @param al:    leading accel [m/s^2] (only for common interface; ignored)
      %
      % @return:  accel [m/s^2]

      % v0eff = obj.determineValidLocalV0();
      % acc_free = obj.calcAccFree(v, v0eff);
      % acc_int = obj.calcAccInt(v, vl);
      % accel = 0.0;
      %
      % if (v0eff < 0.00001)
      %   accel = 0;
      % else
      %   accel = max(-obj.bmax_, acc_free + acc_int);
      % end

      noise_acc = 0.3;
      acc_rnd = noise_acc * (rand() - 0.5);

      v0eff = min(min(obj.v0, obj.speedlimit_), obj.speedmax_);
      v0eff = v0eff * obj.alpha_v0;

      % actual acceleration model
      acc_free = 0.0;
      if (v < v0eff)
        acc_free = obj.a*(1-(v/v0eff)^4);
      else
        acc_free = obj.a*(1-v/v0eff);
      end

      s_star = obj.s0 + max(0., v * obj.T + 0.5 * v * (v-vl) / sqrt(obj.a * obj.b));
      acc_int = -obj.a * (s_star / max(s, obj.s0))^2;

      accel = 0;
      if (v0eff < 0.00001)
        accel = 0;
      else
        accel = max(-obj.bmax_, acc_free + acc_int + acc_rnd);
      end
    end % calcAccel

    function set.v0(obj, v)
      obj.v0 = v;
    end

    function v = get.v0(obj)
      v = obj.v0;
    end

    function set.T(obj, t)
      obj.T = t;
    end

    function t = get.T(obj)
      t = obj.T;
    end

    function set.s0(obj, s)
      obj.s0 = s;
    end

    function s = get.s0(obj)
      s = obj.s0;
    end

    function set.a(obj, aa)
      obj.a = aa;
    end

    function aa = get.a(obj)
      aa = obj.a;
    end

    function set.b(obj, bb)
      obj.b = bb;
    end

    function bb = get.b(obj)
      bb = obj.b;
    end
  end % public methods

  methods (Access=private)
    function valid_v0 = determineValidLocalV0(obj)
      valid_v0 = min(min(obj.v0, obj.speedlimit_), obj.speedmax_);
      valid_v0 = valid_v0 * obj.alpha_v0;
    end % determineValidLocalV0

    function acc_free = calcAccFree(obj, v, v0eff)
      if (v < v0eff)
        acc_free = obj.a*(1-(v/v0eff)^4);
      else
        acc_free = obj.a*(1-v/v0eff);
      end
    end % calcAccFree

    function acc_int = calcAccInt(obj, v, vl, s)
      s_star = obj.calcSStar(v, vl);
      acc_int = -obj.a*(s_star/max(s,obj.s0))^2;
    end % calcAccInt

    function s_star = calcSStar(obj, v, vl)
      s0 = obj.s0;
      s_tmp1 = 0.5 * v * (v - vl) / sqrt(obj.a * obj.b);
      s_tmp2 = max(0, v * obj.T + s_tmp1);
      s_star = obj.s0 + s_tmp2;
    end % calcSStar
  end % private methods
end % IntelligentDriverModel
