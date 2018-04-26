% Defines an intelligent driver model (IDM) according to M. Treiber and A.
% Kesting. Much of this has been borrowed from /inspired by
% https://github.com/movsim/traffic-simulation-de

classdef IntelligentDriverModel < handle
% basis model for the longitudinal dynamics, i.e., accelerations and braking
% decelerations of the drivers
  properties (Access=public)

  end

  properties (Access=private)
  % v0: desired speed [m/s]
  % T_: desired time gap [s]
  % s0_: minimum gap [m]
  % a_: maximum acceleration [m/s^2]
  % b_: maximum deceleration [m/s^2]
  % speedlimit_
  % speedmax_
  % bmax_
  % alpha_v0_: temporary multiplier

    v0_
    T_
    s0_
    a_
    b_
    speedlimit_
    speedmax_
    bmax_
    alpha_v0_
  end

  methods (Access=public)
    function obj = IntelligentDriverModel(v0, T, s0, a, b)
      % Constructor
      % Set some "sensible" defaults (taken from traffic-simulation-de) for
      % speedlimit_, speedmax_, bmax_, & alpha_v0_
      obj.v0_ = v0;
      obj.T_ = T;
      obj.s0_ = s0;
      obj.a_ = a;
      obj.b_ = b;
      obj.speedlimit_ = 1000;
      obj.speedmax_ = 1000;
      obj.bmax_ = 16;
      obj.alpha_v0_ = 1;
    end

    function accel = calcAccel(obj, s, v, vl, al)
      % Calculates the acceleration for the IDM
      % @param s:     actual gap [m]
      % @param v:     actual speed [m/s]
      % @param vl:    leading speed [m/s]
      % @param al:    leading accel [m/s^2] (only for common interface; ignored)
      %
      % @return:  accel [m/s^2]

      v0_eff = obj.determineValidLocalV0();
      acc_free = obj.calcAccFree(v, v0_eff);
      acc_int = obj.calcAccInt(v, vl);
      accel = 0.0;

      if (v0_eff < 0.00001)
        accel = 0;
      else
        accel = max(-obj.bmax_, acc_free + acc_int);
      end
    end % calcAccel
  end % public methods

  methods (Access=private)
    function valid_v0 = determineValidLocalV0(obj)
      valid_v0 = min(obj.v0_, obj.speedlimit_, obj.speedmax_);
      valid_v0 *= obj.alpha_v0;
    end % determineValidLocalV0

    function acc_free = calcAccFree(obj, v, v0_eff)
      if (obj.v_ < v0_eff)
        acc_free = obj.a_*(1-(v/v0eff)^4));
      else
        acc_free = obj.a_*(1-v/v0eff)
      end
    end % calcAccFree

    function acc_int = calcAccInt(obj, v, vl, s)
      s_star = obj.calcSStar(v, vl);
      acc_int = -obj.a_*(s_star/max(s,obj.s0_))^2;
    end % calcAccInt

    function s_star = calcSStar(obj, v, vl)
      s_star = obj.s0_ + max(0.,v*obj.T_ + 0.5*v*(v - vl)/sqrt(obj.a_*obj.b_));
    end % calcSStar
  end % private methods
end % IntelligentDriverModel
