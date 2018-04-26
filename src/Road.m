% Defines a road to simulate trafic according to M. Treiber and A. Kesting. Much
% of this has been borrowed from/inspired by
% https://github.com/movsim/traffic-simulation-de

classdef Road
  properties (Access=public)
  end
  properties (Access=private)
  % @param road_length_:    link length [m]
  % @param traj_x_:         function arc length u -> phys x coordinate (East)
  % @param traj_y_:         function arc length u -> phys y coordinate (North)
  % @param speed_init_:     initial longitudinal speed [m/s]
  % @param vehicles_:       array of vehicles on road
    road_length_
    traj_x_
    traj_y_
    speed_init_
    vehicles_
  end
  methods (Access=public)
    function obj = Road(road_length, init_density, init_speed)
      obj.road_length_ = road_length;
      obj.vehicles_ = obj.createVehicleArray(init_density, init_speed);
    end
  end
  methods (Access=private)
    function [vehicles] = createVehicleArray(obj, init_density, init_speed)
      vel_length = 7; % [m]
      speed = 0.8 * init_speed;
      num_vehicles = floor(obj.road_length_ * init_density);
      obj.vehicles_(1, num_vehicles) = Vehicle(vel_length);

      for i = num_vehicles:-1:1 % counting backwards for Memory pre-allocation
        % find the position of the current vehicle
        u = (num_vehicles-i-1) * obj.road_length_/(num_vehicles);
        obj.vehicles_(i).setu(u);
        obj.vehicles_(i).setspeed(speed);
      end
    end
  end
end
