% Defines a road to simulate trafic according to M. Treiber and A. Kesting. Much
% of this has been borrowed from/inspired by
% https://github.com/movsim/traffic-simulation-de

classdef Road < handle
  properties (Access=public)
    road_length
    num_vehicles
    vehicles
  end
  properties (Access=private)
  % @param road_length:    link length [m]
  % @param traj_x_:         function arc length u -> phys x coordinate (East)
  % @param traj_y_:         function arc length u -> phys y coordinate (North)
  % @param speed_init_:     initial longitudinal speed [m/s]
  % @param vehicles:       array of vehicles on road
    
    traj_x_
    traj_y_
    speed_init_
  end
  methods
    function obj = Road(road_length, init_density, vehicle, v_def)
      % Constructor
      % @param road_length = length of the road
      % @param init_density = intial density of the road [vehicle/m]
      % @param vehicle: vehicle object
      % @param v_def:   container.Map that contains init_speed and length
      % keys
      obj.road_length = road_length;
      obj.num_vehicles = floor(obj.road_length * init_density);
      obj.vehicles = obj.createVehicleArray(vehicle, v_def);
    end
    
    function num_vehicles = get.num_vehicles(obj)
      num_vehicles = obj.num_vehicles;
    end
    
    function road_length = get.road_length(obj)
      road_length = obj.road_length;
    end
    
    function vels = get.vehicles(obj)
      vels = obj.vehicles;
    end
  end
  methods (Access=private)
    function [vehicles] = createVehicleArray(obj, vehicle, v_def)
      % Creates an array of homogeneous vehicles
      % @param vehicle: vehicle object
      % @param v_def:   container.Map that contains init_speed and length
      % keys
      % @returns vehicles: array of homogeneous vehicles
      speed = 0.8 * v_def('init_speed');
      vel_len = v_def('length');
      n_vehicles = obj.num_vehicles;
      vehicles(1, n_vehicles) = vehicle;
      % obj.vehicles(1, num_vehicles) = Vehicle(vel_length);

      for i = n_vehicles:-1:1 % counting backwards for Memory pre-allocation
        % find the position of the current vehicle
        u = (n_vehicles-i-1) * obj.road_length/(n_vehicles);
        vehicles(i).u = u;
        vehicles(i).length = vel_len;
        vehicles(i).speed = speed;
      end
    end
  end
end