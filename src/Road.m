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
    dt_
  end
  methods
    function obj = Road(road_length, init_density, vehicle, v_param, driver, d_param, dt)
      % Constructor
      % @param road_length = length of the road
      % @param init_density = intial density of the road [vehicle/m]
      % @param vehicle: vehicle object
      % @param v_param:   container.Map that contains init_speed and length keys
      % @param driver: driver object
      % @param d_param: container.Map that contains the parameters about the
      % driver
      obj.road_length = road_length;
      obj.num_vehicles = floor(obj.road_length * init_density);
      obj.vehicles = obj.createVehicleArray(vehicle, v_param, driver, d_param);
      obj.dt_ = dt;
    end

    function calcAccelerations(obj)
      for i = 1:obj.num_vehicles
        this_vehicle = obj.vehicles(i);
        lead_id = this_vehicle.lead_id;
        lead_vehicle = obj.vehicles(lead_id);
        s = lead_vehicle.u - lead_vehicle.length - this_vehicle.u;
        s = s  + obj.road_length;
        speed = this_vehicle.speed;
        lead_speed = lead_vehicle.speed;
        lead_accel = lead_vehicle.accel;

        accel = this_vehicle.driver.calcAccel(s, speed, lead_speed, lead_accel);
        this_vehicle.accel = accel;
        obj.vehicles(i) = this_vehicle;
        % obj.vehicles(i).driver.calcAccel(s, speed, lead_speed, lead_accel);
      end
    end

    function updateSpeedPositions(obj)
      dt = obj.dt_;
      road_len = obj.road_length;

      for i = 1:obj.num_vehicles
        % vehicle = obj.vehicles(i);
        speed = obj.vehicles(i).speed;
        accel = obj.vehicles(i).accel;
        % disp('id'); disp(obj.vehicles(i).id);
        % disp('speed'); disp(obj.vehicles(i).speed);
        % disp('accel'); disp(obj.vehicles(i).accel);
        % Positional update with old speed
        obj.vehicles(i).u = obj.vehicles(i).u + max(0, speed*dt + 0.5*accel*dt^2) - road_len;
        % disp('u'); disp(obj.vehicles(i).u)

        % Update the speed
        obj.vehicles(i).speed = max(speed + accel*dt, 0);

        % obj.vehicles(i) = vehicle;
      end
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
    function [vehicles] = createVehicleArray(obj, vehicle, v_param, driver, d_param)
      % Creates an array of homogeneous vehicles
      % @param vehicle: vehicle object
      % @param v_param:   container.Map that contains init_speed and length keys
      % @param driver: driver object
      % @param d_param: container.Map that contains the parameters about the
      % driver
      % @returns vehicles: array of homogeneous vehicles
      speed = 0.8 * v_param('init_speed');
      vel_len = v_param('length');
      n_vehicles = obj.num_vehicles;
      vehicles(1, n_vehicles) = vehicle;
      driver.a = d_param('a');
      driver.b = d_param('b');
      driver.s0 = d_param('s0');
      driver.T = d_param('T');
      driver.v0 = d_param('v0');
      % obj.vehicles(1, num_vehicles) = Vehicle(vel_length);

      for i = 1:n_vehicles % counting backwards for Memory pre-allocation
        % find the position of the current vehicle
        u = (n_vehicles-i-1) * obj.road_length/(n_vehicles);
        vehicles(i).u = u;
        vehicles(i).length = vel_len;
        vehicles(i).speed = speed;
        vehicles(i).driver = driver;
        if i == 1
          vehicles(i).lead_id =  n_vehicles;
        elseif i == n_vehicles
          vehicles(i).lead_id = 1;
        else
          vehicles(i).lead_id = i + 1;
        end
      end
    end
  end
end
