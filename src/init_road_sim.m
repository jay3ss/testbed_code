close all;
road_length = 1000; % m
init_density = 0.02; % vehicle/m
init_speed = 25; % m/s
vel_length = 7; % m

keys = {'init_speed', 'length'};
vals = {init_speed, vel_length};

v_def = containers.Map(keys, vals);

vehicle = Vehicle;

road = Road(road_length, init_density, vehicle, v_def);

step_size = 1/road.num_vehicles;
phi = 1:step_size:2*pi;

xs = zeros(1, road.num_vehicles);
ys = zeros(1, road.num_vehicles);

for i = 1:road.num_vehicles
  xs(i) = road.road_length/(2*pi) * road.vehicles(i).u * cos(phi(i));
  ys(i) = road.road_length/(2*pi) * road.vehicles(i).u * sin(phi(i));
end

scatter(xs, ys)
title('Vehicle positions')