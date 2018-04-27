close all;
road_length = 1000; % m
init_density = 0.05; % vehicle/m
init_speed = 25; % m/s
vel_length = 7; % m

keys = {'init_speed', 'length'};
vals = {init_speed, vel_length};

v_def = containers.Map(keys, vals);

vehicle = Vehicle;

road = Road(road_length, init_density, vehicle, v_def);
road_radius = road_length / (2*pi);

% step_size = 1/road.num_vehicles;
% phi = 1:step_size:2*pi;

xs = zeros(1, road.num_vehicles);
ys = zeros(1, road.num_vehicles);

for i = 1:road.num_vehicles
  phi = 2 * pi * road.vehicles(i).u / road_length;
  xs(i) = road_radius * cos(phi);
  ys(i) = road_radius * sin(phi);
end

figure;
plot(xs, ys, 'bx')
% labels = num2str((1:road.num_vehicles)');
% text(xs(:,1), ys(:,2), labels, 'horizontal','left', 'vertical','bottom')
grid on
pbaspect([1 1 1])
title('Vehicle positions (m)')
