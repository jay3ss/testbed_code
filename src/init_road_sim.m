clear;
close all;
% Vehicle parameters
road_length = 100; % m
init_density = 0.3; % vehicle/m
init_speed = 25; % m/s
vel_length = 7; % m

% Driver parameters
a = 0.3;
b = 3.0;
s0 = 2.0;
T = 1.5;
v0 = 120.0;

dt = 0.05;

keys = {'init_speed', 'length'};
vals = {init_speed, vel_length};

v_params = containers.Map(keys, vals);

vehicle = Vehicle;

driver = IntelligentDriverModel;

keys = {'a', 'b', 's0', 'T', 'v0'};
vals = {a, b, s0, T, v0};
d_params = containers.Map(keys, vals);

road = Road(road_length, init_density, vehicle, v_params, driver,  d_params, dt);
road_radius = road_length / (2*pi);

% step_size = 1/road.num_vehicles;
% phi = 1:step_size:2*pi;

xs = zeros(1, road.num_vehicles);
ys = zeros(1, road.num_vehicles);

end_time = 800; % seconds
time = 0:dt:end_time;
[z, sim_length] = size(time);

Xs = zeros(road.num_vehicles, sim_length);
Ys = zeros(road.num_vehicles, sim_length);

axis_sz = 1.2*road_radius;
% axis_sz = 1.2*road.road_length;

outfile = 'idm.gif';
line = linspace(0, road.road_length, road.num_vehicles);

avg_speed = 0;
% labels = cellstr( num2str((1:10)') );  %' # labels correspond to their order

col = 1;
data_color ='bo';
txt_color = 'blue';
labels = [];

for l = 1:road.num_vehicles
  labels = [labels, ' ' , num2str(l)];
end

labels = string(split(labels));

for t = time
  for i = 1:road.num_vehicles
    phi = 2 * pi * road.vehicles(i).u / road_length;
    xs(i) = (road_radius * cos(phi));
    ys(i) = (road_radius * sin(phi));
%     xs(i) = road.vehicles(i).u;    
    avg_speed  = avg_speed + road.vehicles(i).speed;
    if mod(i, 2) == 0
      data_color = 'bo';
      txt_color = 'blue';
    else
      data_color = 'ro';
      txt_color = 'red';
    end
    plot(xs(i), ys(i), data_color);
    hold on;
    text(xs(i), ys(i), num2str(i), ...
        'Color', txt_color, ...
        'VerticalAlignment','bottom', ...
        'HorizontalAlignment','right')
  end
  Xs(:,col) = xs';
  Ys(:,col) = ys';
  col = col + 1;
  avg_speed = avg_speed / road.num_vehicles;
%   plot(xs, ys, )
%   text(xs, ys, labels, 'VerticalAlignment','bottom', ...
%                        'HorizontalAlignment','right')
%     xs = zeros(1, road.num_vehicles);
%     for i = 1:road.num_vehicles
%         xs(i) = road.vehicles(i).u;
%     end
%   plot(1:road.road_length, road.vehicles(i).u, 'bx');
%     scatter(line, xs, 'bx');
axis([-axis_sz axis_sz -axis_sz axis_sz]);
%   axis([-1.2*road.road_length 1.2*road.road_length -0.2*road.road_length 0.2*road.road_length]);
%   axis([-5 road.road_length -5 5]);
%   labels = num2str((1:road.num_vehicles));
%   text(xs(:,1), ys(:,2), labels, 'horizontal','left', 'vertical','bottom')
  grid on;
  pbaspect([1 1 1])

  % gif utilities
  txt = ['Vehicle positions (m),  Sim Time: ', num2str(t), ' s,  Avg. speed ', num2str(avg_speed), ' m/s'];
  title(txt)
  set(gcf, 'color', 'w');
  drawnow;
  frame = getframe(1);
  im = frame2im(frame);
  [imind, cm] = rgb2ind(im, 256);

  if t == 0
    imwrite(imind, cm, outfile, 'gif', 'DelayTime', 0, 'loopcount', inf);
  else
     imwrite(imind, cm, outfile, 'gif', 'DelayTime', 0, 'writemode', 'append');
  end
  clf;
  road.calcAccelerations();
  road.updateSpeedPositions();
end
