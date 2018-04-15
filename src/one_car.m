% Example usage
Plot the range policy

alpha = 1; beta = 1;
v_max = 15; h_stop = 5; h_go = 25;
hv = HumanDriver(v_max, h_stop, h_go, alpha, beta);
max_test_headway = 30;
step_size = 0.01;
headways = 0:step_size:max_test_headway;
num_its = max_test_headway / step_size + 1;
ranges = zeros(1, num_its);

for i = 1:num_its
  hv.headway_ = headways(i);
  ranges(i) = hv.range_policy();
end

plot(headways, ranges)
