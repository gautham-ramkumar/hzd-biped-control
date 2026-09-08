set_path;
f = [-0.4907, -2.3947, 0.3053, 0.6256, 0.8814, 2.6360, 2.7919, 2.9537];
[t_sol, z_sol] = sim_zero_dynamics(f);
q1_min = -f(1); q1_max = f(1);
s_params = [q1_min, q1_max];
alpha2 = [-f(5), -f(4), f(3:5)];
alpha3 = [f(8)-f(5), -f(7)+2*f(6), f(6:8)];
a = [alpha2, alpha3];
J_integral = 0;
for i = 1:length(t_sol)-1
    u = func_compute_control_action(z_sol(i,:),a,s_params);
    dt = t_sol(i+1) - t_sol(i);
    J_integral = J_integral + norm(u)^2 * dt;
end
J_mean = 0;
for i = 1:length(t_sol)
    u = func_compute_control_action(z_sol(i,:),a,s_params);
    J_mean = J_mean + norm(u)^2;
end
J_mean = J_mean / length(t_sol);

disp(['J_integral per step: ', num2str(J_integral)]);
disp(['J_mean per step: ', num2str(J_mean)]);

% Using sample report formula
% speed = 2.92
v_walk = 2.917;
d = 1.4515;
Mtot = 35; g = 9.81;

MCOT_integral_v = J_integral / (Mtot * g * v_walk);
MCOT_mean_v = J_mean / (Mtot * g * v_walk);
MCOT_mean_d = J_mean / (Mtot * g * d);
MCOT_integral_d = J_integral / (Mtot * g * d);

disp(['Sample MCOT (J_mean / M g v): ', num2str(MCOT_mean_v)]);
disp(['Sample MCOT (J_integral / M g v): ', num2str(MCOT_integral_v)]);
disp(['Sample MCOT (J_mean / M g d): ', num2str(MCOT_mean_d)]);
disp(['Sample MCOT (J_integral / M g d): ', num2str(MCOT_integral_d)]);
