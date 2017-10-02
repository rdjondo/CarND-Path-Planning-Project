%clc;
clear;

sk = 1;
sk_dot = 0;
sk_double_dot = 0;

sT_dot = 20;
sT_double_dot = 0.0;

tic
[coeffs_s, delta_t, T] = optim_jmt(sk,sk_dot,sk_double_dot,sT_dot, sT_double_dot);
toc

% Plot
plot_jmt(coeffs_s, delta_t, T)