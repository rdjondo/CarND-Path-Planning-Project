%clc;
clear;

sk = 1;
sk_dot = 0;
sk_double_dot = 0;

sT_dot_vec = -20:0.01:20;
sT_double_dot = 0.0;

for sT_dot=sT_dot_vec
    
    timer_slow=tic;
    [coeffs_s1, delta_t1, T1] = optim_jmt(sk,sk_dot,sk_double_dot,sT_dot, sT_double_dot, false);
    time1 = toc(timer_slow);
    
    timer_fast = tic;
    [coeffs_s2, delta_t2, T2] = optim_jmt(sk,sk_dot,sk_double_dot,sT_dot, sT_double_dot, true);
    time2 = toc(timer_fast);
    
    coef_error = sum((coeffs_s1 - [0 coeffs_s2]).^2);
%     if coef_error>1e-4
%         disp(['Large diff error ('...
%             num2str(coef_error) ') for sT_dot = ' num2str(sT_dot) ])
%     end
end

disp(['Speed improvement: ' num2str(time1/time2) ])
% Plot
figure(1)
plot_jmt(coeffs_s1, delta_t1, T1)

figure(2)
plot_jmt(coeffs_s2, delta_t2, T2)
