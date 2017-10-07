% Optimize sk and T 
function [coeffs_s, delta_t, T] = optim_jmt(sk,sk_dot,sk_double_dot,sT_dot, sT_double_dot)

% Make static 
sT = sk + 128;
% Increase sT when on jerk(t), a5 is negative (and -a4/(2*a5) is between 0
% and T)
% Reduce sT when jerk on jerk(t), a5 is positive and (-a4/(2*a5) is between
% 0 and T)
% Stop when a5 is very small


virtual_acceleration = 3;
if abs(sT_dot-sk_dot)<0.1
warning('speed difference is small')
end

T = max(0.5,abs(sT_dot-sk_dot)/virtual_acceleration);

delta_t = 0.02;
N_samples = T/delta_t;


num_iter = 1;

coeffs_s = jmt(sk,sk_dot,sk_double_dot,sT,sT_dot, sT_double_dot,T);
%coeffs_jerk = polyder(polyder(polyder(coeffs_s)));

sT_inc = (sT-sk)/2; % search increment
direction = 1;
direction_change = 1;


while abs(coeffs_s(1))>10e-6
  % Stop when a5 is very small
  
  num_iter = num_iter + 1;
  
  % jerk(t) is a quadratic function
  if coeffs_s(1)<0
    % Increase sT when on jerk(t), a5 is negative (and -a4/(2*a5) is between 0
    % and T)  
    direction_change = direction_change * direction;
    direction = 1;
  else
    % Reduce sT when jerk on jerk(t), a5 is positive and (-a4/(2*a5) is between
    % 0 and T)
    direction_change = direction_change * direction;
    direction = -1;
  end
  
  
  if direction_change<0
    sT_inc = sT_inc/2;
  end
  
  % Change terminal position
  sT = sT + sT_inc*direction;
  
  coeffs_s = jmt(sk,sk_dot,sk_double_dot,sT,sT_dot, sT_double_dot,T);
  %coeffs_jerk = polyder(polyder(polyder(coeffs_s)));
  % Plot
  % plot_jmt(coeffs_s, delta_t, T)
  % pause(0.1)
  
  if num_iter>=50
    break
  end
end

num_iter
sT_inc
