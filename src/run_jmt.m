clear

sk = 0;
sk_dot =0;
sk_double_dot = 0.0;
sk_triple_dot = 0.0;

sT = sk + 60;
% Increase sT when acceleration decreases first, then increases (around T/2) then decreases
% Reduce sT when increases first then decreases (around T/2) and then increases (around T)

sT_dot = 25;
sT_double_dot = 0.0;
sT_triple_dot = 0.0;
N_samples = 250;
delta_t = 0.02;

T = N_samples*delta_t

coeffs = jmt(sk,sk_dot,sk_double_dot,sT,sT_dot, sT_double_dot,T)

figure(1)
subplot(221)
t = 0:delta_t:T;
X= polyval(coeffs, t);
plot(t, X, '.')
ylabel('position')
grid on

subplot(222)
V = diff( X )/delta_t;
t = t(1:end-1);
plot(t, V, '.')
ylabel('speed')
grid on

subplot(223)
A = diff( V )/delta_t;
t = t(1:end-1);
plot(t, A, '.')
ylabel('acceleration')
grid on

subplot(224)
J = diff( A )/delta_t;
t = t(1:end-1);
plot(t, J, '.')
ylabel('jerk')
grid on
