
function plot_jmt(coeffs, delta_t, T)

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
