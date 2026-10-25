%% design_pid.m - Kutup atama ile PID kazanclari
quad_params;                                  % m, zeta, wn, alpha yuklenir

Kd = m*(2*zeta*wn + alpha);
Kp = m*(wn^2 + 2*zeta*wn*alpha);
Ki = m*(alpha*wn^2);

% Kapali cevrim karakteristik polinomu: m s^3 + Kd s^2 + Kp s + Ki
p = [m Kd Kp Ki];
kutuplar = roots(p);

fprintf('Kp = %.3f\nKi = %.3f\nKd = %.3f\n', Kp, Ki, Kd);
disp('Kapali cevrim kutuplari:');
disp(kutuplar);