%% design_pid_v2.m - Kutup Atama İle Nominal PID Tasarımı
quad_params_v2; % quad_params_v2'yi çağırıyor

% Pole Placement Hesabı
Kd = m * (2 * zeta * wn + alpha);
Kp = m * (wn^2 + 2 * zeta * wn * alpha);
Ki = m * (alpha * wn^2);

% Kapalı Çevrim Karakteristik Polinomu
poly_coeffs = [m, Kd, Kp, Ki];
kutuplar    = roots(poly_coeffs);

% Kazançları Workspace'e Kaydet
Kp0 = Kp;
Ki0 = Ki;
Kd0 = Kd;

fprintf('\n--- Analitik Hesaplanan Nominal PID Kazançları ---\n');
fprintf('Kp = %.3f | Ki = %.3f | Kd = %.3f\n', Kp, Ki, Kd);
disp('Kapalı Çevrim Kutupları:');
disp(kutuplar);