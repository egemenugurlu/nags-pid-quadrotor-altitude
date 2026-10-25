%% run_compare_v2.m - Simülasyon Koşturucu ve Metrik Hesaplayıcı
clearvars -except ctrl_mode; clc; close all;

if ~exist('ctrl_mode', 'var')
    ctrl_mode = 0; % Varsayılan: 0 (Nominal PID)
end

design_pid_v2;                          % design_pid_v2'yi çağırıyor
mdl    = 'quadrotor_1d_v3';             % Simulink model adın
t_step = 1;                              % Referans basamak zamanı [s]
t_sine = 12;                             % Sinüzoidal rüzgar başlangıcı [s]
tol    = 0.01;                           % Yerleşme bandı [m]

% Simulink Modeli Çalıştır
out = sim(mdl);
t   = out.tout;
zr  = out.zref_log;
z   = out.z_log;
u   = out.ucmd_log;
d   = out.wind_log;

% Metrikleri Hesapla
M = compute_metrics(t, z, zr, u, t_step, wind_t0, wind_dur, t_sine, tol);

% Sonuçları Ekrana Yazdır
mode_names = {'Nominal PID (Mod 0)', 'Agresif PID (Mod 1)', 'NAGS-PID / MLP (Mod 2)'};
fprintf('\n===================================================\n');
fprintf('   KONTROL PERFORMANS SONUÇLARI [%s]\n', mode_names{ctrl_mode + 1});
fprintf('===================================================\n');
fprintf('Aşım (basamak)           : %.1f %%\n', M.overshoot);
fprintf('Rüzgar çukuru (5-8 s)    : %.3f m\n',  M.dip);
fprintf('Rüzgar sonrası fırlama   : %.3f m\n',  M.rebound);
fprintf('Rüzgar sonrası yerleşme  : %.2f s\n',  M.t_settle);
fprintf('Sinüs tepeden tepeye     : %.3f m\n',  M.p2p);
fprintf('Sinüs RMS hata           : %.3f m\n',  M.rms);
fprintf('ITAE (toplam)            : %.4f\n',    M.itae);
fprintf('Max U_cmd                : %.2f N (limit %.2f N)\n', M.u_max, U_max);
fprintf('===================================================\n');

% Grafik Çizimi
figure('Name', sprintf('Kontrol Tepkisi - %s', mode_names{ctrl_mode + 1}));

subplot(3,1,1);
plot(t, zr, 'r--', t, z, 'b-', 'LineWidth', 1.2); grid on;
ylabel('z [m]'); legend('z_{ref}', 'z'); 
title(sprintf('Sistem Yanıtı (%s)', mode_names{ctrl_mode + 1}));

subplot(3,1,2);
plot(t, u, 'g-', t([1 end]), [U_max U_max], 'r--', 'LineWidth', 1.2); grid on;
ylabel('U_{cmd} [N]'); legend('U_{cmd}', 'U_{max}');

subplot(3,1,3);
plot(t, d, 'k-', 'LineWidth', 1.2); grid on;
ylabel('d [N]'); xlabel('t [s]');

%% Metrik Fonksiyonu
function M = compute_metrics(t, z, zr, u, t_step, w_t0, w_dur, t_sine, tol)
    e     = zr - z;
    zf    = zr(end);
    t_end = w_t0 + w_dur;

    m1 = t >= t_step & t < w_t0;
    M.overshoot = (max(z(m1)) - zf) / zf * 100;

    m2 = t >= w_t0 & t < t_end;
    M.dip = zf - min(z(m2));
    
    m3 = t >= t_end & t < t_sine;
    M.rebound = max(z(m3)) - zf;
    
    ta = t(m3);
    ea = abs(e(m3));
    k  = find(ea > tol, 1, 'last');
    if isempty(k)
        M.t_settle = 0;
    elseif k == numel(ta)
        M.t_settle = NaN;
    else
        M.t_settle = ta(k+1) - t_end;
    end

    m4 = t >= t_sine;
    M.p2p = max(z(m4)) - min(z(m4));
    M.rms = sqrt(trapz(t(m4), e(m4).^2) / (t(end) - t(find(m4,1))));

    M.itae  = trapz(t, t .* abs(e));
    M.u_max = max(u);

    M_pid = M; 
    save('baseline_pid.mat', 'M_pid');
end