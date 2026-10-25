%% run_compare.m - Simulasyonu kosturur, metrikleri hesaplar, grafik cizer
clear; clc; close all;
design_pid;                              % quad_params + Kp, Ki, Kd
mdl = 'quadrotor_1d_v2';
t_step = 1;                              % z_ref basamak zamani [s]
t_sine = 12;                             % sinus ruzgar baslangici [s]
tol    = 0.01;                           % yerlesme bandi [m]

out = sim(mdl);
t   = out.tout;
zr  = out.zref_log;
z   = out.z_log;
u   = out.ucmd_log;
d   = out.wind_log;

M = compute_metrics(t, z, zr, u, t_step, wind_t0, wind_dur, t_sine, tol);

%% Tablo
fprintf('\n--- Klasik PID ---\n');
fprintf('Asim (basamak)           : %.1f %%\n', M.overshoot);
fprintf('Ruzgar cukuru (5-8 s)    : %.3f m\n',  M.dip);
fprintf('Ruzgar sonrasi firlama   : %.3f m\n',  M.rebound);
fprintf('Ruzgar sonrasi yerlesme  : %.2f s\n',  M.t_settle);
fprintf('Sinus tepeden tepeye     : %.3f m\n',  M.p2p);
fprintf('Sinus RMS hata           : %.3f m\n',  M.rms);
fprintf('ITAE (toplam)            : %.4f\n',    M.itae);
fprintf('Max U_cmd                : %.2f N (limit %.2f N)\n', M.u_max, U_max);

%% Grafik
figure('Name', 'Klasik PID');
subplot(3,1,1);
plot(t, zr, '--', t, z, 'LineWidth', 1.2); grid on;
ylabel('z [m]'); legend('z_{ref}', 'z'); title('Klasik PID');
subplot(3,1,2);
plot(t, u, t([1 end]), [U_max U_max], 'r--'); grid on;
ylabel('U_{cmd} [N]');
subplot(3,1,3);
plot(t, d); grid on;
ylabel('d [N]'); xlabel('t [s]');

%% Metrik fonksiyonu (MLP icin de aynisi kullanilacak) - dosyanin en sonunda kalmali
function M = compute_metrics(t, z, zr, u, t_step, w_t0, w_dur, t_sine, tol)
    e     = zr - z;
    zf    = zr(end);
    t_end = w_t0 + w_dur;

    % Basamak
    m1 = t >= t_step & t < w_t0;
    M.overshoot = (max(z(m1)) - zf)/zf*100;

    % Ruzgar darbesi
    m2 = t >= w_t0 & t < t_end;
    M.dip = zf - min(z(m2));                 % ruzgar altindaki cukur
    m3 = t >= t_end & t < t_sine;
    M.rebound = max(z(m3)) - zf;             % ruzgar kesilince firlama
    ta = t(m3);
    ea = abs(e(m3));
    k  = find(ea > tol, 1, 'last');
    if isempty(k)
        M.t_settle = 0;
    elseif k == numel(ta)
        M.t_settle = NaN;                    % sinus baslayana kadar yerlesmedi
    else
        M.t_settle = ta(k+1) - t_end;
    end

    % Sinus ruzgari
    m4 = t >= t_sine;
    M.p2p = max(z(m4)) - min(z(m4));
    M.rms = sqrt(trapz(t(m4), e(m4).^2) / (t(end) - t(find(m4,1))));

    M.itae  = trapz(t, t.*abs(e));
    M.u_max = max(u);

    M_pid = M; save('baseline_pid.mat','M_pid');
end