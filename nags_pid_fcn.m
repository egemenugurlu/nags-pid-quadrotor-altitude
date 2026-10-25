function [Kp, Ki, Kd] = nags_pid_fcn(e, v, Ie, W1, b1, W2, b2, W3, b3, Kp0, Ki0, Kd0, gamma)
%#codegen
% NAGS-PID Yapay Sinir Ağı İleri Besleme Bloğu

% 1. Girdi Normalizasyonu
e_norm  = e / 0.5;
v_norm  = v / 2.0;
Ie_norm = Ie / 2.0;

x = [e_norm; v_norm; Ie_norm]; % (3x1) Sütun vektörü

% 2. Gizli Katman 1 (ReLU)
h1 = max(0, W1 * x + b1);       % (16x1)

% 3. Gizli Katman 2 (ReLU)
h2 = max(0, W2 * h1 + b2);      % (16x1)

% 4. Çıkış Katmanı (Tanh)
out = tanh(W3 * h2 + b3);       % (3x1)

% 5. Kazanç Ölçekleme (%40 Güvenlik Sınırı)
dKp = out(1) * gamma;
dKi = out(2) * gamma;
dKd = out(3) * gamma;

% 6. Aktif PID Kazançları
Kp = Kp0 * (1.0 + dKp);
Ki = Ki0 * (1.0 + dKi);
Kd = Kd0 * (1.0 + dKd);
end