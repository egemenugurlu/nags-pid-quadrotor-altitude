%% quad_params_v2.m - 1D Quadrotor & NAGS-PID Sistem Parametreleri

% 1) Fiziksel Parametreler
m       = 1.0;                  % Kütle [kg]
g       = 9.81;                 % Yerçekimi ivmesi [m/s^2]
c_d     = 0.1;                  % Aerodinamik sürükleme katsayısı [kg/m]
w_hover = 700;                  % Hover motor hızı [rad/s]
k_f     = m*g/(4*w_hover^2);    % İtki katsayısı
tau_m   = 0.05;                 % Motor zaman sabiti [s]
U_max   = 2*m*g;                % Toplam maks itki [N] (19.62 N)
w_max   = sqrt(U_max/(4*k_f));  % Motor maks hızı [rad/s]
z0      = 0;                    % Başlangıç irtifası [m]

% 2) PID Tasarım Parametreleri (Kutup Atama)
zeta  = 0.707;                  % Sönüm oranı
wn    = 4.0;                    % Doğal frekans [rad/s]
alpha = 2.0;                    % Entegratör kutbu

% 3) Rüzgar Bozucu Senaryosu
wind_amp = -6.0;                % Genlik [N]
wind_t0  = 5.0;                 % Başlangıç zamanı [s]
wind_dur = 3.0;                 % Süre [s]

% 4) Kontrol Modu Seçimi (0 = Nominal, 1 = Agresif, 2 = MLP/NAGS-PID)
if ~exist('ctrl_mode', 'var')
    ctrl_mode = 0; 
end

agr_scale = 1.4;                % Agresif mod kazanç çarpanı (%40 artış)

% 5) Yapay Sinir Ağı (MLP) Ağırlıklarının Yüklenmesi (Mod 2 İçin)
if exist('nags_pid_weights.json', 'file')
    json_str = fileread('nags_pid_weights.json');
    weights  = jsondecode(json_str);
    
    % MATLAB sürümüne göre cell/double kontrolü ile güvenli aktarım
    if iscell(weights.W1), W1 = cell2mat(weights.W1); else, W1 = weights.W1; end
    if iscell(weights.b1), b1 = cell2mat(weights.b1); else, b1 = weights.b1; end
    if iscell(weights.W2), W2 = cell2mat(weights.W2); else, W2 = weights.W2; end
    if iscell(weights.b2), b2 = cell2mat(weights.b2); else, b2 = weights.b2; end
    if iscell(weights.W3), W3 = cell2mat(weights.W3); else, W3 = weights.W3; end
    if iscell(weights.b3), b3 = cell2mat(weights.b3); else, b3 = weights.b3; end
    
    gamma = 0.40;               % %40 Güvenlik ölçekleme sınırı
end

disp(['quad_params_v2 yüklendi | Aktif Kontrol Modu (ctrl_mode): ', num2str(ctrl_mode)]);