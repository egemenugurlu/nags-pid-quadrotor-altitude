%% quad_params.m - 4 motorlu Quadrotor (1D irtifa) parametreleri

%% 1) Fiziksel parametreler
m       = 1.0;                  % kutle [kg]
g       = 9.81;                 % yercekimi [m/s^2]
c_d     = 0.1;                  % surtunme katsayisi [kg/m]
w_hover = 700;                  % hover motor hizi [rad/s]
k_f     = m*g/(4*w_hover^2);    % itki katsayisi, T_i = k_f*w_i^2
tau_m   = 0.05;                 % motor zaman sabiti [s]
U_max   = 2*m*g;                % toplam max itki [N]
w_max   = sqrt(U_max/(4*k_f));  % motor max hizi [rad/s]
z0      = 0;                    % baslangic irtifasi [m]

%% 2) PID tasarim parametreleri (kutup atama)
zeta  = 0.707;                  % sonum orani
wn    = 4;                      % dogal frekans [rad/s]
alpha = 2;                      % integral kutbu

%% 3) Ruzgar bozucusu
wind_amp = -6;                  % genlik [N], asagi yonlu
wind_t0  = 5;                   % baslangic zamani [s]
wind_dur = 3;                   % sure [s]

if ~exist('ctrl_mode', 'var')
    ctrl_mode = 0; 
end

agr_scale = 1.4; % Agresif mod çarpanı (%40 artış)

% MLP Yapay Sinir Ağı Ağırlıklarını Yükle (Mod 2 İçin)
if exist('nags_pid_weights.json', 'file')
    json_str = fileread('nags_pid_weights.json');
    weights = jsondecode(json_str);
    W1 = cell2mat(weights.W1);
    b1 = cell2mat(weights.b1);
    W2 = cell2mat(weights.W2);
    b2 = cell2mat(weights.b2);
    W3 = cell2mat(weights.W3);
    b3 = cell2mat(weights.b3);
    gamma = 0.40;
end