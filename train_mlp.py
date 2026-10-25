import json
import time
import numpy as np

# --- 1. FİZİKSEL PARAMETRELER ---
m = 1.0; g = 9.81; tau_m = 0.05
w_hover = 700.0
k_f = (m * g) / (4 * w_hover**2)
w_max = np.sqrt(19.62 / (4 * k_f))

# Nominal PID Kazançları
Kp0 = 27.312; Ki0 = 32.0; Kd0 = 7.656

# Ölçüm gürültüsü (kontrolcü gürültülü z ve z_dot görür)
SIGMA_Z = 0.005   # [m]
SIGMA_V = 0.05    # [m/s]
W_EFFORT = 40.0   # kontrol eforu (U_cmd sıçraması) ceza ağırlığı
TEST_SEED = 1000  # test gürültüsü, eğitimde kullanılmaz

# 9 eğitim senaryosu: (z_hedef, darbe[N], sinüs genlik[N], sinüs frekans[rad/s])
TRAIN_SET = [
    (0.15, -3.0, 2.0, 1.0), (0.20, -3.0, 4.0, 1.5), (0.30, -3.0, 2.5, 3.0),
    (0.20, -4.5, 2.0, 1.0), (0.30, -4.5, 4.0, 1.5), (0.15, -4.5, 2.5, 3.0),
    (0.30, -8.0, 2.0, 1.0), (0.15, -8.0, 4.0, 1.5), (0.20, -8.0, 2.5, 3.0),
]


# --- 2. YENİ RK4 İNTEGRATÖRÜ (Motor Gecikmesi İçerir) ---
def rk4_step(state, U_sat, t, dt, get_wind_func):
    def dynamics(st, t_eval):
        sz, sv, sw = st
        
        # A) Motor Dinamiği (Birinci Derece Gecikme)
        w_cmd = np.sqrt(max(0.0, U_sat) / (4 * k_f))
        w_cmd = min(w_cmd, w_max)
        dw_dt = (w_cmd - sw) / tau_m
        
        # B) Quadrotor Dinamiği
        Thrust = 4 * k_f * (sw**2)
        F_drag = -0.1 * sv * abs(sv)
        d_wind = get_wind_func(t_eval)  # Rüzgarı o anki zamana göre çek
        
        dv_dt = (Thrust - m * g + F_drag + d_wind) / m
        dz_dt = sv
        
        return np.array([dz_dt, dv_dt, dw_dt])

    k1 = dynamics(state, t)
    k2 = dynamics(state + 0.5 * dt * k1, t + 0.5 * dt)
    k3 = dynamics(state + 0.5 * dt * k2, t + 0.5 * dt)
    k4 = dynamics(state + dt * k3, t + dt)
    
    return state + (dt / 6.0) * (k1 + 2*k2 + 2*k3 + k4)


# --- 3. MLP POLICY SINIFI ---
class MLPPolicy:
    def __init__(self, seed=42):
        np.random.seed(seed)
        self.W1 = np.random.randn(16, 3) * 0.2
        self.b1 = np.zeros((16, 1))
        self.W2 = np.random.randn(16, 16) * 0.2
        self.b2 = np.zeros((16, 1))
        self.W3 = np.random.randn(3, 16) * 0.2
        self.b3 = np.zeros((3, 1))

    def get_params(self):
        return np.concatenate([
            self.W1.ravel(), self.b1.ravel(),
            self.W2.ravel(), self.b2.ravel(),
            self.W3.ravel(), self.b3.ravel(),
        ])

    def set_params(self, params):
        self.W1 = params[0:48].reshape(16, 3)
        self.b1 = params[48:64].reshape(16, 1)
        self.W2 = params[64:320].reshape(16, 16)
        self.b2 = params[320:336].reshape(16, 1)
        self.W3 = params[336:384].reshape(3, 16)
        self.b3 = params[384:387].reshape(3, 1)

    def forward(self, e, v, Ie, bound_scale=0.40):
        # Durum normalizasyonları
        x = np.array([[e / 0.5], [v / 2.0], [Ie / 2.0]])

        h1 = np.maximum(0, np.dot(self.W1, x) + self.b1)
        h2 = np.maximum(0, np.dot(self.W2, h1) + self.b2)
        out = np.tanh(np.dot(self.W3, h2) + self.b3)

        dKp = out[0, 0] * bound_scale
        dKi = out[1, 0] * bound_scale
        dKd = out[2, 0] * bound_scale

        return Kp0 * (1.0 + dKp), Ki0 * (1.0 + dKi), Kd0 * (1.0 + dKd)


# --- 4. SİMÜLASYON MOTORU ---
def simulate_scenario(policy, z_target, w_pulse, s_amp, s_freq, fixed_mult=None, noise_seed=None):
    dt = 0.005
    T = 20.0
    steps = int(T / dt)
    
    def get_wind(t_eval):
        if 5.0 <= t_eval < 8.0:
            return w_pulse
        elif t_eval >= 12.0:
            return s_amp * np.sin(s_freq * (t_eval - 12.0))
        return 0.0

    state = np.array([0.0, 0.0, w_hover])  # [z, v, w]
    Ie = 0.0

    if noise_seed is None:
        nz = np.zeros(steps); nv = np.zeros(steps)
    else:
        rng = np.random.default_rng(noise_seed)
        nz = rng.standard_normal(steps) * SIGMA_Z
        nv = rng.standard_normal(steps) * SIGMA_V
    
    z_hist = np.zeros(steps)
    zr_hist = np.zeros(steps)
    u_raw_hist = np.zeros(steps)
    u_sat_hist = np.zeros(steps)
    Kp_h = np.zeros(steps); Ki_h = np.zeros(steps); Kd_h = np.zeros(steps)
    
    for i in range(steps):
        t = i * dt
        z, v, w = state
        
        zr = 0.0 if t < 1.0 else z_target
        z_m = z + nz[i]          # kontrolcünün gördüğü ölçümler
        v_m = v + nv[i]
        e = zr - z_m
        
        # Kontrolcü Seçimi (Sabit PID mi, Ağ mı?)
        if fixed_mult is not None:
            Kp, Ki, Kd = Kp0 * fixed_mult, Ki0 * fixed_mult, Kd0 * fixed_mult
        else:
            Kp, Ki, Kd = policy.forward(e, v_m, Ie)
            
        U_cmd = m*g + Kp*e - Kd*v_m + Ki*Ie
        U_sat = np.clip(U_cmd, 0.0, 19.62)
        
        # Anti-Windup Clamping (Sınır ihlali yoksa Ie'yi güncelle)
        if 0.0 < U_cmd < 19.62:
            Ie += e * dt
            
        # Loglar
        z_hist[i] = z; zr_hist[i] = zr
        u_raw_hist[i] = U_cmd; u_sat_hist[i] = U_sat
        Kp_h[i] = Kp; Ki_h[i] = Ki; Kd_h[i] = Kd
        
        state = rk4_step(state, U_sat, t, dt, get_wind)
        
    t_hist = np.linspace(0, T, steps, endpoint=False)
    return t_hist, z_hist, zr_hist, u_raw_hist, u_sat_hist, Kp_h, Ki_h, Kd_h


# --- 5. MALİYET (COST) FONKSİYONU ---
def evaluate_cost(t, z, zr, u_raw, Kp_h, Ki_h, Kd_h, dt=0.005):
    m_step = (t >= 1.0) & (t < 5.0)
    m_drop = (t >= 5.0) & (t < 8.0)
    m_peak = (t >= 8.0) & (t < 12.0)
    m_sine = t >= 12.0

    # 0. Basamak aşımı (referansın üstüne çıkan en büyük değer)
    step_over = np.maximum(0, z[m_step] - zr[m_step]).max()

    # 1. ITAE Performansı (Dinamik referans hatası)
    e_all = np.abs(zr - z)
    itae = np.sum(t * e_all * dt)

    # 2. Rüzgar Çukuru ve Fırlama Cezaları (Dinamik)
    wind_drop = np.maximum(0, zr[m_drop] - z[m_drop]).max() if np.any(m_drop) else 0.0
    wind_peak = np.maximum(0, z[m_peak] - zr[m_peak]).max() if np.any(m_peak) else 0.0

    # 3. Sinüs İzleme RMS Hata (Dinamik)
    e_sine = zr[m_sine] - z[m_sine]
    sine_rms = np.sqrt(np.mean(e_sine**2)) if np.any(m_sine) else 0.0

    # 4. Kazanç Titreşimi (Chatter Cezası)
    chatter = np.mean((np.diff(Kp_h)/dt)**2 + (np.diff(Ki_h)/dt)**2 + (np.diff(Kd_h)/dt)**2)

    # 5. DÜZELTİLMİŞ Eyleyici Doyum Cezası (Ham ucmd üzerinden normalize edilmiş MSE)
    # Motora ait U_max = 19.62, Hover U = 9.81.
    sat_pen = np.mean(np.maximum(0, np.abs(u_raw - 9.81) - 9.81)**2) / (9.81**2)

    # 6. Kontrol eforu: ardışık U_cmd farkı (gürültüyü büyüten kazançları cezalar)
    effort = np.mean(np.diff(u_raw)**2)

    # Toplam Ağırlıklı Maliyet
    cost = (
        1.0   * itae          # genel takip hatası (zaman ağırlıklı)
        + 150.0 * step_over     # basamakta referansı aşma [m]
        + 150.0 * wind_drop     # rüzgâr darbesinde en büyük çukur [m]
        + 25.0  * wind_peak     # darbe sonrası fırlama [m]
        + 25.0  * sine_rms      # sinüs rüzgârda RMS hata [m]
        + 0.0005 * chatter      # kazançların değişim hızı
        + 60.0  * sat_pen       # ham U_cmd'nin sınırı aşması (normalize)
        + W_EFFORT * effort     # U_cmd'nin ardışık adım farkı (titreşim)
    )
    return cost


# --- 6. EVRİMSEL STRATEJİLER (ES) EĞİTİM DÖNGÜSÜ ---
def scenario_cost(policy, k, fixed_mult=None):
    z_t, p_val, s_amp, s_freq = TRAIN_SET[k]
    t, z, zr, u_raw, u_sat, Kp_h, Ki_h, Kd_h = simulate_scenario(
        policy, z_t, p_val, s_amp, s_freq, fixed_mult=fixed_mult, noise_seed=k)
    return evaluate_cost(t, z, zr, u_raw, Kp_h, Ki_h, Kd_h)


def train_es(seed=42):
    policy = MLPPolicy(seed=seed)
    rng = np.random.default_rng(seed)
    center_params = policy.get_params()
    num_params = len(center_params)

    pop_size = 50
    generations = 40
    sigma = 0.12
    lr = 0.06

    # Her senaryonun nominal PID maliyeti -> maliyetler bağıl (nominal = 1.0) olur,
    # böylece farklı senaryolarda alınan maliyetler birbiriyle kıyaslanabilir.
    nom_cost = np.array([scenario_cost(policy, k, fixed_mult=1.0) for k in range(len(TRAIN_SET))])

    def full_cost(params):
        # 9 senaryonun hepsinde ortalama bağıl maliyet (checkpoint seçimi için)
        policy.set_params(params)
        return float(np.mean([scenario_cost(policy, k) / nom_cost[k] for k in range(len(TRAIN_SET))]))

    best_full, best_params = full_cost(center_params), center_params.copy()
    print(f'NAGS-PID Eğitimi Başlıyor | Parametre: {num_params} | Nominal PID bağıl maliyet = 1.0000')
    start_time = time.time()

    for gen in range(generations):
        k = int(rng.integers(len(TRAIN_SET)))      # her nesilde tüm popülasyon aynı senaryo
        noise = rng.standard_normal((pop_size // 2, num_params))
        pop_noise = np.vstack([noise, -noise])
        pop_params = center_params + sigma * pop_noise
        costs = np.zeros(pop_size)

        for i in range(pop_size):
            policy.set_params(pop_params[i])
            costs[i] = scenario_cost(policy, k) / nom_cost[k]

        A = (costs - np.mean(costs)) / (np.std(costs) + 1e-8)
        grad = np.dot(pop_noise.T, A) / (pop_size * sigma)
        center_params = center_params - lr * grad

        # Merkez ağırlığı 9 senaryoda değerlendir, en iyisini sakla
        fc = full_cost(center_params)
        if fc < best_full:
            best_full = fc
            best_params = center_params.copy()

        z_t, p_val, _, _ = TRAIN_SET[k]
        print(f'Nesil {gen+1:02d}/{generations} | Senaryo {k} (z:{z_t:.2f}, Rüz:{p_val}N) | '
              f'Nesil ort.: {np.mean(costs):.3f} | Merkez (9 senaryo): {fc:.4f} | En iyi: {best_full:.4f}')

    print(f'\nEğitim Bitti! Süre: {time.time() - start_time:.2f}s')
    policy.set_params(best_params)
    return policy, best_params


# --- 7. BİREBİR KARŞILAŞTIRMA (TEST) TABLOSU ---
def compute_metrics(t, z, zr, u_raw):
    # Senin belirlediğin test senaryosunun (z=0.2) metrik hesaplamaları
    m1 = (t >= 1.0) & (t < 5.0)
    m2 = (t >= 5.0) & (t < 8.0)
    m3 = (t >= 8.0) & (t < 12.0)
    m4 = (t >= 12.0)
    
    overshoot = (np.max(z[m1]) - 0.2) / 0.2 * 100.0 if np.any(m1) else 0.0
    dip = 0.2 - np.min(z[m2]) if np.any(m2) else 0.0
    rebound = np.max(z[m3]) - 0.2 if np.any(m3) else 0.0
    
    ta = t[m3]
    e_abs = np.abs(0.2 - z[m3])
    idx = np.where(e_abs > 0.01)[0]
    if len(idx) == 0: settle = 0.0
    elif idx[-1] == len(ta) - 1: settle = float('nan')
    else: settle = ta[idx[-1] + 1] - 8.0
    
    p2p = np.max(z[m4]) - np.min(z[m4]) if np.any(m4) else 0.0
    rms = np.sqrt(np.mean((0.2 - z[m4])**2)) if np.any(m4) else 0.0
    itae = np.sum(t * np.abs(zr - z) * 0.005)
    
    jit = np.sqrt(np.mean(np.diff(u_raw)**2))
    return [overshoot, dip, rebound, settle, p2p, rms, itae, np.max(u_raw), jit]

def run_final_tests(trained_policy, noisy):
    print("\n" + "="*70)
    print("TEST SENARYOSU: -6 N Darbe ve 3N 2rad/s Sinüs (Eğitimde GÖRÜLMEDİ!) | Gürültü:", "VAR" if noisy else "YOK")
    print("="*70)
    
    headers = ["Metrik", "Nominal PID (Mod 0)", "1.4x Sabit (Mod 1)", "NAGS-PID (Mod 2)"]
    print(f"{headers[0]:<15} | {headers[1]:<20} | {headers[2]:<20} | {headers[3]:<20}")
    print("-" * 80)
    
    # Test Senaryosu Parametreleri
    tz = 0.2; tp = -6.0; sa = 3.0; sf = 2.0
    ns = TEST_SEED if noisy else None
    
    # 1. Mod 0 (Nominal)
    t, z0, zr0, u_raw0, _, _, _, _ = simulate_scenario(trained_policy, tz, tp, sa, sf, fixed_mult=1.0, noise_seed=ns)
    M0 = compute_metrics(t, z0, zr0, u_raw0)
    
    # 2. Mod 1 (1.4x Sabit)
    t, z1, zr1, u_raw1, _, _, _, _ = simulate_scenario(trained_policy, tz, tp, sa, sf, fixed_mult=1.4, noise_seed=ns)
    M1 = compute_metrics(t, z1, zr1, u_raw1)
    
    # 3. Mod 2 (NAGS-PID)
    t, z2, zr2, u_raw2, _, _, _, _ = simulate_scenario(trained_policy, tz, tp, sa, sf, fixed_mult=None, noise_seed=ns)
    M2 = compute_metrics(t, z2, zr2, u_raw2)
    
    metric_names = ["Aşım (%)", "Çukur (m)", "Fırlama (m)", "Yerleşme (s)", "Sinüs p2p (m)", "RMS (m)", "ITAE", "Max U_cmd (N)", "U titreşim RMS (N)"]
    for i in range(9):
        print(f"{metric_names[i]:<15} | {M0[i]:<20.4f} | {M1[i]:<20.4f} | {M2[i]:<20.4f}")


if __name__ == '__main__':
    best_policy, best_weights = train_es()

    weights_dict = {
        'W1': best_policy.W1.tolist(), 'b1': best_policy.b1.tolist(),
        'W2': best_policy.W2.tolist(), 'b2': best_policy.b2.tolist(),
        'W3': best_policy.W3.tolist(), 'b3': best_policy.b3.tolist(),
    }
    with open('nags_pid_weights.json', 'w') as f:
        json.dump(weights_dict, f, indent=4)
        
    print("\n>>> Güncellenmiş ağırlıklar 'nags_pid_weights.json' dosyasına kaydedildi! <<<")
    run_final_tests(best_policy, noisy=False)
    run_final_tests(best_policy, noisy=True)