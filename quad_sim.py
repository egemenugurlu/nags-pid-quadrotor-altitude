import numpy as np

# =============================================================================
# 12.2.a Parametreler & Nominal Kazanç Hesaplama
# =============================================================================
m = 1.0           # kg
g = 9.81          # m/s^2
c_d = 0.1         # Sürüklenme katsayısı
w_hover = 700.0   # rad/s
k_f = (m * g) / (4.0 * w_hover**2)
tau_m = 0.05      # s
U_max = 2.0 * m * g  # 19.62 N
w_max = np.sqrt(U_max / (4.0 * k_f))

# Kutup Atama ile Nominal Kazançlar (zeta = 0.707, wn = 4.0, alpha = 2.0)
zeta = 0.707
wn = 4.0
alpha = 2.0

Kp_nom = m * (wn**2 + 2.0 * zeta * wn * alpha)  # 27.312
Ki_nom = m * (alpha * wn**2)                     # 32.000
Kd_nom = m * (2.0 * zeta * wn + alpha)           # 7.656

print(f"Hesaplanan Nominal Kazançlar: Kp={Kp_nom:.3f}, Ki={Ki_nom:.3f}, Kd={Kd_nom:.3f}")

# =============================================================================
# 12.2.f & g Vektörize RK4 Simülatörü
# =============================================================================
def simulate(gain_fn, scn_type="test", T=20.0, dt=0.005, B=1):
    """
    Vektörize Quadrotor 1D Simülatörü
    gain_fn: gain_fn(e, v, Ie) -> Kp, Ki, Kd (her biri (B,) boyutunda)
    Outputs: t (N,), z (N, B), zr (N, B), u (N, B), u_cmd (N, B)
    """
    N = int(np.round(T / dt)) + 1
    t = np.linspace(0, T, N)
    
    # Durumlar: [z, v, w, J, Ie] -> Her biri (B,) boyutunda
    z  = np.zeros(B)
    v  = np.zeros(B)
    w  = np.full(B, w_hover)
    J  = np.zeros(B)
    Ie = np.zeros(B)
    
    # Sonuç Matrisleri (N, B)
    z_hist     = np.zeros((N, B))
    zr_hist    = np.zeros((N, B))
    u_hist     = np.zeros((N, B))
    ucmd_hist  = np.zeros((N, B))
    
    # Türev Sağ Taraf Fonksiyonu (RK4 Alt Adımları İçin)
    def rhs(z_i, v_i, w_i, J_i, Ie_i, z_ref_i, w_cmd_i, d_i, Ki_i):
        dz = v_i
        U = 4.0 * k_f * w_i**2
        dv = (U - m * g - c_d * v_i * np.abs(v_i) + d_i) / m
        dw = (w_cmd_i - w_i) / tau_m
        e_i = z_ref_i - z_i
        dJ = Ki_i * e_i
        dIe = e_i
        return dz, dv, dw, dJ, dIe

    for i in range(N):
        ti = t[i]
        
        # Referans z_ref (N, B) veya scalar
        z_ref = 0.2 if ti >= 1.0 else 0.0
        zr_arr = np.full(B, z_ref)
        
        # Rüzgar Bozucusu d(t)
        d_val = 0.0
        if 5.0 <= ti < 8.0:
            d_val = -6.0
        elif ti >= 12.0:
            d_val = 3.0 * np.sin(2.0 * (ti - 12.0))
        d_arr = np.full(B, d_val)
        
        # 1. Hata ve Kazançlar (Adım Başı Hesaplama)
        e = zr_arr - z
        Kp, Ki, Kd = gain_fn(e, v, Ie)
        
        # 2. Kontrol Sinyalleri
        U_cmd = m * g + Kp * e + J - Kd * v
        U_sat = np.clip(U_cmd, 0.0, U_max)
        w_cmd = np.clip(np.sqrt(U_sat / (4.0 * k_f)), 0.0, w_max)
        U_act = 4.0 * k_f * w**2
        
        # Kayıt
        z_hist[i]    = z
        zr_hist[i]   = zr_arr
        u_hist[i]    = U_act
        ucmd_hist[i] = U_cmd
        
        if i == N - 1:
            break
            
        # 3. RK4 Adımı (w_cmd, d_arr ve Ki adım boyunca sabit)
        k1_z, k1_v, k1_w, k1_J, k1_Ie = rhs(z, v, w, J, Ie, zr_arr, w_cmd, d_arr, Ki)
        
        z_k2  = z  + 0.5 * dt * k1_z
        v_k2  = v  + 0.5 * dt * k1_v
        w_k2  = w  + 0.5 * dt * k1_w
        J_k2  = J  + 0.5 * dt * k1_J
        Ie_k2 = Ie + 0.5 * dt * k1_Ie
        k2_z, k2_v, k2_w, k2_J, k2_Ie = rhs(z_k2, v_k2, w_k2, J_k2, Ie_k2, zr_arr, w_cmd, d_arr, Ki)
        
        z_k3  = z  + 0.5 * dt * k2_z
        v_k3  = v  + 0.5 * dt * k2_v
        w_k3  = w  + 0.5 * dt * k2_w
        J_k3  = J  + 0.5 * dt * k2_J
        Ie_k3 = Ie + 0.5 * dt * k2_Ie
        k3_z, k3_v, k3_w, k3_J, k3_Ie = rhs(z_k3, v_k3, w_k3, J_k3, Ie_k3, zr_arr, w_cmd, d_arr, Ki)
        
        z_k4  = z  + dt * k3_z
        v_k4  = v  + dt * k3_v
        w_k4  = w  + dt * k3_w
        J_k4  = J  + dt * k3_J
        Ie_k4 = Ie + dt * k3_Ie
        k4_z, k4_v, k4_w, k4_J, k4_Ie = rhs(z_k4, v_k4, w_k4, J_k4, Ie_k4, zr_arr, w_cmd, d_arr, Ki)
        
        z  += (dt / 6.0) * (k1_z  + 2.0 * k2_z  + 2.0 * k3_z  + k4_z)
        v  += (dt / 6.0) * (k1_v  + 2.0 * k2_v  + 2.0 * k3_v  + k4_v)
        w  += (dt / 6.0) * (k1_w  + 2.0 * k2_w  + 2.0 * k3_w  + k4_w)
        J  += (dt / 6.0) * (k1_J  + 2.0 * k2_J  + 2.0 * k3_J  + k4_J)
        Ie += (dt / 6.0) * (k1_Ie + 2.0 * k2_Ie + 2.0 * k3_Ie + k4_Ie)
        
    return t, z_hist, zr_hist, u_hist, ucmd_hist

# =============================================================================
# 12.3 Metrik Hesaplama Fonksiyonu
# =============================================================================
def compute_metrics(t, z, zr, ucmd, dt=0.005):
    """
    z, zr, ucmd: (N, B) veya (N,) boyutunda diziler
    """
    if z.ndim == 1:
        z = z[:, np.newaxis]
        zr = zr[:, np.newaxis]
        ucmd = ucmd[:, np.newaxis]
        
    B = z.shape[1]
    
    # Maskeler
    m_step  = (t >= 1.0) & (t < 5.0)
    m_drop  = (t >= 5.0) & (t < 8.0)
    m_peak  = (t >= 8.0) & (t < 12.0)
    m_sine  = (t >= 12.0)
    
    # 1. Aşım (%)
    z_max_step = np.max(z[m_step, :], axis=0)
    overshoot = ((z_max_step - 0.2) / 0.2) * 100.0
    
    # 2. Rüzgar Çukuru (m)
    z_min_drop = np.min(z[m_drop, :], axis=0)
    wind_drop = 0.2 - z_min_drop
    
    # 3. Rüzgar Sonrası Fırlama (m)
    z_max_peak = np.max(z[m_peak, :], axis=0)
    wind_peak = z_max_peak - 0.2
    
    # 4. Yerleşme Süresi (s) - %2 tolerans bandı (0.004 m)
    settling_time = np.zeros(B)
    t_peak = t[m_peak]
    z_peak = z[m_peak, :]
    tol = 0.02 * 0.2  # 0.004 m
    for b in range(B):
        out_indices = np.where(np.abs(z_peak[:, b] - 0.2) > tol)[0]
        if len(out_indices) > 0:
            settling_time[b] = t_peak[out_indices[-1]] - 8.0
        else:
            settling_time[b] = 0.0
            
    # 5. Sinüs Tepeden Tepeye (m)
    sine_p2p = np.max(z[m_sine, :], axis=0) - np.min(z[m_sine, :], axis=0)
    
    # 6. Sinüs RMS Hata (m)
    e_sine = 0.2 - z[m_sine, :]
    sine_rms = np.sqrt(np.mean(e_sine**2, axis=0))
    
    # 7. ITAE
    e_all = np.abs(zr - z)
    itae = np.sum(t[:, np.newaxis] * e_all * dt, axis=0)
    
    # 8. Max U_cmd (N)
    max_ucmd = np.max(ucmd, axis=0)
    
    return {
        "Overshoot (%)": overshoot,
        "Wind Drop (m)": wind_drop,
        "Wind Peak (m)": wind_peak,
        "Settling Time (s)": settling_time,
        "Sine P2P (m)": sine_p2p,
        "Sine RMS (m)": sine_rms,
        "ITAE": itae,
        "Max Ucmd (N)": max_ucmd
    }

# =============================================================================
# 12.4 Simülasyon Çalıştırma ve Karşılaştırma
# =============================================================================
if __name__ == "__main__":
    # 1. Nominal PID
    def gain_fn_nom(e, v, Ie):
        B = e.shape[0] if isinstance(e, np.ndarray) else 1
        return np.full(B, Kp_nom), np.full(B, Ki_nom), np.full(B, Kd_nom)

    t, z_nom, zr_nom, u_nom, ucmd_nom = simulate(gain_fn_nom, B=1)
    m_nom = compute_metrics(t, z_nom, zr_nom, ucmd_nom)

    # 2. Agresif PID (1.5x Kazançlar)
    def gain_fn_agg(e, v, Ie):
        B = e.shape[0] if isinstance(e, np.ndarray) else 1
        return np.full(B, 1.5 * Kp_nom), np.full(B, 1.5 * Ki_nom), np.full(B, 1.5 * Kd_nom)

    _, z_agg, zr_agg, u_agg, ucmd_agg = simulate(gain_fn_agg, B=1)
    m_agg = compute_metrics(t, z_agg, zr_agg, ucmd_agg)

    # Tablo Basımı
    print("\n" + "="*75)
    print(f"{'Metrik':<28} | {'Simulink (Ref)':<15} | {'Python (Nominal)':<16} | {'Python (Agresif 1.5x)':<18}")
    print("="*75)
    
    ref_vals = {
        "Overshoot (%)": "33.1 %",
        "Wind Drop (m)": "0.205 m",
        "Wind Peak (m)": "0.203 m",
        "Settling Time (s)": "2.12 s",
        "Sine P2P (m)": "0.260 m",
        "Sine RMS (m)": "0.085 m",
        "ITAE": "12.5967",
        "Max Ucmd (N)": "18.35 N"
    }

    units = {
        "Overshoot (%)": "%", "Wind Drop (m)": "m", "Wind Peak (m)": "m",
        "Settling Time (s)": "s", "Sine P2P (m)": "m", "Sine RMS (m)": "m",
        "ITAE": "", "Max Ucmd (N)": "N"
    }

    for k in ref_vals.keys():
        val_nom = m_nom[k][0]
        val_agg = m_agg[k][0]
        unit = units[k]
        
        fmt_nom = f"{val_nom:.3f} {unit}".strip() if "ITAE" not in k else f"{val_nom:.4f}"
        fmt_agg = f"{val_agg:.3f} {unit}".strip() if "ITAE" not in k else f"{val_agg:.4f}"
        
        print(f"{k:<28} | {ref_vals[k]:<15} | {fmt_nom:<16} | {fmt_agg:<18}")
    print("="*75)