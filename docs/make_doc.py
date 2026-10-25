import docx
from docx import Document
from docx.enum.table import WD_ALIGN_VERTICAL, WD_TABLE_ALIGNMENT
from docx.enum.text import WD_ALIGN_PARAGRAPH
from docx.oxml import OxmlElement
from docx.oxml.ns import qn
from docx.shared import Inches, Pt, RGBColor


def set_cell_background(cell, fill_color):
  tcPr = cell._element.get_or_add_tcPr()
  shd = OxmlElement('w:shd')
  shd.set(qn('w:val'), 'clear')
  shd.set(qn('w:color'), 'auto')
  shd.set(qn('w:fill'), fill_color)
  tcPr.append(shd)


def set_cell_margins(cell, top=100, bottom=100, left=150, right=150):
  tcPr = cell._element.get_or_add_tcPr()
  tcMar = OxmlElement('w:tcMar')
  for m, val in [
      ('top', top),
      ('bottom', bottom),
      ('left', left),
      ('right', right),
  ]:
    node = OxmlElement(f'w:{m}')
    node.set(qn('w:w'), str(val))
    node.set(qn('w:type'), 'dxa')
    tcMar.append(node)
  tcPr.append(tcMar)


doc = Document()

# Sayfa Kenar Boşlukları (Standart Academic Margin: 2.54 cm / 1 inch)
for section in doc.sections:
  section.top_margin = Inches(1)
  section.bottom_margin = Inches(1)
  section.left_margin = Inches(1)
  section.right_margin = Inches(1)

# Stil Ayarları
style_normal = doc.styles['Normal']
font = style_normal.font
font.name = 'Calibri'
font.size = Pt(11)
font.color.rgb = RGBColor(0x22, 0x22, 0x22)

# BAŞLIK (TITLE)
title_p = doc.add_paragraph()
title_p.alignment = WD_ALIGN_PARAGRAPH.CENTER
run_title = title_p.add_run(
    'YAPAY SİNİR AĞI TABANLI ADAPTİF KAZANÇ AYARLAMALI PID (NAGS-PID) KONTROL'
    ' MİMARİSİ VE EVRİMSEL EĞİTİM YÖNTEMİ\n'
)
run_title.bold = True
run_title.font.size = Pt(16)
run_title.font.color.rgb = RGBColor(0x1B, 0x36, 0x5D)  # Lacivert akademik ton

run_sub = title_p.add_run(
    'TÜBİTAK 2209-A / Araştırma Projesi Teknik Rapor Bölümü'
)
run_sub.italic = True
run_sub.font.size = Pt(11)
run_sub.font.color.rgb = RGBColor(0x55, 0x55, 0x55)

doc.add_paragraph()


def add_custom_heading(text, level):
  p = doc.add_paragraph()
  p.paragraph_format.space_before = Pt(12)
  p.paragraph_format.space_after = Pt(4)
  run = p.add_run(text)
  run.bold = True
  if level == 1:
    run.font.size = Pt(13)
    run.font.color.rgb = RGBColor(0x1B, 0x36, 0x5D)
  elif level == 2:
    run.font.size = Pt(12)
    run.font.color.rgb = RGBColor(0x00, 0x5A, 0x9E)
  return p


# 1. GİRİŞ
add_custom_heading('1. Giriş ve Teorik Motivasyon', 1)
p = doc.add_paragraph(
    'Klasik PID kontrolcüler, sabitleştirilmiş kazanç parametreleri (Kp, Ki,'
    ' Kd) ile lineer zamanla değişmeyen (LTI) sistemlerde yüksek performans'
    ' sunarlar. Ancak, 1D dikey eksende hareket eden quadrotor sistemleri;'
    ' aerodinamik hava sürüklemesi (cd*v*|v|), eyleyici doyumu (U_max) ve ani dış'
    ' bozunumlar (d(t) rüzgar yükü) nedeniyle yüksek dereceden lineer olmayan'
    ' (non-linear) dinamikler sergiler.'
)
p.paragraph_format.line_spacing = 1.15
p.paragraph_format.space_after = Pt(6)

p = doc.add_paragraph(
    'Çalışma noktasının geniş bir yelpazede değiştiği durumlarda sabit kazançlı'
    ' PID kontrolcüler aşım (overshoot), kararsızlık veya rüzgar altında'
    ' hedeften sapma (wind drop) problemlerine yol açar. Bu çalışmada önerilen'
    ' Neural Adaptive Gain-Scheduled PID (NAGS-PID) mimarisi, sistemin anlık'
    ' durum hatalarını ve hızını işleyerek nominal PID kazançlarını gerçek'
    ' zamanlı ve adaptif olarak modüle eder. Böylece hem klasik kontrolün'
    ' kararlılık garantisi korunmakta hem de derin öğrenmenin esnek adaptasyon'
    ' kabiliyeti sisteme entegre edilmektedir.'
)
p.paragraph_format.line_spacing = 1.15
p.paragraph_format.space_after = Pt(6)

# 2. MLP MİMARİSİ
add_custom_heading(
    '2. Yapay Sinir Ağı (MLP) Mimarisi ve Güvenlik Katmanı', 1
)
p = doc.add_paragraph(
    'NAGS-PID yapısında kullanılan Çok Katmanlı Algılayıcı (MLP), sistem'
    ' durumlarını alıp nominal kazançlara uygulanacak bağıntılı değişim'
    ' katsayılarını (dKp, dKi, dKd) üreten sığ bir derin öğrenme modelidir.'
)

add_custom_heading('2.1. Giriş Normalizasyonu', 2)
p = doc.add_paragraph(
    'Yapay sinir ağının gradyan kilitlenmesine girmesini önlemek ve eğitimin'
    ' kararlılığını sağlamak amacıyla durum değişkenleri beklenen fiziki çalışma'
    ' aralıklarına göre normalize edilerek x ∈ ℝ^(3x1) vektörü oluşturulur:'
)
p_eq = doc.add_paragraph(
    'x(t) = [ e(t) / 0.5 ,  v(t) / 2.0 ,  Ie(t) / 2.0 ]ᵀ'
)
p_eq.alignment = WD_ALIGN_PARAGRAPH.CENTER
p_eq.runs[0].bold = True

p = doc.add_paragraph(
    'Burada e(t) = z_ref(t) - z(t) konum hatasını, v(t) = z_nokta(t) dikey hızı'
    ' ve Ie(t) = ∫e(t)dt birikmiş entegral hatasını temsil eder.'
)

add_custom_heading('2.2. Katman Yapısı ve Aktivasyon Fonksiyonları', 2)
p = doc.add_paragraph(
    'Ağ mimarisi 3 → 16 → 16 → 3 şeklinde tasarlanmış olup toplam 387 adet'
    ' eğitilebilir parametreye (ağırlıklar ve biyazlar) sahiptir:'
)
p_eq = doc.add_paragraph(
    'θ = [ vec(W1), b1ᵀ, vec(W2), b2ᵀ, vec(W3), b3ᵀ ]ᵀ ∈ ℝ³⁸⁷'
)
p_eq.alignment = WD_ALIGN_PARAGRAPH.CENTER
p_eq.runs[0].bold = True

p_bullet1 = doc.add_paragraph(style='List Bullet')
p_bullet1.add_run('Gizli Katmanlar: ').bold = True
p_bullet1.add_run(
    'Gizli katmanlarda doğrusal olmayan özellikleri yakalamak için ReLU'
    ' (ReLU(a) = max(0, a)) aktivasyon fonksiyonu kullanılmıştır:\n'
    'h1 = ReLU(W1·x + b1),  h1 ∈ ℝ¹⁶ˣ¹\n'
    'h2 = ReLU(W2·h1 + b2),  h2 ∈ ℝ¹⁶ˣ¹'
)

p_bullet2 = doc.add_paragraph(style='List Bullet')
p_bullet2.add_run('Çıkış Katmanı: ').bold = True
p_bullet2.add_run(
    'Ağın çıktıları [-1, +1] aralığına Tanh aktivasyonu ile'
    ' sıkıştırılır:\nout = tanh(W3·h2 + b3),  out ∈ ℝ³ˣ¹'
)

add_custom_heading('2.3. Güvenlik Sınırı (Safety Bound Scaling)', 2)
p = doc.add_paragraph(
    'Kontrol sisteminde ağın başlangıçta rastgele üreteceği çıktılardan dolayı'
    ' sistemin kararsızlığa girmesini engellemek adına, azami değişim oranı %40'
    ' (γ = 0.40) ile sınırlandırılmıştır. Aktif kontrol kazançları nominal'
    ' kazançlar (Kp0, Ki0, Kd0) üzerinden şu şekilde hesaplanır:'
)
p_eq = doc.add_paragraph(
    'dKp = out[0] · γ,   dKi = out[1] · γ,   dKd = out[2] · γ\n'
    'Kp(t) = Kp0 · (1 + dKp) ∈ [0.60 Kp0, 1.40 Kp0]\n'
    'Ki(t) = Ki0 · (1 + dKi) ∈ [0.60 Ki0, 1.40 Ki0]\n'
    'Kd(t) = Kd0 · (1 + dKd) ∈ [0.60 Kd0, 1.40 Kd0]'
)
p_eq.alignment = WD_ALIGN_PARAGRAPH.CENTER

# 3. FİZİKSEL SİSTEM DİNAMİĞİ
add_custom_heading('3. Fiziksel Sistem Dinamiği ve Sayısal İntegrasyon', 1)
p = doc.add_paragraph(
    'Quadrotorün 1D dikey eksendeki non-linear hareket denklemi Newton-Euler'
    ' mantığıyla türetilmiştir:'
)
p_eq = doc.add_paragraph(
    'm · z_çift_nokta(t) = U_sat(t) - m·g - cd·v(t)·|v(t)| + d(t)'
)
p_eq.alignment = WD_ALIGN_PARAGRAPH.CENTER
p_eq.runs[0].bold = True

p = doc.add_paragraph(
    'Burada m = 1.0 kg toplam kütleyi, g = 9.81 m/s² yerçekimi ivmesini, cd = 0.1'
    ' N/(m/s)² aerodinamik sürükleme katsayısını ve d(t) dış rüzgar kuvvetini'
    ' temsil eder.'
)

add_custom_heading('3.1. Kontrol Yasası ve Anti-Windup Mekanizması', 2)
p = doc.add_paragraph(
    'Türev patlamasını (Derivative Kick) engellemek amacıyla türev terimi'
    ' konuma değil doğrudan ölçülen dikey hıza yazılmıştır (Derivative on'
    ' Measurement). Kontrol sinyali ve doyum hesabı:'
)
p_eq = doc.add_paragraph(
    'U_cmd(t) = m·g + Kp(t)·e(t) - Kd(t)·v(t) + Ki(t)·Ie(t)\n'
    'U_sat(t) = clamp(U_cmd(t), 0.0, 19.62 N)'
)
p_eq.alignment = WD_ALIGN_PARAGRAPH.CENTER

p = doc.add_paragraph(
    'Sistemin doyuma girdiği durumlarda entegral teriminin şişmesini engellemek'
    ' için Clamping Anti-Windup uygulanır: e_int_türev(t) = e(t) (eğer 0 <'
    ' U_cmd < U_max) aksi durumda 0.'
)

add_custom_heading(
    '3.2. 4. Derece Runge-Kutta (RK4) Sayısal İntegrasyonu', 2
)
p = doc.add_paragraph(
    'Simülasyon doğruluğunu ve sayısal kararlılığı sağlamak için Euler yöntemi'
    ' yerine dt = 0.005 s (200 Hz) adım boyutuna sahip RK4 integratörü'
    ' kullanılmıştır. Adım türevleri k1, k2, k3, k4 ağırlıklı ortalaması'
    ' alınarak x_(k+1) = x_k + (dt/6)·(k1 + 2k2 + 2k3 + k4) biçiminde güncellenir.'
)

# 4. EVRİMSEL STRATEJİLER (ES)
add_custom_heading(
    '4. Gradyansız Evrimsel Stratejiler (ES) İle Optimizasyon', 1
)
p = doc.add_paragraph(
    'Simülasyon ortamında yer alan eyleyici doyumu (clip), zaman aralıklı rüzgar'
    ' bozunumu ve RK4 sayısal entegrasyonu nedeniyle uçuş simülasyonunun'
    ' analitik türevi (∂J/∂θ) alınamaz. Bu nedenle geri yayılım (Backprop)'
    ' yerine Siyah-Kutu Gradyan Tahminleyicisi (Black-Box Gradient Estimation)'
    ' mantığına dayanan Evrimsel Stratejiler (ES) algoritması seçilmiştir.'
)

add_custom_heading('4.1. Antitetik Örnekleme (Antithetic Sampling)', 2)
p = doc.add_paragraph(
    'Varyansı düşürmek ve optimizasyon hızını iki katına çıkarmak için her'
    ' jenerasyonda popülasyon boyutunun yarısı kadar (N_half = 25) ε ~ 𝒩(0, I)'
    ' rastgele gürültü vektörü üretilir. Popülasyonun diğer yarısı ise bu'
    ' gürültülerin tam ters işaretlilerinden oluşturulur. 50 adet bireyin'
    ' parametreleri θ_i = θ_center + σ·ε_i (σ = 0.15) şeklinde mutasyona uğratılır.'
)

add_custom_heading(
    '4.2. Performans Normalizasyonu ve Güncelleme Kuralı', 2
)
p = doc.add_paragraph(
    'Her bireyin maliyeti J_i z-score yöntemiyle normalize edilir: A_i = (J_i -'
    ' μ_J) / (σ_J + 10⁻⁸). Gradyan yönü tahmini ve merkez parametre güncellemesi'
    ' şu şekilde icra edilir:'
)
p_eq = doc.add_paragraph(
    'grad = (1 / (N·σ)) · ∑ (A_i · ε_i)\nθ_center ← θ_center - α · grad  (α ='
    ' 0.08)'
)
p_eq.alignment = WD_ALIGN_PARAGRAPH.CENTER
p_eq.runs[0].bold = True

# 5. MALİYET FONKSİYONU VE TABLO
add_custom_heading(
    '5. Çok Amaçlı Maliyet (Cost / Loss) Fonksiyonu Tasarımı', 1
)
p = doc.add_paragraph(
    'Sistemin hedeflenen tüm uçuş kriterlerini tek bir skalar değer üzerinden'
    ' optimize edebilmesi için 6 bileşenli ağırlıklı maliyet fonksiyonu J'
    ' tanımlanmıştır:'
)
p_eq = doc.add_paragraph(
    'J = w1·J_ITAE + w2·J_drop + w3·J_peak + w4·J_rms + w5·J_chatter +'
    ' w6·J_sat'
)
p_eq.alignment = WD_ALIGN_PARAGRAPH.CENTER
p_eq.runs[0].bold = True

add_custom_heading('5.1. Maliyet Bileşenleri ve Tasarım Gerekçeleri', 2)

# TABLO OLUŞTURMA
table = doc.add_table(rows=7, cols=4)
table.alignment = WD_TABLE_ALIGNMENT.CENTER

headers = [
    'Maliyet Bileşeni',
    'Katsayı (wi)',
    'Büyüklük Mertebesi',
    'Mühendislik Amacı ve Tasarım Gerekçesi',
]
hdr_cells = table.rows[0].cells
for i, header_text in enumerate(headers):
  hdr_cells[i].text = header_text
  hdr_cells[i].paragraphs[0].runs[0].bold = True
  hdr_cells[i].paragraphs[0].runs[0].font.color.rgb = RGBColor(
      0xFF, 0xFF, 0xFF
  )
  set_cell_background(hdr_cells[i], '1B365D')
  set_cell_margins(hdr_cells[i], top=120, bottom=120, left=150, right=150)

data = [
    (
        'J_ITAE',
        '1.0',
        '~ 10 - 15',
        (
            'Sistem yükselme süresini optimize eder. Geç zaman hatalarını'
            ' katlanarak cezalandırır.'
        ),
    ),
    (
        'J_drop',
        '12.0',
        '~ 1.5 - 2.5',
        (
            '5-8 s arasındaki -6 N rüzgarda drone\'un aşağı çökmesini engeller.'
            ' Ağa rüzgarı hissettiği an kazançları artırma refleksini öğretir.'
        ),
    ),
    (
        'J_peak',
        '12.0',
        '~ 1.5 - 2.5',
        (
            '8. saniyede rüzgar aniden kesildiğinde biriken entegralden dolayı'
            ' drone\'un yukarı fırlamasını (overshoot) engeller.'
        ),
    ),
    (
        'J_sine',
        '20.0',
        '~ 1.0 - 2.0',
        (
            '12-20 s aralığındaki sinüzoidal türbülansta referans izleme'
            ' hassasiyetini artırır.'
        ),
    ),
    (
        'J_chatter',
        '0.001',
        '~ 1.0 - 3.0',
        (
            'Kazanç türevlerinin karesini cezalandırarak Kp, Ki, Kd'
            ' değişimlerinin pürüzsüz olmasını sağlar, motor titreşimini'
            ' engeller.'
        ),
    ),
    (
        'J_sat',
        '50.0',
        'Sert Kısıt',
        (
            'Motor komutu 19.62 N limitini aştığı an ağa devasa ceza basarak'
            ' motor doyumu ihlalini engeller.'
        ),
    ),
]

for row_idx, row_data in enumerate(data, start=1):
  row_cells = table.rows[row_idx].cells
  bg_color = 'F2F4F7' if row_idx % 2 == 1 else 'FFFFFF'
  for col_idx, cell_value in enumerate(row_data):
    row_cells[col_idx].text = cell_value
    set_cell_background(row_cells[col_idx], bg_color)
    set_cell_margins(
        row_cells[col_idx], top=100, bottom=100, left=150, right=150
    )

# Sütun Genişlikleri
col_widths = [Inches(1.2), Inches(1.0), Inches(1.3), Inches(3.0)]
for row in table.rows:
  for i, w in enumerate(col_widths):
    row.cells[i].width = w

doc.add_paragraph()

# 6. LİTERATÜRDEKİ YERİ
add_custom_heading(
    '6. Yöntemin Derin Öğrenme ve Kontrol Literatüründeki Yeri', 1
)
p = doc.add_paragraph(
    'Bu projede geliştirilen NAGS-PID mimarisi, klasik gain-scheduling'
    ' yaklaşımlarındaki tablolama (Look-up Table) zorunluluğunu ortadan'
    ' kaldırmaktadır. Çevrimdışı evrimsel optimizasyon sayesinde model, türev'
    ' gereksinimi duymadan karmaşık simülasyon ortamında eğitilmiş; elde edilen'
    ' ağırlık matrisleri ise doğrudan MATLAB/Simulink ortamına yerleştirilebilir'
    " JSON formatına ('nags_pid_weights.json') aktarılmıştır. Mimari, düşük"
    ' işlem gücüne sahip mikrodenetleyicilerde (örneğin STM32) mikro-saniye'
    ' mertebesinde ileri besleme (Forward Pass) koşturulabilecek hafiflikte (387'
    ' parametre) tasarlanmıştır.'
)

# Dosyayı Kaydet
output_filename = 'NAGS_PID_TUBITAK_Raporu.docx'
doc.save(output_filename)
print(
    f"Rapor başarıyla oluşturuldu ve '{output_filename}' adıyla kaydedildi!"
)