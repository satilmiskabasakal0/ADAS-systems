# ADAS ödevi: bağımsız referans çözüm

Bu klasör, dört görevi baştan sona **denklem → uygulama → deney → doğrulama → rapor**
sırasıyla çözer. Ana projedeki `Task-*` dosyalarını değiştirmez. Türkçe açıklamalar
[teknik raporda](output/pdf/rapor.md), bütün deney sonuçları `output/` altında bulunur.

Mayıs 2025 teslimiyle ilişkisi, doğrulanmış düzeltmeler ve yöntem değişiklikleri
[yeniden inceleme kaydında](../docs/REVIEW_2026-09.md) açıklanır. Bu klasör geçmiş
teslimin yerine geçmez; bağımsız bir referans uygulamadır.

## Başlangıç

Python 3.11 veya üzeri kullanın. Kaydedilmiş deneyler Python 3.14.6 ile çalıştırıldı.
Sisteminizde bu sürümler için paket bulunabilirliği işletim sistemi/Python sürümüne bağlıdır.

```bash
cd adas_reference_solution
python3 -m venv .venv
source .venv/bin/activate
# Windows PowerShell: .venv\Scripts\Activate.ps1
python -m pip install -r requirements.txt
python reproduce.py
```

Son komut testleri çalıştırır, başarısızlıkta durur, deneyleri üretir, PDF ve Markdown
raporlarını oluşturur ve kaynak/veri SHA-256 özetlerini kaydeder. Etkileşimli grafik
penceresi açılmaz; `Agg` arka ucu kullanılır. İnternet bağlantısı yalnız paketlerin
kurulumu için gerekir. Ödev PDF'si veya önceki rapor çalışma anında gerekli değildir.

İşlemleri ayrı çalıştırmak için:

```bash
python -m unittest discover -s tests -v > output/data/test_results.txt 2>&1
# Test başarısızsa sonraki adıma geçmeyin; test_results.txt dosyasını inceleyin.
python run_all.py
python build_report.py
```

`run_all.py --output /hedef/klasor` özel çıktı dizinini destekler. Aynı dizini
`build_report.py --output /hedef/klasor` için de verin ve test günlüğünü onun
`data/test_results.txt` yoluna kaydedin. Varsayılan çıktı yolu çağrı dizinine bağlı değildir.

## Dosya haritası

| Dosya | Sorumluluğu |
|---|---|
| `adas/braking.py` | Sürtünme sınırı, gidilen/kalan mesafe, dümenleme yaklaşımı |
| `adas/trajectory.py` | Sınır koşullu quintic, analitik jerk ve sürekli zaman kontrolleri |
| `adas/tracking.py` | Referans yol, izdüşüm, iki kontrolcü ve yaw-rate dinamiği |
| `adas/speed.py` | Parça sabit yol, tam yay geometrisi, uygulanabilir hız zarfı |
| `tests/test_models.py` | 17 analitik/sınır durumu testi |
| `run_all.py` | Deneyler, figürler, CSV ve JSON sonuçları |
| `build_report.py` | Kaydedilmiş sonuçlardan Türkçe PDF ve Markdown üretimi |
| `reproduce.py` | Bütün akış, testte durdurma, kaynak/veri manifesti |

## Ödevin birebir kısmı ve açık uzantılar

### Task-1

Ödev denklemleri (1)-(3) uygulanır. `-5 m/s²` talep olarak ele alınır; gerçekleşen
ivme `-min(5, μg)` olur. `d=3 m` seçilmiştir; ödev bu değeri belirtmez.
`s(t)` gidilen mesafe, `v(t)²/(2b)` kalan durma mesafesidir. Yalnız dümenleme
ve frenleme altında kalan yanal kapasiteyle dümenleme ayrı çizilir.

Dümenleme formülü sabit boylamsal hız ve sabit yanal ivme yaklaşımıdır. Frenleme
teriminin eklendiği alternatif mesafe yalnız kaçış durmadan önce tamamlanabiliyorsa
CSV'ye yazılır; tam bir araç manevra modeli olarak sunulmaz.

### Task-2

Ödev denklemleri (4)-(9) uygulanır. Her `(T, Δs)` çifti için başlangıç ve son
konum/hız/ivme koşullarını sağlayan **tek** quintic çözülür. `T` değişkendir;
`k_t` gerçekten süreyi ağırlıklandırır. `s_d=s_target(T)` yorumuyla pozisyon
cezası `Δs²` alınır. `D0=10 m`, `tau=1.5 s` açık seçimlerdir.

1. `u=t/T` ile ölçeklenmiş zamanda sabit 3×3 sistemi çöz.
2. `j=q'''/T³` ve `J=T∫j(u)²du` değerlerini analitik hesapla.
3. Ödev maliyetini bütün adaylarda hesaplayıp en küçüğünü seç.
4. **Ayrı güvenlik uzantısında** hız, ivme, jerk ve takip marjının sürekli zaman
   ekstremumlarını kontrol et; yalnız geçerli adaylardan seçim yap.

Filtre: `v>=0`, `-5<=a<=2 m/s²`, `|j|<=5 m/s³`,
`lead_s-s-(D0+tau*v)>=0`. Bu filtre evrensel sürüş güvenliği garantisi değildir.
Uygun aday yoksa `None` döner. Kaba grid `T=2:0.25:12 s`, `Δs=-20:1:10 m`;
temel ağırlıklar için yarım adımlı gridle hassasiyet de raporlanır. Elde edilen
optimum yalnız sonlu aday kümesi içindir. Filtreli çözümün grid değişimine belirgin
hassasiyeti raporda saklanmaz; sürekli optimizasyon kanıtı olarak sunulmaz.

### Task-3

Ödev denklemleri (10)-(17) ve referans parametreleri uygulanır. Denklem (13)'te
tekrarlanan `X` ifadesi `Y` olarak düzeltilir. Feedback'te en yakın yol noktasının
yönelimi kullanılır; ileri bir noktanın heading'i ile yakındaki noktanın yanal
hatası karıştırılmaz. Pure Pursuit, ileri yol ile bakış çemberinin kesişimini kullanır.

Ortak parametreler: `v=20 m/s`, `tau=0.5 s`, `dt=0.01 s`, süre `12 s`,
yol örneklemesi `0.25 m`, eğrilik komutu `±0.12 1/m`. Komut adım boyunca
sabit tutularak RK4 ile entegrasyon yapılır. RMSE, poliline dik uzaklıktan
zaman ağırlıklı hesaplanır; aynı zaman indeksindeki referans farkı kullanılmaz.
Maksimum hata, gerçekleşen yanal ivme, eğrilik doyumu ve son X ayrıca saklanır.

Bu bir lastik modeli değildir. Büyük yanal ivmeli deneyler matematiksel model
çıktısıdır; fiziksel olarak uygulanabilir sürüş diye değerlendirilmemelidir.
`dt` ve yol örneklemesi ayrı ayrı yarıya indirilerek hassasiyet ölçülür.

### Task-4

Ödevin yol eğriliği ve trafik limitleri aynen kullanılır. Denklem (27)'deki
`v_next²=v²+2aΔs` bağıntısı bütün gelecek limitlerden geriye taşınır, ardından
hızlanma kapasitesi ileriye uygulanır. Bu, tek-limit tetikleyicisinin bütün yol
bilindiğinde kullanılan **açık bir uzantısıdır**. ODE kapalı çevrim izleyicisi değildir.

Hızın karesi her aralıkta doğrusal olduğundan parça sabit ivme ve tam geçiş
zamanı elde edilir. Başlangıç hızı değiştirilemez. Gelecek limitlerle uyumsuzsa
`infeasible_initial_speed`, sıfır hızla geçilemeyecek aralık varsa
`blocked_zero_speed_interval` döner. İvme geçişlerinde jerk sınırlı değildir.

Yol geometrisi sabit eğrilikli parçaların tam yay integraliyle üretilir;
ödevin segment sonu yönelimli Euler ifadesine göre sayısal iyileştirmedir.
Bu geometriden yapay bir “araç takip RMSE” ölçümü oluşturulmaz.

## Sonuçları nasıl okumalı?

- `output/data/results.json`: raporun tek sayısal kaynağı ve çalışma ortamı.
- `task2_candidates.csv`: seçilmeyenler dahil bütün adaylar ve geçerlilikleri.
- `task2_sweep.csv`: ağırlıklar değişirken orijinal/filtreli seçimin bütün ölçümleri.
- `task3_summary.csv`: kontrolcü performansı; `task3_convergence.csv`: adım etkisi.
- `task4_summary.csv`: geçerli ve uygulanamayan senaryolar birlikte.
- Diğer CSV'ler: grafiklerin üretildiği zaman/mesafe geçmişleri; sütun adları birimlidir.
- `output/figures/`: aynı grafiklerin PNG ve SVG sürümleri.
- `output/pdf/rapor.md`: düzenlenebilir rapor; `adas_referans_raporu.pdf`: paylaşılabilir rapor.

Task-1'de `NaN` birleşik dümenlemenin mevcut fren talebi altında tanımsız olduğunu,
Task-4 yol limitlerinde `inf` düz yolda eğrilik kaynaklı sınır bulunmadığını belirtir.
JSON özetleri standart JSON'dur; desteklenmeyen NaN/Infinity içermez.

## Kapsam ve kaynaklar

Kaynak, kullanıcının sağladığı dört sayfalık hareket planlama/kontrol ödevidir.
Önceki 72 sayfalık rapor teknik sorular için incelendi. Hiçbiri bu klasöre
kopyalanmadı; bu çözüm kendi sonuçlarını üretir. Eğitim amaçlıdır; gerçek araç
uygulaması veya güvenlik doğrulaması olarak sunulmamalıdır.

Kod üst dizindeki MIT lisansına tabidir. Paketlenen DejaVu yazı tiplerinin ayrı
lisansı `assets/LICENSE_DEJAVU` dosyasında bulunur.
