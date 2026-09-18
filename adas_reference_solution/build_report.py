"""Build the Turkish PDF and Markdown report using saved experimental evidence.

Run run_all.py and the tests first. No simulation is performed by this renderer.
"""
from pathlib import Path
import argparse
import json
from html import escape
from reportlab.lib import colors
from reportlab.lib.styles import getSampleStyleSheet, ParagraphStyle
from reportlab.lib.enums import TA_LEFT
from reportlab.lib.pagesizes import A4
from reportlab.platypus import SimpleDocTemplate, Paragraph, Spacer, Table, TableStyle, Image, PageBreak
from reportlab.pdfbase import pdfmetrics
from reportlab.pdfbase.ttfonts import TTFont

ROOT=Path(__file__).resolve().parent


def build(output):
    results=json.loads((output/'data'/'results.json').read_text())
    tests=(output/'data'/'test_results.txt').read_text()
    if not tests.rstrip().endswith('OK'):
        raise RuntimeError('Run the test suite successfully before publishing the report.')
    for font,file in [('Body','DejaVuSans.ttf'),('Bold','DejaVuSans-Bold.ttf')]:
        pdfmetrics.registerFont(TTFont(font,str(ROOT/'assets'/file)))
    pdfmetrics.registerFontFamily('Body',normal='Body',bold='Bold',italic='Body',boldItalic='Bold')
    styles=getSampleStyleSheet()
    styles.add(ParagraphStyle(name='Main',fontName='Body',fontSize=9.2,leading=14,spaceAfter=8,textColor=colors.HexColor('#263445')))
    styles.add(ParagraphStyle(name='TitleTR',fontName='Bold',fontSize=25,leading=31,spaceAfter=16,textColor=colors.HexColor('#12334C')))
    styles.add(ParagraphStyle(name='HeadTR',fontName='Bold',fontSize=17,leading=22,spaceAfter=13,textColor=colors.HexColor('#12334C')))
    styles.add(ParagraphStyle(name='SubTR',fontName='Bold',fontSize=11,leading=16,spaceBefore=7,spaceAfter=7,textColor=colors.HexColor('#087D89')))
    styles.add(ParagraphStyle(name='SmallTR',fontName='Body',fontSize=7.5,leading=10,spaceAfter=5))
    styles.add(ParagraphStyle(name='FormulaTR',fontName='Body',fontSize=9.5,leading=15,backColor=colors.HexColor('#EEF4F6'),borderPadding=8,spaceBefore=5,spaceAfter=14))
    story=[]; md=[]
    def para(text):
        story.append(Paragraph(escape(text),styles['Main'])); md.append(text+'\n')
    def heading(text):
        story.append(Paragraph(escape(text),styles['HeadTR'])); md.append('## '+text+'\n')
    def sub(text):
        story.append(Paragraph(escape(text),styles['SubTR'])); md.append('### '+text+'\n')
    def equation(text):
        story.append(Paragraph(escape(text).replace('\n','<br/>'),styles['FormulaTR'])); md.append('```text\n'+text+'\n```\n')
    def table(headers,rows,widths=None):
        data=[[Paragraph(escape(str(c)),styles['SmallTR']) for c in row] for row in [headers]+rows]
        t=Table(data,colWidths=widths,repeatRows=1,hAlign='LEFT')
        t.setStyle(TableStyle([('BACKGROUND',(0,0),(-1,0),colors.HexColor('#DEEAF0')),
                              ('ROWBACKGROUNDS',(0,1),(-1,-1),[colors.white,colors.HexColor('#F5F8FA')]),
                              ('VALIGN',(0,0),(-1,-1),'TOP'),('LEFTPADDING',(0,0),(-1,-1),7),
                              ('RIGHTPADDING',(0,0),(-1,-1),7),('TOPPADDING',(0,0),(-1,-1),7),
                              ('BOTTOMPADDING',(0,0),(-1,-1),6),('LINEBELOW',(0,0),(-1,0),.7,colors.HexColor('#8BA5B4'))]))
        story.extend([t,Spacer(1,10)])
        md.append('| '+' | '.join(headers)+' |\n| '+' | '.join(['---']*len(headers))+' |\n'+
                  '\n'.join('| '+' | '.join(map(str,row))+' |' for row in rows)+'\n')
    def fig(name,caption,width=490):
        from PIL import Image as PILImage
        path=output/'figures'/f'{name}.png'
        with PILImage.open(path) as im: h=width*im.height/im.width
        story.extend([Image(str(path),width=width,height=h),Paragraph(escape(caption),styles['SmallTR']),Spacer(1,8)])
        md.append(f'![{caption}](../figures/{name}.png)\n')
    def page(): story.append(PageBreak()); md.append('\n---\n')
    def f(value,d=3): return f'{value:.{d}f}'
    t2=results['task2']; baseline=t2['baseline']; filtered=t2['filtered']
    t3=results['task3']; t4=results['task4']

    story.append(Paragraph('Hareket Planlama<br/>ve Kontrol',styles['TitleTR']))
    md.append('# Hareket Planlama ve Kontrol\n')
    para('ADAS ödevi için uçtan uca referans çözüm • Türkçe teknik rapor • Sürüm 1.0')
    para('Amaç: Verilen dört görevi açık varsayımlar, tekrar üretilebilir deneyler ve ölçülebilir doğrulama ile çözmek. Bu çalışma, önceki teslimin düzeltilmiş kopyası yerine ayrı bir uygulamadır. Grafikler ve tablolar bu klasördeki kodun kaydedilmiş çıktılarından üretilmiştir.')
    sub('Çözümün ana kararları')
    table(['Görev','Uygulanan yöntem','Doğrulama'],[
        ['1 - Frenleme','Sürtünmeyle sınırlı ivme; gidilen ve kalan mesafenin ayrılması','Analitik durma mesafesi ve sürtünme çemberi'],
        ['2 - Yörünge','Farklı süre / son konum adayları; altı sınır koşullu quintic','Analitik jerk integrali; sürekli aralıkta ekstremumlar'],
        ['3 - Şerit değiştirme','Aynı yaw-rate gecikmesi üzerinde Feedback ve Pure Pursuit','Ortak geometrik hata; zaman/yol adımı hassasiyeti'],
        ['4 - Hız asistanı','Gelecek limitlerden geriye frenleme, ileriye hızlanma zarfı','Her yol aralığında hız ve ivme koşulları; uygulanabilirlik']
    ],[83,230,182])
    sub('Bu çalıştırmadan öne çıkan sonuçlar')
    para(f'Task-2: {t2["candidate_count"]} adayın {t2["feasible_count"]} tanesi açıklanan fiziksel filtreyi geçti. Orijinal maliyet seçiminin süresi {f(baseline["T_s"],2)} s, filtreli seçimin süresi {f(filtered["T_s"],2)} s oldu. Bu değerler yalnızca tanımlanan sonlu aday kümesi için optimumdur.')
    para(f'Task-4: Beş senaryonun {sum(r["status"]=="feasible" for r in t4["cases"])} tanesi uygulanabilir bulundu. Çok düşük ve sıfır frenleme kapasitesiyle başlayan senaryolar başarılı gibi gösterilmedi.')
    para('Analitik referans ve sınır durumlarını içeren 17 otomatik test geçti. Bu kontroller uygulamanın belirtilen modele uygunluğunu ölçer; gerçek araç güvenliği veya sertifikasyon kanıtı değildir.')
    para('Okuma sırası: model kararları ve sonuçlar → doğrulama → çalıştırma ve kaynaklar. Kod ve ham veriler raporun yanında teslim edilir.')

    page(); heading('01 / Frenleme ve dümenleme')
    para('Kaynak: ödev, bölüm 1, denklemler (1)-(3). Başlangıç hızı 25 m/s, talep edilen fren ivmesi -5 m/s², g=9,81 m/s². Verilmeyen yanal açıklık için d=3 m seçildi. Tepki süresi, araç boyutları ve direksiyon geçişi model dışında tutuldu.')
    equation('b = min(5, μg); v(t) = max(v₀ - bt, 0)\ns(t) = v₀t - bt²/2; s_kalan(t) = v(t)²/(2b)\na_y,kalan = sqrt((μg)² - b²); s_dümenleme = v sqrt(2d/a_y)')
    para('Fren ivmesi bir talep olarak yorumlandı. μ=0,15 ve 0,50 için tam -5 m/s² fiziksel olarak mümkün değildir. μ=5/g≈0,5097 eşiktir. Gidilen mesafe artarken kalan durma mesafesi azalır; bunlar aynı grafik başlığı altında karıştırılmadı.')
    table(['μ','Gerçek fren [m/s²]','Durma süresi [s]','Durma mesafesi [m]'],
          [[f(r['mu'],2),f(-r['brake_mps2']),f(r['stop_time_s']),f(r['stop_distance_m'])] for r in results['task1']],[55,155,135,150])
    fig('task1_braking','Şekil 1. Hareket ve anlık hızla hesaplanan manevra mesafeleri.',470)
    para('Sağ alt grafik, ödevin sabit boylamsal hız yaklaşımını kalan yanal kapasiteye uygular; gerçek birleşik manevra simülasyonu değildir. Frenleme sürerken x=v₀t-bt²/2 düzeltmesi ayrıca CSV özeti içinde verilir ve yalnızca yanal kaçış durmadan önce tamamlanıyorsa geçerli sayılır. Sürtünme doyumunda yanal kapasite sıfırdır; bu, freni azaltarak yapılabilecek başka bir manevranın da imkânsız olduğu anlamına gelmez.')

    page(); heading('02 / Yörünge: matematik ve adaylar')
    para('Kaynak: ödev, bölüm 2, denklemler (4)-(9). Ego başlangıcı [0 m,25 m/s,0 m/s²], öndeki araç [50 m,20 m/s,0 m/s²]. D₀=10 m ve τ=1,5 s açık tasarım seçimleridir. Öndeki araç sabit hızlıdır.')
    equation('s_hedef(T) = 50 + 20T - (10 + 1,5×20) = 10 + 20T\ns(T) = s_hedef(T) + Δs; v(T) = 20; a(T) = 0\nC = k_j J + k_t T + k_s Δs²; J = integral₀ᵀ j(t)² dt')
    para('Ödevde s_d ayrı tanımlanmadığından s_d=s_hedef(T) kabul edildi. Böylece son konum cezası Δs² olur. Süreyi sabitlemek yerine T=2...12 s (0,25 s adım) ve Δs=-20...10 m (1 m adım) tarandı. Her aday altı sınır koşulunu tam sağlar; serbest katsayıları gelişigüzel değiştirerek son hız/ivme koşulları bozulmaz.')
    sub('Sayısal koşullandırma')
    equation('u=t/T; q(u)=b₀+b₁u+b₂u²+b₃u³+b₄u⁴+b₅u⁵\nb₀=s₀; b₁=T v₀; b₂=T²a₀/2\n[1 1 1; 3 4 5; 6 12 20] [b₃ b₄ b₅]ᵀ = son koşul artıkları')
    para('Zaman u∈[0,1] aralığına taşındı; matris T ile kötü ölçeklenmez. Fiziksel türevler q′/T, q″/T² ve q‴/T³ olarak hesaplanır. Jerk karesi polinomu tam olarak integre edilir. Sayısal integral veya rastgele başlangıç gerekmiyor.')
    sub('Ödev çözümü ile güvenlik uzantısı ayrı')
    para('İlk seçim yalnızca ödev maliyetini minimize eder. İkinci seçim aynı maliyeti, tanımlanan fiziksel koşulları geçen adaylar arasında minimize eder. Ek filtre ödevin maliyet denkleminde zorunlu değildir; güvenli sonuç iddiasını ayrıca sınamak amacıyla eklenmiştir.')
    equation('v(t) ≥ 0; -5 ≤ a(t) ≤ 2 m/s²; |j(t)| ≤ 5 m/s³\ns_lead(t)-s_ego(t) ≥ D₀ + τv_ego(t)')
    para('Bunlar sürekli zaman aralığında denetlenir: her polinomun türevinin gerçek kökleri ve uç noktaları değerlendirilir. Sadece 20 veya 100 zaman örneğinde kontrol yapılmaz. Sayısal kök hesabı ve 10⁻⁸ tolerans kullanıldığı için bu, sembolik bir ispat değildir.')
    para('J birimi m²/s⁵, T birimi s, konum karesi birimi m²’dir. Toplam maliyetin boyutsuz sayılması istenirse k_j, k_t, k_s sırasıyla s⁵/m², 1/s ve 1/m² birimleri taşır. Gizli normalizasyon kullanılmadı; ağırlık değerleri yalnızca bu SI tanımı ve senaryo ile anlamlıdır.')

    page(); heading('03 / Yörünge: ağırlıkların etkisi')
    para('Temel ağırlıklar (1,1,1). Her deneyde yalnızca bir ağırlık 0,1 / 1 / 10 / 100 değerlerini alır. Kesikli çizgi ödev maliyetini, düz çizgi aynı maliyetin fiziksel filtreli seçimini gösterir. Diğer parametreler değişmez.')
    fig('task2_weights','Şekil 2. Ağırlık değişimine karşı seçilen süre ve RMS jerk.',480)
    table(['Seçim','T [s]','Δs [m]','J [m²/s⁵]','Min. marj [m]','Filtre'],
          [[label,f(c['T_s'],2),f(c['delta_m'],1),f(c['jerk_integral']),f(c['min_margin_m']),str(c['feasible'])]
           for label,c in [('Ödev',baseline),('Filtreli',filtered)]],[80,55,65,105,110,80])
    para('k_j artışı daha düşük jerk integraline öncelik verir; k_t artışı seçilen süreyi kısaltmaya baskı yapar; k_s artışı terminal ofseti sıfıra yaklaştırmaya öncelik verir. Ancak sonlu grid, aktif güvenlik koşulları ve ağırlık dengesi nedeniyle bazı seçimler değişmeyebilir. Her eğri için kesin monotoni veya sürekli optimum iddiası yapılmaz.')
    para('Ağırlıklı maliyetin büyümesi tek başına fiziksel hareketin kötüleştiğini göstermez: ağırlığın kendisi de değişmektedir. Bu nedenle toplam maliyetle birlikte süre, ofset, RMS jerk, ivme aralığı ve minimum takip marjı CSV’de ayrı saklanır. Farklı sürelerde jerk integrali ve RMS jerk aynı sıralamayı vermek zorunda değildir.')

    page(); heading('04 / Yörünge: kabul ölçütü ve grid etkisi')
    fig('task2_trajectories','Şekil 3. Temel ağırlıklarda ödev ve fiziksel filtreli seçimler.',480)
    para(f'Orijinal maliyet seçiminin minimum takip marjı {f(baseline["min_margin_m"])} m, filtreli seçimin marjı {f(filtered["min_margin_m"])} m. Negatif marj, tampon mesafesi koşulunun ihlalidir; doğrudan çarpışma ile aynı şey değildir. Çarpışma geometrisi araç boyutlarını gerektirir; bu model noktasal boylamsal durumlar kullanır.')
    table(['Grid / seçim','T [s]','Δs [m]','Maliyet'],[
        ['0,25 s / 1 m - ödev',f(baseline['T_s'],3),f(baseline['delta_m'],2),f(baseline['cost'],5)],
        ['0,125 s / 0,5 m - ödev',f(t2['refined_baseline']['T_s'],3),f(t2['refined_baseline']['delta_m'],2),f(t2['refined_baseline']['cost'],5)],
        ['0,25 s / 1 m - filtreli',f(filtered['T_s'],3),f(filtered['delta_m'],2),f(filtered['cost'],5)],
        ['0,125 s / 0,5 m - filtreli',f(t2['refined_filtered']['T_s'],3),f(t2['refined_filtered']['delta_m'],2),f(t2['refined_filtered']['cost'],5)]
    ],[220,80,90,105])
    para('İnceltilmiş grid aynı süre/ofset sınırlarını kullanır. Daha iyi maliyet bulunması, kaba gridin sürekli problem için kesin optimum olmadığını gösterir. Bu deney, aday sınırlarının yeterliliğini kanıtlamaz; uygulamada sınırlar ve başlangıç koşulları ayrıca taranmalıdır. Uygun aday yoksa seçici None döndürür; en az kötü güvensiz adayı başarılı diye sunmaz.')

    page(); heading('05 / Şerit değiştirme: ortak model')
    para('Kaynak: ödev, bölüm 3, denklemler (10)-(17). dy₁=dy₂=3,5 m; dx₁=dx₂=25 m; Xs₁=60 m; Xs₂=130 m; α=2,4. Referans Y(X), çift tanh eğrisidir; yönelim atan(dY/dX) ile elde edilir. Ödev denklem (13)’teki ikinci Ẋ ifadesi Ẏ olarak yorumlandı.')
    equation('ṙ=(-r+vκ_cmd)/τ; ψ̇=r; Ẋ=v cosψ; Ẏ=v sinψ\nFeedback: κ_cmd=k_y e_y+k_ψ e_ψ\nPure Pursuit: κ_cmd=2 y_local/L_actual²')
    para('v=20 m/s, τ=0,5 s, başlangıç [X,Y,ψ,r]=[0,0,0,0], süre 12 s. Eğrilik komutu ±0,12 1/m ile sınırlıdır. Sıfır mertebe tutulan komutla RK4 integrasyonu ve 0,01 s kontrol aralığı kullanılır. Bu eğrilik sınırı lastik tutunması garantisi değildir; gerçekleşen yanal ivme ayrıca kaydedilir.')
    para('Feedback hatası en yakın yol segmentine dik izdüşümden ölçülür; heading hatası aynı segmentin yöneliminden gelir. Pure Pursuit, yolun ileri kısmı ile bakış çemberinin kesişimini hedefler. Yol dışına çok uzak düşüldüğünde en yakın bakış mesafeli ileri yol noktası kullanılır. Yol sonunda durum sıfırlanmaz; simülasyon sonlanır.')
    para('Feedback temel kazançları k_y=0,01 ve k_ψ=2; yüksek/düşük k_y deneyleri 1 ve 10⁻⁶, yüksek/düşük k_ψ deneyleri 5 ve 0,5 kullanır. Diğer kazanç temel değerde tutulur. Pure Pursuit bakış mesafeleri 10,20,55 m’dir; bu mesafe Feedback formülünde kullanılmaz.')
    fig('task3_tracking','Şekil 4. Aynı model üzerinde kontrolcü karşılaştırması ve parametre hassasiyeti.',480)

    page(); heading('06 / Şerit değiştirme: ölçümler')
    table(['Deney','RMSE [m]','Maks. hata [m]','Maks. |a_y| [m/s²]','Doyum oranı'],
          [[r['name'],f(r['rmse_m']),f(r['max_error_m']),f(r['max_abs_lateral_accel_mps2']),f(r['saturation_fraction'])]
           for r in t3['cases']],[145,85,90,105,70])
    para('RMSE, yol poliline olan en kısa uzaklığın karesinin zamana göre ortalamasından hesaplanır. Aynı zaman indeksindeki referans ve araç konumlarını karşılaştırmak, boylamsal ilerleme farkını da yanal hata gibi sayabilir. Burada her iki kontrolcü aynı geometrik ölçütle değerlendirilir.')
    para('Pure Pursuit k_y ve k_ψ kullanmaz; bu iki değeri değiştirip aynı Pure Pursuit eğrisini tekrar çalıştırmak yerine yalnızca bakış mesafesi tarandı. Feedback için kazançlar ayrı değiştirildi. “En iyi” ifadesi yalnızca bu yol, hız, gecikme ve denenmiş değerler için kullanılabilir; genel bir kazanç aralığı önerilmez.')
    para('Bu deneyde temel Feedback daha düşük geometrik hata üretirken 20 m bakışlı Pure Pursuit daha düşük tepe yanal ivme üretir. 10 m bakışta salınım, 55 m bakışta virajları kesme eğilimi görülür. Yalnız RMSE’ye göre kontrolcü seçmek konfor ve fiziksel uygulanabilirlik farkını gizler.')
    sub('Bağımsız adım hassasiyeti')
    table(['Kontrolcü','dt [s]','Yol adımı [m]','RMSE [m]'],
          [[r['controller'],f(r['dt_s'],3),f(r['path_spacing_m'],3),f(r['rmse_m'],5)] for r in t3['convergence']],
          [180,90,110,115])
    para('Zaman adımı ve yol örnekleme adımı ayrı ayrı yarıya indirildi. Bu tablo, bildirilen RMSE’nin ayrıklaştırmaya hassasiyetini gösterir; bütün kazançlar için kararlılık ispatı değildir. Büyük yanal ivme, geometrik olarak yolu izleyen bir sonucun fiziksel araç sınırlarını aşabileceğini gösterir. Bu ödev modeli lastik kuvvetlerini veya direksiyon hız sınırını içermediğinden gerçekçi sürüş iddiası yapılmaz.')

    page(); heading('07 / Hız asistanı: limitlerden plan üretme')
    para('Kaynak: ödev, bölüm 4, denklemler (18)-(27). Yol 3000 m; geçişler 400,1000,1800,2400 m. Eğrilikler 0 / 0,005 / 0 / 0,0025 / 0 1/m; trafik hızları 90 / 50 / 70 / 50 / 90 km/sa. Bütün hesaplar SI birimleriyle yapılır.')
    equation('v_lim=min(v_trafik, sqrt(a_y,konfor/|κ|))\nκ=0 için v_yol=∞; b=|a_x,konfor|\nd_fren=(v²-v_lim²)/(2b), yalnız v>v_lim ve b>0 için')
    para('Ödevin tetik mesafesi aynı kinematik bağıntıyla bütün gelecek limitlere uygulanır. 1 m yol gridine tüm geçiş noktaları eklenir. Geriye geçişte v_i≤sqrt(v_(i+1)²+2bΔs) koşulu, ileri geçişte v_(i+1)≤sqrt(v_i²+2a_hızlanmaΔs) koşulu uygulanır. Hızlanma sınırı 1 m/s² seçildi. Sabit 200 m ileri görüş kullanılmadı.')
    para('Bu uygulama, ödevde önerilen tek-limit tetikleyicisinin bütün yol bilindiğindeki uzantısıdır; ayrı bir çevrimiçi hız kontrolcüsü veya solve_ivp çözümü değildir. Her aralıkta v² konumla doğrusal, ivme sabittir. Δt=2Δs/(v_i+v_(i+1)) ile zaman üretilir; böylece ṡ=v ve v̇=a denklemleri bu aralıklarda sağlanır.')
    fig('task4_speed','Şekil 5. Uygulanabilir hız profilleri ve ortak yol geometrisi.',480)

    page(); heading('08 / Hız asistanı: uygulanabilirlik')
    table(['Senaryo','b / a_y [m/s²]','Başlangıçta izinli hız [m/s]','Durum / süre'],
          [[r['name'],f'{r["brake_mps2"]} / {r["lateral_limit_mps2"]}',f(r['admissible_initial_speed_mps']),
            f'{f(r["travel_time_s"],2)} s' if r['status']=='feasible' else 'Uygulanamaz'] for r in t4['cases']],
          [120,105,130,140])
    para('Başlangıç hızı geriye hesaplanan hız zarfından büyükse, verilen fren kapasitesiyle gelecek limitler karşılanamaz. Başlangıç hızı gizlice düşürülmez; senaryo açıkça reddedilir. Bunun gerçek araçtaki karşılığı, konfor sınırından bağımsız acil frenleme kapasitesi ve risk yönetimi gerektirir; bu ek politika burada tasarlanmadı.')
    para('b=0,01 ve a_y=0,01 için ilk viraj limiti 1,414 m/s (5,09 km/sa), 25 m/s’den bu hıza iniş mesafesi 31.150 m’dir. İlk viraj 400 m’de olduğundan, başlangıç koşullarıyla birlikte bu senaryo uygulanamaz. b=0 ve a_y=0 durumunda pozitif hızla viraja girmek de kabul edilmez.')
    table(['Uygulanabilir senaryo','Maks. hız aşımı [m/s]','İvme aralığı [m/s²]','Maks. a_y [m/s²]'],
          [[r['name'],f(r['max_speed_excess_mps'],8),f'{f(r["min_accel_mps2"])} / {f(r["max_accel_mps2"])}',f(r['max_lateral_accel_mps2'])]
           for r in t4['cases'] if r['status']=='feasible'],[130,125,130,110])
    sub('Geometri ve sayısal sınırlar')
    para('X,Y yol geometrisi sabit eğrilikli parçaların tam çember yayı integraliyle oluşturulur. Ödevdeki segment sonu yönelimli Euler yaklaşımı yerine bu açıkça belirtilen sayısal iyileştirme seçildi. Geometri hızdan bağımsızdır; bundan “araç takip RMSE’si” üretilmez. Task-3 ayrı bir yanal kontrol deneyidir.')
    para('Bir aralıkta hızın karesi doğrusal olduğundan hız ve yanal ivme limitlerini aralık uçlarında kontrol etmek yeterlidir. İvme değişimleri anidir; jerk sınırlaması yoktur. Bu nedenle profil ivme sınırlarını karşılasa bile tamamen konforlu bir gerçek araç sürüşü olarak adlandırılmaz.')
    table(['Yol adımı [m]','Seyahat süresi [s]','Maks. hız aşımı [m/s]'],
          [[f(r['ds_m'],1),f(r['travel_time_s'],6),f(r['max_speed_excess_mps'],8)] for r in t4['convergence']],
          [145,175,175])

    page(); heading('09 / Doğrulama ve önceki soruların yanıtı')
    para('Testler yalnızca kodun kendisini tekrar etmeyen analitik referanslar ve fiziksel özdeşliklerle tasarlandı. unittest çıktısı output/data/test_results.txt dosyasında saklanır.')
    table(['Kontrol','Bağımsız referans / beklenti'],[
        ['Frenleme','s_gidilen+s_kalan=v₀²/(2b); son hız sıfır; b≤μg'],
        ['Quintic','Bütün başlangıç/son türevleri; birim yer değiştirmede 10u³-15u⁴+6u⁵'],
        ['Jerk integrali','Durgun-durgun birim yer değiştirmede J=720/T⁵'],
        ['Güvenlik aralığı','Uçları pozitif, ortası negatif polinomla örnekleme dışı ihlal'],
        ['Kontrol geometrisi','Doğruya dik izdüşüm; bakış çemberi kesişimi; yol sonunda sıfırlamama'],
        ['Hız planı','Tam çember yayı; aralık bazında ivme/hız koşulları; sıfır hız ve yetersiz fren'],
        ['Kinematik zaman','Δs=(v_i+v_(i+1))Δt/2 özdeşliği; sınırdaki hızın karşılanması']
    ],[115,380])
    sub('Sorulara doğrudan cevaplar')
    para('Düşük sürtünmede -5 m/s² sabit mi kalmalı? Hayır. Talep ve gerçekleşen ivme ayrılmalı. Sürtünme sınırı fiziksel olarak uygulanmalı ve bu yorum ödev açıklamasına yazılmalı.')
    para('Çok büyük jerk maliyeti normal mi? Her büyük değer taşma demek değildir. Önce katsayıların birimleri ve başlangıç tahmini kontrol edilir. Önceki raporda T⁴ ve T⁵ ile çarpma, aşağıda bölme diye açıklanan kodla çelişiyordu. Burada altı sınır koşullu çözüm ve normalize zaman bu sorunu ortadan kaldırır.')
    para('k_t neden etkisiz olabilir? Orijinal maliyette T sabitse k_tT bütün adaylara aynı sabiti ekler. Bu çözümde süre adaylar arasında değişir. Hız takibi cezası, süre maliyetinin yerine kullanılmadı.')
    para('Düşük RMSE güvenli sürüş kanıtı mı? Hayır. Geometrik hata ile hız/ivme/mesafe koşulları ayrı ölçülür. Bu çözümde uygulanamayan hız senaryolarına düşük bir yol RMSE’si verilerek başarı yorumu yapılmaz.')
    para('Normalizasyon sabitini değiştirmek zararsız mı? Hayır. Bir terimi 1000 yerine 10’a bölmek ağırlığını 100 kat artırır. Birimler, ölçekler ve ağırlıklar birlikte raporlanmalı; başarısız çözücü veya geçersiz aday başarı diye kabul edilmemelidir.')

    page(); heading('10 / Çalıştırma, kapsam ve kaynaklar')
    sub('Tekrar üretme')
    equation('python3 -m venv .venv\nsource .venv/bin/activate\npython -m pip install -r requirements.txt\npython reproduce.py')
    para('Komutlar bu çözüm klasöründe çalıştırılır; Windows etkinleştirme komutu README’dedir. reproduce.py test günlüğünü kaydeder, testler geçerse deneyleri ve raporu üretir, kaynak/veri SHA-256 manifestini yazar. run_all.py farklı çalışma dizininden çağrılsa da çıktıları kendi output klasörüne yazar. --output ile özel dizin seçilebilir; rapor oluşturucuda aynı dizin kullanılmalıdır.')
    para('output/data: bütün adaylar, parametre taramaları, zaman/konum serileri, yakınsama tabloları ve results.json. output/figures: PNG ve ölçeklenebilir SVG grafikler. output/pdf: bu rapor ve düzenlenebilir Markdown metni. Kaynak PDF’ler yeniden dağıtılmadı; çözüm bunları çalışma anında okumaz.')
    sub('Çalıştırma ortamı')
    para(', '.join(f'{k}: {v}' for k,v in results['environment'].items()))
    sub('Sınırlar ve bir sonraki geliştirme')
    para('Bu çalışma dört bağımsız eğitim deneyi içerir; birleşik bir otonom sürüş sistemi değildir. Algılama belirsizliği, hareketli engeller, lastik kayması, araç boyutları, sensör/aktüatör gecikmeleri (Task-3’teki yaw-rate gecikmesi dışında), yol eğimi ve trafik etkileşimleri yoktur. Task-2 sabit hızlı lider kullanır. Task-4 bütün yolu önceden bilir. Parametreler gerçek araç kalibrasyonu değildir.')
    para('Öncelikli uzantılar: farklı başlangıç koşullarıyla senaryo matrisi; frenleyen lider; yanal ivme ve direksiyon hız sınırları; jerk sınırlı hız geçişleri; Task-2 ve Task-3’ün ortak yeniden planlama döngüsünde birleştirilmesi. Bu uzantılar mevcut ödev çözümünden ayrı deneyler olarak tutulmalıdır.')
    sub('Kaynaklar ve denklem eşleştirmesi')
    para('[1] Kullanıcının sağladığı 5dcccc7010a242f585d0167af8cac80b (1).pdf: Otonom Sürüş Teknolojileri Uzmanlık Programı, Otonom Araçlarda Hareket Planlama ve Kontrol; Şerit Değiştirme ve Akıllı Hız Asistanı Fonksiyonlarının Tasarımı. Bölüm 1 → braking.py; bölüm 2 → trajectory.py; bölüm 3 → tracking.py; bölüm 4 → speed.py.')
    para('[2] Kullanıcının sağladığı ford_final_report (1).pdf, Satılmış Kabasakal, 72 sayfa. Önceki yöntemleri ve teknik soruları anlamak için incelendi; bu raporun sayısal sonuç kaynağı değildir.')
    para('Ödevdeki kaynakça, Zegelaar; Rajamani; Werling ve ark.; Falcone ve ark.; Snider; Lima ve ark.; Gámez Serna ve Ruichek çalışmalarını listeler. Bu referans çözüm söz konusu yayınların tamamını bağımsız doğruladığını iddia etmez; verilen ödev denklemlerini ve açıkça tanımlanan uzantıları uygular.')
    para('Yazım belirsizlikleri: denklem (4) zaman polinomu olarak; denklem (6) üç son durum ve süre metaverisi olarak; denklem (13) Ẏ olarak yorumlandı. Fren mesafesi formüllerinde pozitif yavaşlama büyüklüğü kullanıldı. D₀, τ, d ve sayısal çözüm adımları ödevde verilmedikleri yerde açık varsayım olarak belirtildi.')

    output_pdf=output/'pdf'; output_pdf.mkdir(parents=True,exist_ok=True)
    (output_pdf/'rapor.md').write_text('\n'.join(md),encoding='utf-8')
    def footer(canvas,doc):
        canvas.setStrokeColor(colors.HexColor('#CBD6DC')); canvas.line(48,43,A4[0]-48,43)
        canvas.setFont('Body',8); canvas.setFillColor(colors.HexColor('#607383'))
        canvas.drawString(48,29,'ADAS • Referans çözüm • Eğitim amaçlı simülasyon')
        canvas.drawRightString(A4[0]-48,29,str(doc.page))
    doc=SimpleDocTemplate(str(output_pdf/'adas_referans_raporu.pdf'),pagesize=A4,rightMargin=48,leftMargin=48,
                          topMargin=45,bottomMargin=57,title='ADAS - Hareket Planlama ve Kontrol',
                          author='Satılmış Kabasakal için hazırlanmış referans çözüm')
    doc.build(story,onFirstPage=footer,onLaterPages=footer)
    print(output_pdf/'adas_referans_raporu.pdf')


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',type=Path,default=ROOT/'output')
    build(parser.parse_args().output.resolve())
