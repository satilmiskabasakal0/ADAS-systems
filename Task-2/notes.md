🟦 1. summary_comparison.png – Tüm Parametrelerin Karşılaştırması
📈 Pozisyon Hatası
k_s (yeşil çizgi) küçükken pozisyon hatası çok büyük → çünkü pozisyon doğruluğu önemsenmiyor.

k_j büyükken (mavi) hata yine artıyor → çünkü konfor baskın, hedefe tam ulaşmak zorunda değil.

k_t neredeyse etkilemiyor → çünkü süre ne olursa olsun pozisyona etkisi az.

📉 RMS Jerk
k_j küçükken jerk değeri yüksek → konfor önemsenmemiş.

k_j büyüdükçe jerk azalıyor → konforlu hale geliyor.

k_t ve k_s değişimi jerk'e çok az etki ediyor.

🚘 Minimum Takip Mesafesi
k_j küçükken mesafe daralıyor → çünkü pozisyona agresif gidiliyor.

k_s küçükken de mesafe azalıyor → hedefe çok yaklaşma çabası riski artırıyor.

k_t yine sabit → zamanın minimum mesafeye etkisi yok denecek kadar az.

📌 Özet: En etkili parametreler:

Konfor için k_j

Hedef doğruluğu için k_s

Zaman k_t, en az etki eden unsur

🟧 2. k_j_effect_analysis.png – Jerk Parametresinin Etkisi
📊 Grafikler:
k_j arttıkça:

Pozisyon hedefe yaklaşma azalıyor.

Hız/ivme/jerk profilleri daha düzgün → konfor artıyor.

Ancak hedefe ulaşmada sapma başlıyor.

Bar grafikte Jerk Maliyeti çok artarken, Pozisyon Maliyeti sıfıra yakın → çünkü sistem konfora odaklanıyor.

📌 Yorum: k_j çok büyükse sistem "çok yumuşak" davranıyor ama bu hedef dışı sapmaya neden olabiliyor.

🟩 3. k_s_effect_analysis.png – Hedef Doğruluğu (Pozisyon) Parametresi
📊 Grafikler:
k_s büyüdükçe:

Araç hedefe tam olarak ulaşıyor.

Ama jerk ve ivme profilleri daha agresif hale geliyor.

Bar grafikte Pozisyon Maliyeti sıfırlanıyor, Jerk artıyor.

📌 Yorum: k_s büyüdükçe sistem hedef pozisyona çok önem veriyor ama bu da sürüş konforundan çalıyor.

🟨 4. k_t_effect_analysis.png – Süre Parametresi
📊 Grafikler:
Tüm eğriler üst üste → k_t’nin pozisyon, hız, ivme, jerk üzerinde neredeyse hiç etkisi yok.

Bar grafikte sadece Zaman Maliyeti artıyor.

📌 Yorum: k_t sadece toplam maliyeti yükseltiyor ama araç davranışını değiştirmiyor.

🔚 Genel Yorum
Parametre	Ne işe yarar?	Artarsa ne olur?
k_j	Konfor	Daha yumuşak sürüş ama hedef sapması artabilir
k_s	Hedef doğruluğu	Daha isabetli pozisyon ama sarsıntı artar
k_t	Süre maliyeti	Sadece toplam maliyeti etkiler, davranışı etkilemez