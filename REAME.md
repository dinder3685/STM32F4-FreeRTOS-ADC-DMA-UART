# STM32 FreeRTOS ADC-DMA-UART Project

## Proje Özeti

Bu proje, **STM32F4** serisi mikrodenetleyici kullanılarak 
FreeRTOS ortamında **ADC (DMA destekli)**, **UART haberleşmesi** 
ve **görev senkronizasyonu (mutex)** gibi temel bileşenlerin entegrasyonunu amaçlar.
 Üç adet FreeRTOS taskı, ADC verilerini işler ve UART üzerinden iletişim kurar.

---

##  Kullanılan Bileşenler

- **Mikrodenetleyici:** STM32F407VG
- **IDE:** STM32CubeIDE
- **RTOS:** FreeRTOS (CubeMX ile entegre)
- **Programlama Dili:** C
- **Haberleşme:** UART1 (RX), UART2 (TX)
- **Analog Girişler:** ADC1 (kanal 0, 1, 2)
- **DMA:** ADC verilerini direct memory access ile memorye  aktarmak için
- **GPIO:** PA5 (LED kontrolü)

---

## Sistem Mimarisi

### Görevler (Tasks)

| Görev          | Açıklama                                                                 |
|----------------|--------------------------------------------------------------------------|
| `StartNormalTask` | UART1 üzerinden veri alır, gelen veriyi tekrar UART2 den ger döndürür.     |
| `StartHighTask`   | ADC verilerini işler, mutex kullanarak güvenli şekilde voltaj hesaplar. |
| `StartLowTask`    | ADC voltajlarını kontrol eder, belirli eşik üzerindeyse LED yakar.      |


