# teknofest_A2-Orta--itifa-roket_flight_computer
Raspberry Pi Pico tabanlı A2 orta irtifa roket uçuş bilgisayarı; sensör verilerini okuyarak telemetri gönderir, SD karta kayıt alır ve paraşüt açılma durumlarını yönetir.     ( BMP280 + ikinci barometre + BNO055 + GPS + LoRa + SD kart)
fünye tetiklemesini mosfet ile daha yüksek voltta yapın 

## Önemli Notlar
-lora e32 nin uart rate adress channel ayarlarını ft232 i ile bilgisyara baglayıp yapın kodda frekans ayarı yapılmadı power ayarları aynı olsun ayrıca 

- Deniz seviyesi basıncı, uçuş yapılacak alana göre güncellenmelidir.
- LoRa E32 modülünün UART rate, adres, kanal ve güç ayarları FT232 veya benzeri bir USB-UART dönüştürücü ile önceden yapılmalıdır.
- Kod içerisinde LoRa frekans veya kanal ayarı yapılmamaktadır.
- Paraşüt/fünye tetikleme devresi doğrudan Pico pininden beslenmemeli, MOSFET veya uygun bir sürücü devre ile yapılmalıdır.
- Sistem gerçek uçuş öncesinde yerde detaylı şekilde test edilmelidir.


## Proje Özeti

Bu yazılım aşağıdaki görevleri yerine getirir:

- BMP280 sensörü ile basınç, sıcaklık ve irtifa ölçümü yapar.
- BNO055 sensörü ile ivme ve jiroskop verilerini okur.
- L86 GPS modülü ile konum, hız, uydu sayısı ve GPS doğruluk verilerini alır.
- LoRa E32 modülü üzerinden yer istasyonuna telemetri verisi gönderir.
- SD karta uçuş verilerini CSV formatında kaydeder.
- Roketin uçuş durumunu takip eder.
- Apogee noktasında birincil paraşüt/ayrılma çıkışını tetikler.
- Belirlenen irtifada ikincil/ana paraşüt çıkışını tetikler.
- İniş sonrası buzzer ve LED ile roketin bulunmasını kolaylaştırır.

## Kullanılan Donanımlar

- Raspberry Pi Pico
- BNO055 IMU sensörü
- BMP280 basınç sensörü
- Quectel L86 GPS modülü
- LoRa E32-433T modülü
- SD kart modülü
- Buzzer
- LED
- MOSFET kontrollü paraşüt tetikleme devresi


Raspberry Pi Pico

BNO055 + BMP280 I2C:
SDA  -> GP4
SCL  -> GP5
VCC  -> 3.3V
GND  -> GND

LoRa E32-433T:
E32 RX -> Pico GP0 TX
E32 TX -> Pico GP1 RX
GND    -> GND
VCC    -> uygun besleme

Quectel L86 GPS:
GPS TX -> Pico GP9 RX
GPS RX -> Pico GP8 TX  // çoğu zaman şart değil ama bağlanabilir
VCC    -> modülüne göre 3.3V veya 5V
GND    -> GND

SD Card SPI:
CS   -> GP17
SCK  -> GP18
MOSI -> GP19
MISO -> GP16
VCC  -> 3.3V
GND  -> GND

Çıkışlar:
Birincil ayrılma sinyali -> GP10
İkincil ayrılma sinyali  -> GP11
Buzzer                   -> GP12
LED / flaşör             -> GP13
