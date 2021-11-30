# Projekt opticke zavory
---
##### [1] Seznam součástek
https://www.conrad.cz/reflexni-svetelna-zavora-datalogic-s100-pr-2-b00-pk.k1488410
https://www.conrad.cz/reflektor-datalogic-s940700075-r5-provedeni-kulate.k1488406
https://rpishop.cz/headery/1891-2x20pinovy-gpio-nastavec-stohovaci-122mm.html
https://www.amazon.com/2-23inch-OLED-Display-HAT-Interface/dp/B07Z1ZHPBZ
> **_Poznámka:_**  Displej komunikuje pomocí I2C nebo SPI. Defaultně je nastavena komunikace na SPI. Pomocí nulových odporů nachazejících se přímo na displeji je potřeba posunout spravně tyto odpory pro zajištění komunikace pomocí I2C.

Další součástky v adresáři **optic_barrier_hw** soubor **basket.pdf**

---
##### [2] Užitečné příkazy a odkazy
* Odkaz na knihovnu pro komunikace s OLED displejem
>https://www.waveshare.com/wiki/2.23inch_OLED_HAT
* Zapnutí/vypnutí služby
```
sudo systemctl enable optic_barrier.service
sudo systemctl disable optic_barrier.service
systemctl status optic_barrier.service
```
* Zjistenni jakou IP adresu má RPI
```
sudo nmap -sP 10.37.1.*
Actual IP 10.37.1.23
```
* Zkopírování image z jedné SD na druhou SD
>https://www.youtube.com/watch?v=VNqYrmYztZo
```
sudo dd if=/dev/mmcblk0 of=raspi.img
sudo dd if=raspi.img of=/dev/mmcblk0
```

* Analyzování procesů k optimalizaci doby spouštění
```
systemd-analyze critical-chain optic_barrier.service
systemd-analyze blame
```
