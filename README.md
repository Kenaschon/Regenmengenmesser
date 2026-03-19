# 🌧️ Regenmengenmesser (ESP8266 + MQTT + Web UI)

Ein smarter Regenmengenmesser auf Basis eines **Wemos D1 mini (ESP8266)** mit **Hall-Sensor**, optionalem **BME280/BMP280** und Integration in **ioBroker über MQTT**.

---

## ✨ Features

* 🌧️ **Niederschlagsmessung**

  * Kippwaage mit S49E Linear-Hall-Sensor
  * Umrechnung in mm (konfigurierbar)
  * Regenrate (mm/h)

* 📊 **Statistiken**

  * 1 Minute / 10 Minuten / 60 Minuten
  * Tagesmenge
  * Gesamtmenge

* 🌡️ **Umweltdaten (optional)**

  * Automatische Erkennung:

    * BME280 → Temperatur, Luftfeuchte, Druck
    * BMP280 → Temperatur, Druck
  * UI passt sich automatisch an

* 📡 **MQTT Integration**

  * Perfekt für **ioBroker (mqtt.0)**
  * Retained Messages
  * Strukturierte Topics

* 🌐 **Web UI**

  * Live-Dashboard im Browser
  * Systeminfos (WLAN, Sensorstatus, etc.)

* ⏱️ **Zeit & Stabilität**

  * NTP Zeitsynchronisation
  * Failsafe-Reboot
  * OTA Updates

---

## 🔧 Hardware

* Wemos D1 mini (ESP8266)
* S49E Linear Hall Sensor   (bedruckte Seite vom Sensor muß zum Magneten zeigen)
* Kippwaage (Regenmesser)
* Optional:

  * BME280 oder BMP280 (I2C)

---

## 🔌 Verkabelung

| Komponente  | Pin Wemos D1 mini |
| ----------- | ----------------- |
| Hall Sensor | A0                |
| BME/BMP SDA | D2 (GPIO4)        |
| BME/BMP SCL | D1 (GPIO5)        |

---

## ⚙️ Konfiguration

Im Sketch anpassen:

```cpp
const char* WIFI_SSID     = "DEIN_WLAN";
const char* WIFI_PASSWORD = "PASSWORT";

const char* MQTT_HOST     = "192.168.x.x";
const uint16_t MQTT_PORT  = 1883;
const char* MQTT_USER     = "user";
const char* MQTT_PASSWORD = "pass";
```

---

## 📡 MQTT Topics

Basis:

```
regenmengenmesser/...
```

Beispiele:

* `regenmengenmesser/regen_mm_total`
* `regenmengenmesser/regen_mm_1m`
* `regenmengenmesser/regenrate_mm_h`
* `regenmengenmesser/temperature`
* `regenmengenmesser/humidity`
* `regenmengenmesser/pressure`

---

## 📏 Kalibrierung

```cpp
static const float MM_PER_TIP = 0.64f;
```

➡️ Dieser Wert hängt von deiner Kippwaage ab!
Typisch: **0.2 – 0.7 mm pro Kippvorgang**

---

## 🔍 Sensor-Erkennung

Automatisch beim Start:

* BME280 erkannt → Temp + Feuchte + Druck
* BMP280 erkannt → Temp + Druck
* Kein Sensor → nur Regenmessung

---

## 🌐 Web Interface

* Zugriff über die IP des ESP
* Anzeige von:

  * Regenwerten
  * Sensorstatus
  * WLAN-Infos
  * Systemdaten

---

## 🔄 OTA Update

Firmware kann over-the-air aktualisiert werden:

```cpp
const char* OTA_HOSTNAME  = "Regenmengenmesser";
```

---

## 🛡️ Stabilität

* Automatischer Neustart bei Fehlern
* Watchdog-ähnliches Verhalten
* MQTT/WLAN Reconnect

---

## 🧠 Funktionsweise

1. Hall-Sensor erkennt Kippbewegung
2. Entprellung + Schwellwerte
3. Zählen der „Tips“
4. Umrechnung in mm
5. Zeitbasierte Fenster (1m, 10m, 60m)
6. Berechnung Regenrate

---

## 📸 Beispiel-Anwendung

* Gartenbewässerung steuern
* Smart Home Visualisierung (ioBroker)
* Wetterstation erweitern

---

## 🚀 ToDo / Ideen

* [ ] Home Assistant Auto Discovery
* [ ] SD-Karten Logging
* [ ] Push-Benachrichtigungen bei Starkregen
* [ ] HTTPS Web UI

---

## 📜 Lizenz

Freie Nutzung für private Projekte 👍
(bei Veröffentlichung gerne Credits geben)

---

## ❤️ Credits

* Adafruit Libraries
* ESP8266 Community
* MQTT / ioBroker Ecosystem

---

## 🤝 Beitrag

Pull Requests & Ideen willkommen!

---

🌧️ Viel Spaß beim Basteln!
