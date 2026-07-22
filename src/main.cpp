#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include "esp_camera.h"
#include "esp_wifi.h"
#include "img_converters.h"
#include "soc/rtc_cntl_reg.h"
#include "soc/soc.h"

#include <ESP32QRCodeReader.h>

#define FLASH_LED_GPIO 4

static const char* AP_SSID = "ESP32-CAM-QR";
static const char* AP_PASS = "12345678";
static constexpr int AP_CHANNEL = 1;
static constexpr uint32_t QR_STALE_MS = 10000;
static constexpr uint32_t PREVIEW_INTERVAL_MS = 700;

ESP32QRCodeReader reader(CAMERA_MODEL_AI_THINKER);
WebServer server(80);

portMUX_TYPE qrMux = portMUX_INITIALIZER_UNLOCKED;
char lastQrPayload[512] = "";
volatile uint32_t lastQrMs = 0;
volatile uint32_t scanCount = 0;
volatile uint32_t validCount = 0;
volatile bool flashOn = false;
volatile bool scannerReady = false;
volatile int scannerSetupResult = SETUP_OK;

const char INDEX_HTML[] PROGMEM = R"HTML(
<!doctype html>
<html>
<head>
  <meta name="viewport" content="width=device-width,initial-scale=1">
  <title>ESP32-CAM QR Scanner</title>
  <style>
    body { margin: 0; font-family: Arial, sans-serif; background: #101318; color: #f4f7fb; }
    main { max-width: 760px; margin: 0 auto; padding: 18px; }
    img { display: block; width: 100%; max-width: 640px; background: #000; border: 1px solid #303743; }
    .value { padding: 14px; min-height: 56px; background: #171c23; border: 1px solid #303743; overflow-wrap: anywhere; }
    .row { margin: 10px 0; color: #b9c3d0; }
    a { display: inline-block; margin: 10px 8px 10px 0; padding: 10px 12px; background: #8ee3a1; color: #101318; text-decoration: none; font-weight: 700; }
    code { color: #8ee3a1; }
  </style>
</head>
<body>
  <main>
    <h2>ESP32-CAM QR Scanner + Preview</h2>
    <div class="row">Point a QR code at the camera. Hold it steady and fill the center of the view.</div>
    <img id="preview" alt="camera preview" src="/jpg">
    <div id="payload" class="value">Waiting for QR...</div>
    <div id="meta" class="row"></div>
    <p><a href="/flash">Toggle flash</a><a href="/qr">Raw JSON</a></p>
  </main>
  <script>
    async function refresh() {
      try {
        const r = await fetch('/qr', { cache: 'no-store' });
        const q = await r.json();
        document.getElementById('payload').textContent = q.payload || 'Waiting for QR...';
        document.getElementById('meta').textContent =
          'fresh=' + q.fresh + ' age_ms=' + q.age_ms + ' scans=' + q.scans + ' valid=' + q.valid_scans + ' flash=' + q.flash;
      } catch (e) {
        document.getElementById('meta').textContent = 'Lost connection to ESP32-CAM';
      }
    }
    function refreshPreview() {
      document.getElementById('preview').src = '/jpg?t=' + Date.now();
    }
    setInterval(refresh, 500);
    setInterval(refreshPreview, 700);
    refresh();
  </script>
</body>
</html>
)HTML";

void copyPayload(const QRCodeData& data)
{
  const size_t maxLen = sizeof(lastQrPayload) - 1;
  const size_t len = data.payloadLen > (int)maxLen ? maxLen : (size_t)data.payloadLen;

  portENTER_CRITICAL(&qrMux);
  memcpy(lastQrPayload, data.payload, len);
  lastQrPayload[len] = 0;
  lastQrMs = millis();
  validCount++;
  portEXIT_CRITICAL(&qrMux);
}

void qrTask(void* parameter)
{
  QRCodeData qrCodeData;

  for (;;)
  {
    if (reader.receiveQrCode(&qrCodeData, 100))
    {
      scanCount++;
      if (qrCodeData.valid)
      {
        copyPayload(qrCodeData);
        Serial.print("QR: ");
        Serial.write(qrCodeData.payload, qrCodeData.payloadLen);
        Serial.println();
      }
      else
      {
        Serial.println("QR detected but invalid");
      }
    }
    vTaskDelay(pdMS_TO_TICKS(20));
  }
}

String jsonEscape(const char* text)
{
  String out;
  while (*text)
  {
    const char c = *text++;
    if (c == '\\' || c == '"')
    {
      out += '\\';
      out += c;
    }
    else if (c == '\n')
    {
      out += "\\n";
    }
    else if (c == '\r')
    {
      out += "\\r";
    }
    else if ((uint8_t)c >= 32)
    {
      out += c;
    }
  }
  return out;
}

void handleRoot()
{
  server.send_P(200, "text/html", INDEX_HTML);
}

void handleQr()
{
  char payload[sizeof(lastQrPayload)];
  uint32_t seenMs;
  uint32_t scans;
  uint32_t valids;
  const uint32_t now = millis();

  portENTER_CRITICAL(&qrMux);
  strncpy(payload, lastQrPayload, sizeof(payload));
  payload[sizeof(payload) - 1] = 0;
  seenMs = lastQrMs;
  scans = scanCount;
  valids = validCount;
  portEXIT_CRITICAL(&qrMux);

  const uint32_t age = seenMs ? now - seenMs : 0;
  const bool fresh = seenMs && age <= QR_STALE_MS;

  String json = "{";
  json += "\"payload\":\"";
  json += jsonEscape(payload);
  json += "\",\"fresh\":";
  json += fresh ? "true" : "false";
  json += ",\"age_ms\":";
  json += age;
  json += ",\"scans\":";
  json += scans;
  json += ",\"valid_scans\":";
  json += valids;
  json += ",\"flash\":";
  json += flashOn ? "true" : "false";
  json += ",\"ready\":";
  json += scannerReady ? "true" : "false";
  json += ",\"setup_result\":";
  json += scannerSetupResult;
  json += "}";

  server.send(200, "application/json", json);
}

void handleJpg()
{
  if (!scannerReady)
  {
    server.send(503, "text/plain", "camera not ready");
    return;
  }

  camera_fb_t* fb = esp_camera_fb_get();
  if (!fb)
  {
    server.send(503, "text/plain", "camera capture failed");
    return;
  }

  uint8_t* jpg = nullptr;
  size_t jpgLen = 0;
  const bool ok = fmt2jpg(fb->buf, fb->len, fb->width, fb->height, fb->format, 60, &jpg, &jpgLen);
  esp_camera_fb_return(fb);

  if (!ok || !jpg || !jpgLen)
  {
    if (jpg)
    {
      free(jpg);
    }
    server.send(500, "text/plain", "jpeg conversion failed");
    return;
  }

  WiFiClient client = server.client();
  client.print("HTTP/1.1 200 OK\r\n");
  client.print("Content-Type: image/jpeg\r\n");
  client.print("Cache-Control: no-store, no-cache, must-revalidate\r\n");
  client.printf("Content-Length: %u\r\n\r\n", jpgLen);
  client.write(jpg, jpgLen);
  free(jpg);
}

void handleFlash()
{
  flashOn = !flashOn;
  digitalWrite(FLASH_LED_GPIO, flashOn ? HIGH : LOW);
  server.sendHeader("Location", "/");
  server.send(303);
}

void setupWifi()
{
  WiFi.persistent(false);
  WiFi.mode(WIFI_AP);
  WiFi.setSleep(false);
  WiFi.setTxPower(WIFI_POWER_19_5dBm);
  esp_wifi_set_ps(WIFI_PS_NONE);
  esp_wifi_set_max_tx_power(78);
  WiFi.softAP(AP_SSID, AP_PASS, AP_CHANNEL, 0, 1);
}

void setup()
{
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);

  Serial.begin(115200);
  delay(500);
  Serial.println();
  Serial.println("ESP32-CAM QR scanner boot");
  Serial.printf("PSRAM: %s\r\n", psramFound() ? "yes" : "no");

  pinMode(FLASH_LED_GPIO, OUTPUT);
  digitalWrite(FLASH_LED_GPIO, LOW);

  scannerSetupResult = reader.setup();
  if (scannerSetupResult == SETUP_OK)
  {
    scannerReady = true;
    reader.beginOnCore(1);
    xTaskCreate(qrTask, "qrTask", 8 * 1024, nullptr, 4, nullptr);
    Serial.println("QR reader ready");
  }
  else
  {
    scannerReady = false;
    Serial.print("QR reader setup failed: ");
    Serial.println(scannerSetupResult);
  }

  setupWifi();
  server.on("/", HTTP_GET, handleRoot);
  server.on("/qr", HTTP_GET, handleQr);
  server.on("/jpg", HTTP_GET, handleJpg);
  server.on("/flash", HTTP_GET, handleFlash);
  server.begin();

  Serial.print("Wi-Fi: ");
  Serial.println(AP_SSID);
  Serial.print("Password: ");
  Serial.println(AP_PASS);
  Serial.print("Open: http://");
  Serial.println(WiFi.softAPIP());
  Serial.println("Preview: /jpg, QR data: /qr");
}

void loop()
{
  server.handleClient();
}
