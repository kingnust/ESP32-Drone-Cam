#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include "esp_camera.h"
#include "esp_wifi.h"
#include "img_converters.h"
#include "soc/rtc_cntl_reg.h"
#include "soc/soc.h"

#include <ESP32QRCodeReader.h>
#include <../src/quirc/quirc.h>
#include "esp_heap_caps.h"

#define FLASH_LED_GPIO 4

static const char* AP_SSID = "ESP32-CAM-QR";
static const char* AP_PASS = "12345678";
static constexpr int AP_CHANNEL = 11;
static constexpr uint32_t QR_STALE_MS = 10000;
static constexpr uint32_t PREVIEW_INTERVAL_MS = 200;
static constexpr uint32_t QR_FRAME_INTERVAL_MS = 250;
static constexpr uint32_t SERIAL_HEARTBEAT_MS = 2000;
static constexpr uint32_t FRAME_PROBE_INTERVAL_MS = 10000;
static constexpr uint32_t FC_QUERY_INTERVAL_MS = 1000;
static constexpr uint32_t FC_LINK_TIMEOUT_MS = 2500;
static constexpr uint8_t MSP_API_VERSION = 1;
static constexpr uint16_t MSP2_CAMERA_QR = 0x3001;
static constexpr uint32_t QR_RETRY_INTERVAL_MS = 500;
static constexpr size_t FC_QR_MAX_PAYLOAD = 96;

ESP32QRCodeReader reader(CAMERA_MODEL_AI_THINKER);
WebServer server(80);

portMUX_TYPE qrMux = portMUX_INITIALIZER_UNLOCKED;
char lastQrPayload[512] = "";
volatile uint32_t lastQrMs = 0;
volatile uint32_t qrEventCount = 0;
volatile uint32_t scanCount = 0;
volatile uint32_t validCount = 0;
volatile uint32_t candidateCount = 0;
volatile uint32_t decodeErrorCount = 0;
volatile uint32_t mirroredDecodeCount = 0;
volatile bool flashOn = false;
volatile bool scannerReady = false;
volatile int scannerSetupResult = SETUP_OK;
volatile uint32_t previewFrameCount = 0;
volatile uint32_t previewErrorCount = 0;
volatile uint32_t fcRequestCount = 0;
volatile uint32_t fcResponseCount = 0;
volatile uint32_t fcChecksumErrorCount = 0;
volatile uint32_t fcLastResponseMs = 0;
volatile uint8_t fcApiProtocol = 0;
volatile uint8_t fcApiMajor = 0;
volatile uint8_t fcApiMinor = 0;
volatile uint32_t qrUartSendCount = 0;
volatile uint32_t qrUartAckCount = 0;
volatile uint32_t qrUartRejectCount = 0;
volatile bool qrUartPending = false;

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
          'FC UART=' + (q.fc_connected ? 'connected' : 'waiting') + ' API=' + q.fc_api + ' requests=' + q.fc_requests + ' responses=' + q.fc_responses + ' crc_errors=' + q.fc_crc_errors + ' QR sent=' + q.qr_uart_sent + ' ack=' + q.qr_uart_acks + ' rejected=' + q.qr_uart_rejected + ' pending=' + q.qr_uart_pending + ' | fresh=' + q.fresh + ' age_ms=' + q.age_ms + ' scans=' + q.scans + ' candidates=' + q.candidates + ' decode_errors=' + q.decode_errors + ' mirrored=' + q.mirrored + ' valid=' + q.valid_scans + ' flash=' + q.flash;
      } catch (e) {
        document.getElementById('meta').textContent = 'Lost connection to ESP32-CAM';
      }
    }
    function refreshPreview() {
      const image = document.getElementById('preview');
      image.onload = () => setTimeout(refreshPreview, 200);
      image.onerror = () => setTimeout(refreshPreview, 600);
      image.src = '/jpg?t=' + Date.now();
    }
    setInterval(refresh, 500);
    refresh();
    refreshPreview();
  </script>
</body>
</html>
)HTML";

void copyPayload(const uint8_t* payload, size_t payloadLen)
{
  const size_t maxLen = sizeof(lastQrPayload) - 1;
  const size_t len = payloadLen > maxLen ? maxLen : payloadLen;

  portENTER_CRITICAL(&qrMux);
  const bool changed = strlen(lastQrPayload) != len || memcmp(lastQrPayload, payload, len) != 0;
  memcpy(lastQrPayload, payload, len);
  lastQrPayload[len] = 0;
  lastQrMs = millis();
  validCount++;
  if (changed) qrEventCount++;
  portEXIT_CRITICAL(&qrMux);
}

void transposeQrCode(struct quirc_code& code)
{
  for (int y = 0; y < code.size; y++)
  {
    for (int x = 0; x < y; x++)
    {
      const int a = y * code.size + x;
      const int b = x * code.size + y;
      const bool aSet = code.cell_bitmap[a >> 3] & (1u << (a & 7));
      const bool bSet = code.cell_bitmap[b >> 3] & (1u << (b & 7));
      if (aSet != bSet)
      {
        code.cell_bitmap[a >> 3] ^= 1u << (a & 7);
        code.cell_bitmap[b >> 3] ^= 1u << (b & 7);
      }
    }
  }
}

void sendFcApiRequest()
{
  static const uint8_t request[] = {'$', 'M', '<', 0, MSP_API_VERSION, MSP_API_VERSION};
  Serial.write(request, sizeof(request));
  Serial.flush();
  fcRequestCount++;
}

uint8_t crc8DvbS2(uint8_t crc, uint8_t value)
{
  crc ^= value;
  for (uint8_t i = 0; i < 8; i++)
  {
    crc = crc & 0x80 ? static_cast<uint8_t>((crc << 1) ^ 0xD5) : static_cast<uint8_t>(crc << 1);
  }
  return crc;
}

void sendMspV2(uint16_t command, const uint8_t* payload, uint16_t length)
{
  uint8_t header[] = {'$', 'X', '<', 0, static_cast<uint8_t>(command),
                      static_cast<uint8_t>(command >> 8), static_cast<uint8_t>(length),
                      static_cast<uint8_t>(length >> 8)};
  uint8_t crc = 0;
  for (size_t i = 3; i < sizeof(header); i++) crc = crc8DvbS2(crc, header[i]);
  Serial.write(header, sizeof(header));
  for (uint16_t i = 0; i < length; i++) crc = crc8DvbS2(crc, payload[i]);
  Serial.write(payload, length);
  Serial.write(crc);
  Serial.flush();
}

void updateFcUart()
{
  enum ParseState : uint8_t {
    SYNC_DOLLAR, SYNC_PROTOCOL, V1_DIRECTION, V1_SIZE, V1_COMMAND, V1_PAYLOAD, V1_CHECKSUM,
    V2_DIRECTION, V2_HEADER, V2_PAYLOAD, V2_CHECKSUM
  };
  static ParseState state = SYNC_DOLLAR;
  static uint16_t size = 0;
  static uint16_t command = 0;
  static uint8_t checksum = 0;
  static uint16_t index = 0;
  static uint8_t header[5];
  static uint8_t payload[128];
  static uint32_t lastQueryMs = 0;
  static uint32_t lastQrSendMs = 0;
  static uint32_t stagedEvent = 0;
  static uint16_t pendingSequence = 0;
  static uint8_t pendingPayload[5 + FC_QR_MAX_PAYLOAD];
  static uint8_t pendingLength = 0;

  const uint32_t now = millis();
  if (now - lastQueryMs >= FC_QUERY_INTERVAL_MS)
  {
    lastQueryMs = now;
    sendFcApiRequest();
  }

  if (!qrUartPending && stagedEvent != qrEventCount)
  {
    char qr[FC_QR_MAX_PAYLOAD + 1] = {};
    uint32_t event;
    portENTER_CRITICAL(&qrMux);
    event = qrEventCount;
    strncpy(qr, lastQrPayload, FC_QR_MAX_PAYLOAD);
    portEXIT_CRITICAL(&qrMux);
    const uint8_t length = static_cast<uint8_t>(strlen(qr));
    if (length)
    {
      pendingSequence++;
      pendingPayload[0] = 1;
      pendingPayload[1] = 1;
      pendingPayload[2] = static_cast<uint8_t>(pendingSequence);
      pendingPayload[3] = static_cast<uint8_t>(pendingSequence >> 8);
      pendingPayload[4] = length;
      memcpy(&pendingPayload[5], qr, length);
      pendingLength = 5 + length;
      stagedEvent = event;
      qrUartPending = true;
      lastQrSendMs = 0;
    }
  }
  if (qrUartPending && now - lastQrSendMs >= QR_RETRY_INTERVAL_MS)
  {
    lastQrSendMs = now;
    sendMspV2(MSP2_CAMERA_QR, pendingPayload, pendingLength);
    qrUartSendCount++;
  }

  while (Serial.available() > 0)
  {
    const uint8_t value = static_cast<uint8_t>(Serial.read());
    switch (state)
    {
      case SYNC_DOLLAR: state = value == '$' ? SYNC_PROTOCOL : SYNC_DOLLAR; break;
      case SYNC_PROTOCOL:
        state = value == 'M' ? V1_DIRECTION : (value == 'X' ? V2_DIRECTION : SYNC_DOLLAR);
        break;
      case V1_DIRECTION: state = (value == '>' || value == '!') ? V1_SIZE : SYNC_DOLLAR; break;
      case V1_SIZE:
        size = value;
        index = 0;
        checksum = value;
        state = size <= sizeof(payload) ? V1_COMMAND : SYNC_DOLLAR;
        break;
      case V1_COMMAND:
        command = value;
        checksum ^= value;
        state = size ? V1_PAYLOAD : V1_CHECKSUM;
        break;
      case V1_PAYLOAD:
        payload[index++] = value;
        checksum ^= value;
        if (index >= size) state = V1_CHECKSUM;
        break;
      case V1_CHECKSUM:
        if (value == checksum)
        {
          if (command == MSP_API_VERSION && size >= 3)
          {
            fcApiProtocol = payload[0];
            fcApiMajor = payload[1];
            fcApiMinor = payload[2];
            fcResponseCount++;
            fcLastResponseMs = now;
          }
        }
        else
        {
          fcChecksumErrorCount++;
        }
        state = SYNC_DOLLAR;
        break;
      case V2_DIRECTION:
        index = 0;
        checksum = 0;
        state = (value == '>' || value == '!') ? V2_HEADER : SYNC_DOLLAR;
        break;
      case V2_HEADER:
        header[index++] = value;
        checksum = crc8DvbS2(checksum, value);
        if (index == sizeof(header))
        {
          command = header[1] | (static_cast<uint16_t>(header[2]) << 8);
          size = header[3] | (static_cast<uint16_t>(header[4]) << 8);
          index = 0;
          state = size <= sizeof(payload) ? (size ? V2_PAYLOAD : V2_CHECKSUM) : SYNC_DOLLAR;
        }
        break;
      case V2_PAYLOAD:
        payload[index++] = value;
        checksum = crc8DvbS2(checksum, value);
        if (index >= size) state = V2_CHECKSUM;
        break;
      case V2_CHECKSUM:
        if (value == checksum)
        {
          if (command == MSP2_CAMERA_QR && size >= 4)
          {
            const uint16_t sequence = payload[2] | (static_cast<uint16_t>(payload[3]) << 8);
            if (payload[0] == 1 && sequence == pendingSequence)
            {
              if (payload[1] == 0 || payload[1] == 1)
              {
                qrUartAckCount++;
                qrUartPending = false;
              }
              else
              {
                qrUartRejectCount++;
                qrUartPending = false;
              }
            }
          }
        }
        else fcChecksumErrorCount++;
        state = SYNC_DOLLAR;
        break;
    }
  }
}

void qrTask(void* parameter)
{
  struct quirc* decoder = quirc_new();
  uint16_t decoderWidth = 0;
  uint16_t decoderHeight = 0;
  uint8_t* rgb = nullptr;
  size_t rgbCapacity = 0;
  if (!decoder)
  {
    vTaskDelete(nullptr);
    return;
  }
  for (;;)
  {
    camera_fb_t* fb = esp_camera_fb_get();
    if (!fb)
    {
      vTaskDelay(pdMS_TO_TICKS(QR_FRAME_INTERVAL_MS));
      continue;
    }

    const size_t requiredRgb = static_cast<size_t>(fb->width) * fb->height * 3;
    if (requiredRgb > rgbCapacity)
    {
      if (rgb) heap_caps_free(rgb);
      rgb = static_cast<uint8_t*>(heap_caps_malloc(requiredRgb, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT));
      rgbCapacity = rgb ? requiredRgb : 0;
    }
    if (!rgb || !fmt2rgb888(fb->buf, fb->len, fb->format, rgb))
    {
      esp_camera_fb_return(fb);
      vTaskDelay(pdMS_TO_TICKS(QR_FRAME_INTERVAL_MS));
      continue;
    }

    if (decoderWidth != fb->width || decoderHeight != fb->height)
    {
      if (quirc_resize(decoder, fb->width, fb->height) < 0)
      {
        esp_camera_fb_return(fb);
        vTaskDelay(pdMS_TO_TICKS(QR_FRAME_INTERVAL_MS));
        continue;
      }
      decoderWidth = fb->width;
      decoderHeight = fb->height;
    }

    uint8_t* gray = quirc_begin(decoder, nullptr, nullptr);
    const size_t pixelCount = static_cast<size_t>(fb->width) * fb->height;
    uint8_t darkest = 255;
    uint8_t brightest = 0;
    for (size_t i = 0; i < pixelCount; i++)
    {
      const uint8_t r = rgb[i * 3];
      const uint8_t g = rgb[i * 3 + 1];
      const uint8_t b = rgb[i * 3 + 2];
      const uint8_t luminance = static_cast<uint8_t>((77u * r + 150u * g + 29u * b) >> 8);
      gray[i] = luminance;
      if (luminance < darkest) darkest = luminance;
      if (luminance > brightest) brightest = luminance;
    }
    const uint16_t span = brightest - darkest;
    if (span >= 32 && span < 220)
    {
      for (size_t i = 0; i < pixelCount; i++)
      {
        gray[i] = static_cast<uint8_t>((static_cast<uint16_t>(gray[i] - darkest) * 255u) / span);
      }
    }
    esp_camera_fb_return(fb);
    quirc_end(decoder);
    scanCount++;

    const int count = quirc_count(decoder);
    candidateCount += count;
    for (int i = 0; i < count; i++)
    {
      struct quirc_code code;
      struct quirc_data data;
      quirc_extract(decoder, i, &code);
      quirc_decode_error_t error = quirc_decode(&code, &data);
      if (error != QUIRC_SUCCESS)
      {
        transposeQrCode(code);
        error = quirc_decode(&code, &data);
        if (error == QUIRC_SUCCESS) mirroredDecodeCount++;
      }
      if (error == QUIRC_SUCCESS)
      {
        copyPayload(data.payload, data.payload_len);
      }
      else
      {
        decodeErrorCount++;
      }
    }
    vTaskDelay(pdMS_TO_TICKS(QR_FRAME_INTERVAL_MS));
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
  json += ",\"candidates\":";
  json += candidateCount;
  json += ",\"decode_errors\":";
  json += decodeErrorCount;
  json += ",\"mirrored\":";
  json += mirroredDecodeCount;
  json += ",\"flash\":";
  json += flashOn ? "true" : "false";
  json += ",\"ready\":";
  json += scannerReady ? "true" : "false";
  json += ",\"setup_result\":";
  json += scannerSetupResult;
  json += ",\"fc_connected\":";
  json += (fcLastResponseMs && now - fcLastResponseMs <= FC_LINK_TIMEOUT_MS) ? "true" : "false";
  json += ",\"fc_requests\":";
  json += fcRequestCount;
  json += ",\"fc_responses\":";
  json += fcResponseCount;
  json += ",\"fc_crc_errors\":";
  json += fcChecksumErrorCount;
  json += ",\"fc_api\":\"";
  json += fcApiMajor;
  json += ".";
  json += fcApiMinor;
  json += "\"";
  json += ",\"qr_uart_sent\":";
  json += qrUartSendCount;
  json += ",\"qr_uart_acks\":";
  json += qrUartAckCount;
  json += ",\"qr_uart_rejected\":";
  json += qrUartRejectCount;
  json += ",\"qr_uart_pending\":";
  json += qrUartPending ? "true" : "false";
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
    previewErrorCount++;
    server.send(503, "text/plain", "camera capture failed");
    return;
  }

  uint8_t* jpg = nullptr;
  size_t jpgLen = 0;
  const bool converted = fmt2jpg(fb->buf, fb->len, fb->width, fb->height,
                                 fb->format, 80, &jpg, &jpgLen);
  esp_camera_fb_return(fb);
  if (!converted || !jpg || !jpgLen)
  {
    previewErrorCount++;
    if (jpg) free(jpg);
    server.send(500, "text/plain", "jpeg conversion failed");
    return;
  }

  WiFiClient client = server.client();
  client.print("HTTP/1.1 200 OK\r\n");
  client.print("Content-Type: image/jpeg\r\n");
  client.print("Cache-Control: no-store, no-cache, must-revalidate\r\n");
  client.printf("Content-Length: %u\r\n\r\n", jpgLen);
  client.write(jpg, jpgLen);
  previewFrameCount++;
  free(jpg);
}

void printCameraFrameProbe()
{
  if (!scannerReady)
  {
    Serial.printf("CAM frame unavailable setup_result=%d\r\n", scannerSetupResult);
    return;
  }

  camera_fb_t* fb = esp_camera_fb_get();
  if (!fb)
  {
    Serial.println("CAM frame capture failed");
    return;
  }

  uint32_t hash = 2166136261UL;
  for (size_t i = 0; i < fb->len; i += 64)
  {
    hash = (hash ^ fb->buf[i]) * 16777619UL;
  }
  Serial.printf("CAM frame=%ux%u bytes=%u format=%d checksum=%08lX preview=%lu errors=%lu\r\n",
                fb->width, fb->height, fb->len, fb->format, hash,
                previewFrameCount, previewErrorCount);
  esp_camera_fb_return(fb);
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
    // Keep QR input uncompressed. JPEG artifacts around finder patterns made
    // small codes visible in preview but unreliable for the decoder.
    esp_camera_deinit();
    reader.cameraConfig.pixel_format = PIXFORMAT_RGB565;
    reader.cameraConfig.frame_size = FRAMESIZE_QVGA;
    reader.cameraConfig.xclk_freq_hz = 20000000;
    reader.cameraConfig.fb_count = 2;
    reader.cameraConfig.fb_location = CAMERA_FB_IN_PSRAM;
    reader.cameraConfig.grab_mode = CAMERA_GRAB_LATEST;
    const esp_err_t cameraResult = esp_camera_init(&reader.cameraConfig);
    if (cameraResult == ESP_OK)
    {
      sensor_t* sensor = esp_camera_sensor_get();
      if (sensor)
      {
        sensor->set_framesize(sensor, FRAMESIZE_QVGA);
        sensor->set_brightness(sensor, 0);
        sensor->set_contrast(sensor, 1);
        sensor->set_saturation(sensor, 0);
      }
      scannerReady = true;
      // quirc's decode structures and JPEG conversion need substantially more
      // stack than the library's grayscale-only scanner task.
      xTaskCreate(qrTask, "qrTask", 24 * 1024, nullptr, 4, nullptr);
      Serial.println("Color VGA camera and QR scanner ready");
    }
    else
    {
      scannerReady = false;
      scannerSetupResult = SETUP_CAMERA_INIT_ERROR;
      Serial.printf("Camera dual-buffer init failed: 0x%X\r\n", cameraResult);
    }
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
  updateFcUart();
  server.handleClient();
}
