#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <Wire.h>
#include <MPU6050_tockn.h>

MPU6050 mpu(Wire);
struct HandMotionRaw
{
  uint8_t ver;
  uint32_t seq;
  uint32_t ts_ms;
  float ax, ay, az;
  float gx, gy, gz;
  uint16_t vbat_mV;
  uint8_t flags;
  uint16_t crc16;
};

uint8_t RECEIVER_MAC[6] = {0x24, 0x6F, 0x28, 0xAA, 0xBB, 0xCC};

static uint16_t crc16(const uint8_t *data, size_t n)
{
  uint16_t c = 0xFFFF;
  for (size_t i = 0; i < n; ++i)
  {
    c ^= data[i];
    for (int b = 0; b < 8; ++b)
      c = (c & 1) ? (c >> 1) ^ 0xA001 : (c >> 1);
  }
  return c;
}

static esp_now_peer_info_t peer{};
static HandMotionRaw pkt{};
static uint32_t seq = 0;

void setup()
{
  Serial.begin(115200);
  delay(100);

  Wire.begin();
  mpu.begin();
  mpu.calcGyroOffsets(true);

  WiFi.mode(WIFI_STA);
  if (esp_now_init() != ESP_OK)
  {
    Serial.println("ESP-NOW init failed!");
    while (1)
      delay(1000);
  }

  memcpy(peer.peer_addr, RECEIVER_MAC, 6);
  peer.channel = 0;
  peer.encrypt = false;
  if (esp_now_add_peer(&peer) != ESP_OK)
  {
    Serial.println("Add peer failed!");
    while (1)
      delay(1000);
  }

  Serial.println("Sender ready.");
}

void loop()
{
  static uint32_t last = 0;
  uint32_t now = millis();
  if (now - last < 10)
  {
    delay(1);
    return;
  }
  last = now;

  mpu.update();
  pkt.ver = 1;
  pkt.seq = ++seq;
  pkt.ts_ms = now;
  pkt.ax = mpu.getAccX();
  pkt.ay = mpu.getAccY();
  pkt.az = mpu.getAccZ();
  pkt.gx = mpu.getGyroX();
  pkt.gy = mpu.getGyroY();
  pkt.gz = mpu.getGyroZ();
  pkt.vbat_mV = 0;
  pkt.flags = 0;

  pkt.crc16 = 0;
  pkt.crc16 = crc16(reinterpret_cast<const uint8_t *>(&pkt), sizeof(pkt));

  esp_err_t e = esp_now_send(RECEIVER_MAC, (const uint8_t *)&pkt, sizeof(pkt));
  if (e != ESP_OK)
  {
  }
}
