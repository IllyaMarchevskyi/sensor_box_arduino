#include <Arduino.h>
#include <Ethernet.h>
#include <utility/w5100.h>
#include <string.h>

// 'server' defined in modules/EthernetModbus.cpp
extern EthernetServer server;

// Time guard API from utils
bool time_guard_allow(const char* key, uint32_t interval_ms);

// API key defined in Config.h (included in main TU)
extern const char API_KEY[];

// HTTP POST helpers (declared here for use from other modules if needed)
bool httpPostSensors(const char* host, uint16_t port, const char* path);
bool httpPostSensors(const IPAddress& ip, uint16_t port, const char* path);
// Test helper: send a tiny JSON {"hello":"world"}
bool httpPostHello(const char* host, uint16_t port, const char* path);
bool httpPostHello(const IPAddress& ip, uint16_t port, const char* path);

void initEthernet() {
  Ethernet.init(ETH_CS);
  
  if (Ethernet.begin(const_cast<byte*>(MAC_ADDR)) == 0) {
    Serial.println("DHCP failed. Using static IP.");
    Ethernet.begin(const_cast<byte*>(MAC_ADDR), STATIC_IP, DNS_IP);
  }
  server.begin();
  Serial.print("Modbus server on ");
  Serial.println(Ethernet.localIP());
  Serial.println("0=NoHardware, 1=W5100, 2=W5200, 3=W5500");
  Serial.print("HW="); Serial.println((int)Ethernet.hardwareStatus());
}



// ============================ HTTP POST helpers ============================

// Builds JSON for the first 18 labels (CO..PRES) using send_arr[] values.
// Returns payload length written to out (without null terminator).
static size_t buildSensorsJson18(char* out, size_t maxLen) {
  // send_arr[] and labels[] are defined in the main TU and included before this file
  extern double send_arr[];            // from sensor_box_arduino.ino
  extern const char* const labels[];   // from Config.h
  const int values_number = 19;

  size_t pos = 0;
  auto emit = [&](const char* s) {
    size_t n = strlen(s);
    if (pos + n >= maxLen) n = (maxLen > pos) ? (maxLen - pos) : 0;
    if (n) { memcpy(out + pos, s, n); pos += n; }
  };

  emit("{");
  emit("\""); emit("id"); emit("\":"); emit("2"); emit(",");
  for (int i = 0; i < values_number; ++i) {
    // Key
    emit("\""); emit(labels[i]); emit("\":");
    // Value (as number). On AVR, use dtostrf to format float/double reliably.
    char num[24];
    dtostrf(send_arr[i], 0, 3, num); // 3 decimals; width 0 to avoid left padding
    // Trim leading spaces if any implementation pads
    char* p = num; while (*p == ' ') ++p;
    emit(p);
    if (i != values_number-1) emit(",");
  }
  emit("}");
  if (pos < maxLen) out[pos] = '\0';
  return pos;
}

// Internal: common sender for arbitrary JSON body
static bool httpPostBody(EthernetClient &client,
                         const char* hostHeader, const char* path,
                         const char* body, size_t bodyLen,
                         uint16_t timeoutMs = 2000) {
  // Request headers
  client.print("POST "); client.print(path); client.print(" HTTP/1.1\r\n");
  client.print("Host: "); client.print(hostHeader); client.print("\r\n");
  client.print("User-Agent: SensorBox/1.0\r\n");
  client.print("Content-Type: application/json\r\n");
  if (API_KEY[0] != '\0') {
    client.print("X-API-Key: "); client.print(API_KEY); client.print("\r\n");
  }
  client.print("Connection: close\r\n");
  client.print("Content-Length: "); client.print(bodyLen); client.print("\r\n\r\n");

  // Body
  client.write(reinterpret_cast<const uint8_t*>(body), bodyLen);
  client.flush();

  // Simple response wait/drain
  uint32_t t0 = millis();
  while (millis() - t0 < timeoutMs) {
    if (client.available()) break;
  }
  // Drain a chunk (optional)
  size_t got = 0;
  while (client.available() && got < 256) {
    (void)client.read();
    ++got;
  }
  client.stop();
  return (got > 0); // consider success if any response data arrived
}

// Internal: sensors sender that builds the body and calls the common routine
static bool httpPostSensorsImpl(EthernetClient &client,
                                const char* hostHeader, const char* path,
                                uint16_t timeoutMs = 2000) {
  char body[768];
  size_t bodyLen = buildSensorsJson18(body, sizeof(body) - 1);
  return httpPostBody(client, hostHeader, path, body, bodyLen, timeoutMs);
}

// Public: POST to hostname
bool httpPostSensors(const char* host, uint16_t port, const char* path) {
  if (!time_guard_allow(path, SEND_DATA_TO_SERVER)) return false;
  
  EthernetClient client;
  if (!client.connect(host, port)) {
    Serial.println(F("HTTP POST connect(host) failed"));
    return false;
  }
  return httpPostSensorsImpl(client, host, path);
}

// Public: POST to IP address
bool httpPostSensors(const IPAddress& ip, uint16_t port, const char* path) {
  if (!time_guard_allow(path, SEND_DATA_TO_SERVER)) return false;

  EthernetClient client;
  if (!client.connect(ip, port)) {
    Serial.println(F("HTTP POST connect(IP) failed"));
    return false;
  }
  // Build Host header from IP (HTTP/1.1 requires Host)
  char hostBuf[24];
  snprintf(hostBuf, sizeof(hostBuf), "%u.%u.%u.%u", ip[0], ip[1], ip[2], ip[3]);
  return httpPostSensorsImpl(client, hostBuf, path);
}

// ========================= Simple "hello: world" POST ======================

bool httpPostHello(const char* host, uint16_t port, const char* path) {
  if (!time_guard_allow(path, SEND_DATA_TO_SERVER)) return false;

  EthernetClient client;
  if (!client.connect(host, port)) {
    Serial.println(F("HTTP HELLO connect(host) failed"));
    return false;
  }
  static const char kBody[] = "{\"hello\":\"world\"}";
  return httpPostBody(client, host, path, kBody, strlen(kBody));
}

bool httpPostHello(const IPAddress& ip, uint16_t port, const char* path) {
  if (!time_guard_allow(path, SEND_DATA_TO_SERVER)) return false;

  EthernetClient client;
  if (!client.connect(ip, port)) {
    Serial.println(F("HTTP HELLO connect(IP) failed"));
    return false;
  }
  char hostBuf[24];
  snprintf(hostBuf, sizeof(hostBuf), "%u.%u.%u.%u", ip[0], ip[1], ip[2], ip[3]);
  static const char kBody[] = "{\"hello\":\"world\"}";
  return httpPostBody(client, hostBuf, path, kBody, strlen(kBody));
}
