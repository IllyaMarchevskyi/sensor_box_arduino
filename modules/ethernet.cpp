#include <Arduino.h>
#include <Ethernet.h>
#include <utility/w5100.h>

// 'server' defined in modules/EthernetModbus.cpp
extern EthernetServer server;

// HTTP POST helpers (declared here for use from other modules if needed)
bool httpPostSensors(const char* host, uint16_t port, const char* path);
bool httpPostSensors(const IPAddress& ip, uint16_t port, const char* path);

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

