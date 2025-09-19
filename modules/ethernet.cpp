#include <Arduino.h>
#include <Ethernet.h>
#include <utility/w5100.h>

// 'server' defined in modules/EthernetModbus.cpp
extern EthernetServer server;

void initEthernet() {
  Ethernet.init(ENC28J60_CS);
  
  if (Ethernet.begin(const_cast<byte*>(MAC_ADDR)) == 0) {
    Serial.println("DHCP failed. Using static IP.");
    Ethernet.begin(const_cast<byte*>(MAC_ADDR), STATIC_IP, DNS_IP);
  }
  W5100.setRetransmissionTime(200); 
  W5100.setRetransmissionCount(1);
  server.begin();
  Serial.print("Modbus server on ");
  Serial.println(Ethernet.localIP());
}



