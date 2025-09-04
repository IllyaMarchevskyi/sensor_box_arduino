void initEthernet() {
  Ethernet.init(ENC28J60_CS);
  if (Ethernet.begin(const_cast<byte*>(MAC_ADDR)) == 0) {
    Serial.println("DHCP failed. Using static IP.");
    Ethernet.begin(const_cast<byte*>(MAC_ADDR), STATIC_IP, DNS_IP);
  }
  server.begin();
  Serial.print("Modbus server on ");
  Serial.println(Ethernet.localIP());
}


static const char* hwName(uint8_t hs) {
  switch (hs) {
    case EthernetNoHardware: return "NoHardware";
    case EthernetW5100:      return "W5100";
    case EthernetW5200:      return "W5200";
    case EthernetW5500:      return "W5500";
    default:                 return "UnknownHW";
  }
}
static const char* linkName(uint8_t ls) {
  switch (ls) {
    case LinkON:   return "LinkON";
    case LinkOFF:  return "LinkOFF";
    default:       return "LinkUnknown";
  }
}
static const char* sockName(uint8_t s) {
  switch (s) {
    case 0x00: return "CLOSED";
    case 0x13: return "INIT";
    case 0x14: return "LISTEN";
    case 0x15: return "SYNSENT";      // немає SYN/ACK від сервера
    case 0x16: return "SYNRECV";
    case 0x17: return "ESTABLISHED";
    case 0x18: return "FIN_WAIT";
    case 0x1A: return "CLOSING";
    case 0x1B: return "TIME_WAIT";
    case 0x1C: return "CLOSE_WAIT";
    case 0x1D: return "LAST_ACK";
    default:   return "OTHER";
  }
}

void printEthDiag(EthernetClient &c) {
#if defined(ETHERNET_H)  // доступно у стандартній Ethernet для W5x00
  Serial.print(F("HW="));   Serial.print(hwName(Ethernet.hardwareStatus()));
  Serial.print(F(" LINK="));Serial.print(linkName(Ethernet.linkStatus()));
#endif
  Serial.print(F(" Sock=0x")); Serial.print(c.status(), HEX);
  Serial.print(F(" (")); Serial.print(sockName(c.status())); Serial.println(F(")"));

  Serial.print(F("Local=")); Serial.print(Ethernet.localIP());
  Serial.print(F(" GW="));   Serial.print(Ethernet.gatewayIP());
  Serial.print(F(" DNS="));  Serial.print(Ethernet.dnsServerIP());
  Serial.print(F(" MASK=")); Serial.println(Ethernet.subnetMask());
}