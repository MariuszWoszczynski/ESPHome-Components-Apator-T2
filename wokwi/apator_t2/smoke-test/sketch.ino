#include <SPI.h>

constexpr int SCK_PIN=33, MOSI_PIN=32, MISO_PIN=19;
constexpr int CS_PIN=23, RESET_PIN=22, DIO1_PIN=21;

uint8_t readReg(uint8_t address) {
  digitalWrite(CS_PIN,LOW); SPI.transfer(address&0x7f);
  const uint8_t value=SPI.transfer(0); digitalWrite(CS_PIN,HIGH); return value;
}
void writeReg(uint8_t address,uint8_t value) {
  digitalWrite(CS_PIN,LOW); SPI.transfer(address|0x80); SPI.transfer(value);
  digitalWrite(CS_PIN,HIGH);
}
void writeFifo(const uint8_t *data,size_t length) {
  digitalWrite(CS_PIN,LOW); SPI.transfer(0x80);
  for(size_t i=0;i<length;i++) SPI.transfer(data[i]);
  digitalWrite(CS_PIN,HIGH);
}
size_t readFifo(uint8_t *data,size_t capacity,uint32_t timeout_ms) {
  const uint32_t started=millis();
  while(digitalRead(DIO1_PIN)==HIGH && millis()-started<timeout_ms) delay(1);
  if(digitalRead(DIO1_PIN)==HIGH) return 0;
  size_t length=0;
  digitalWrite(CS_PIN,LOW); SPI.transfer(0x00);
  while(digitalRead(DIO1_PIN)==LOW && length<capacity) data[length++]=SPI.transfer(0);
  digitalWrite(CS_PIN,HIGH); return length;
}
bool transmitDummy() {
  const uint8_t payload[]={0xa5,0x5a,0x96,0x69};
  writeReg(0x01,0x01); writeFifo(payload,sizeof(payload)); writeReg(0x01,0x03);
  const uint32_t started=millis();
  while(!(readReg(0x3f)&0x08) && millis()-started<200) delay(1);
  writeReg(0x01,0x01); writeReg(0x40,0x10); writeReg(0x01,0x05);
  return millis()-started<200;
}
void setup() {
  Serial.begin(115200); pinMode(CS_PIN,OUTPUT); pinMode(RESET_PIN,OUTPUT);
  pinMode(DIO1_PIN,INPUT); digitalWrite(CS_PIN,HIGH);
  digitalWrite(RESET_PIN,LOW); delay(5); digitalWrite(RESET_PIN,HIGH); delay(5);
  SPI.begin(SCK_PIN,MISO_PIN,MOSI_PIN,CS_PIN);
  if(readReg(0x42)!=0x12) { Serial.println("SX1276_SMOKE_FAIL version"); return; }
  writeReg(0x40,0x10); writeReg(0x01,0x05);
  uint8_t frame[128];
  const size_t uplink1=readFifo(frame,sizeof(frame),4000);
  if(uplink1!=18) { Serial.printf("SX1276_SMOKE_FAIL uplink=%u\n",uplink1); return; }
  if(!transmitDummy()) { Serial.println("SX1276_SMOKE_FAIL tx1"); return; }
  const size_t ack=readFifo(frame,sizeof(frame),200);
  if(ack!=38) { Serial.printf("SX1276_SMOKE_FAIL ack=%u\n",ack); return; }
  const size_t uplink2=readFifo(frame,sizeof(frame),4000);
  if(uplink2!=18) { Serial.printf("SX1276_SMOKE_FAIL uplink2=%u\n",uplink2); return; }
  if(!transmitDummy()) { Serial.println("SX1276_SMOKE_FAIL tx2"); return; }
  const size_t readback=readFifo(frame,sizeof(frame),200);
  if(readback!=56) { Serial.printf("SX1276_SMOKE_FAIL readback=%u\n",readback); return; }
  Serial.println("SX1276_SMOKE_PASS");
}
void loop() { delay(1000); }
