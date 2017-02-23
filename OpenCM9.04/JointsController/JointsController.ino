/* Minimum_Source*/
#define DXL_BUS_SERIAL3 3
#define NUMBER_OF_JOINTS 12

#define WRITE_WORD 0x07
#define WRITE_BYTE 0x06
#define READ_WORD 0x05
#define READ_BYTE 0x04

#define NACK 0x55
#define ACK 0xAA

Dynamixel Dxl(DXL_BUS_SERIAL3);

void setup(){
  Serial2.begin(115200);
  pinMode(BOARD_LED_PIN, OUTPUT);  //toggleLED_Pin_Out
  Dxl.begin(3);
  
  for(int i = 1; i <= NUMBER_OF_JOINTS; i++){
    Dxl.jointMode(i);
  }
}

void loop(){
  toggleLED();
  byte buf[5];
  while(!Serial2.available());
  buf[0] = Serial2.read();
  while(!Serial2.available());
  buf[1] = Serial2.read();
  while(!Serial2.available());
  buf[2] = Serial2.read();
  
  byte command = buf[0] >> 5;
  byte id = buf[0] & 0x1F;
  byte address = buf[1];

  switch(command){
    case READ_BYTE:
    {
      byte chksum = ~(buf[0] + buf[1]);
      if(!(chksum == buf[2])){
        Serial2.flush();
        Serial2.write(NACK);
        break;
      }
      byte byte_data = Dxl.readByte(id, address);
      Serial2.write(ACK);
      Serial2.write(byte_data);
      break;
    }
    case READ_WORD:
    {
      byte chksum = ~(buf[0] + buf[1]);
      if(!(chksum == buf[2])){
        Serial2.flush();
        Serial2.write(NACK);
        break;
      }
      word word_data = Dxl.readWord(id, address);
      Serial2.write(ACK);
      Serial2.write(0xFF & (word_data >> 8));
      Serial2.write(0xFF & word_data);
      break;
    }
    case WRITE_BYTE:
    {
      while(!Serial2.available());
      buf[3] = Serial2.read();
      byte chksum = ~(buf[0] + buf[1] + buf[2]);
      if(!(chksum == buf[3])){
        Serial2.flush();
        Serial2.write(NACK);
        break;
      }
      Dxl.writeByte(id, address, buf[2]);
      Serial2.write(ACK);
      break;
    }
    case WRITE_WORD:
    {
      while(!Serial2.available());
      buf[3] = Serial2.read();
      while(!Serial2.available());
      buf[4] = Serial2.read();
      byte chksum = ~(buf[0] + buf[1] + buf[2] + buf[3]);
      if(!(chksum == buf[4])){
        Serial2.flush();
        Serial2.write(NACK);
        break;
      }
      Dxl.writeWord(id, address, (buf[2] << 8) | buf[3]);
      Serial2.write(ACK);
      break;
    }
    default:
    {
      Serial2.flush();
      Serial2.write(NACK);
      break;
    }
  }
}
