#include <SPI.h>
#include <mcp2515.h>
#include <math.h>
struct can_frame canMsg;
MCP2515 mcp2515(10);
//
uint32_t pgn = 0;
uint32_t prio = 0;
uint32_t canId = 0;
float mx = -90.00, my = -90.00;

void calc_rads(float mx, float my){
  float graden;
  float rads;
  uint16_t formatted_rads;
  graden = atan2(mx, my)*(180/M_PI);
    if (graden < 0){
      graden += 360;
    }  
  Serial.print("compaskoers: ");
  Serial.println(graden);
  rads = graden*(M_PI/180);
  formatted_rads = static_cast<uint16_t>(rads*10000);
  Serial.print("koers in rads: ");
  Serial.println(formatted_rads);
  send_magneetkoers(formatted_rads);
}

uint16_t send_magneetkoers(uint16_t formatted_rads){
//uint16_t send_magneetkoers(){
  pgn = 127250;
  canId = getCanIdFromISO11783Bits(prio, pgn, 0, 0);
  canMsg.can_id  = canId | CAN_EFF_FLAG;
  
  uint16_t *pword = (uint16_t*)(canMsg.data + 1);
  *pword = formatted_rads;
  
  mcp2515.sendMessage(&canMsg);
}

uint32_t getCanIdFromISO11783Bits(uint32_t prio, uint32_t pgn, unsigned int src, unsigned int dst)
{
  uint32_t canId
      = (src & 0xff) | 0x80000000U; // src bits are the lowest ones of the CAN ID. Also set the highest bit to 1 as n2k uses only extended frames (EFF bit).

  uint32_t PF = (pgn >> 8) & 0xff;

  if (PF < 240){ // PDU 1
    canId |= (dst & 0xff) << 8;
    canId |= (pgn << 8);
  }else{ // PDU 2
    canId |= pgn << 8;
  }
  canId |= prio << 26;
  return canId;
}

void setup() {
  canMsg.can_dlc = 8;
  while (!Serial);
  Serial.begin(9600);
  mcp2515.reset();
  mcp2515.setBitrate(CAN_125KBPS);
  mcp2515.setNormalMode();
}

void loop() {
//  send_magneetkoers();
  calc_rads(mx, my);
  delay(5000);
}
