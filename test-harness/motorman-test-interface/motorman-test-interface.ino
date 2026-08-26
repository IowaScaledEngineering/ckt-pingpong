// NMRA Dcc Multifunction Motor Decoder Demo
//
// Author: Alex Shepherd 2019-03-30
// 
// This example requires these Arduino Libraries:
//
// 1) The NmraDcc Library from: http://mrrwa.org/download/
//
// These libraries can be found and installed via the Arduino IDE Library Manager
//
// This is a simple demo of how to drive and motor speed and direction using PWM and a motor H-Bridge
// It uses vStart and vHigh CV values to customise the PWM values to the motor response 
// It also uses the Headling Function to drive 2 LEDs for Directional Headlights
// Apart from that there's nothing fancy like Lighting Effects or a function matrix or Speed Tables - its just the basics...
//

#include "nmradcc2.h"
// Uncomment any of the lines below to enable debug messages for different parts of the code
//#define DEBUG_FUNCTIONS
//#define DEBUG_SPEED
//#define DEBUG_PWM
//#define DEBUG_DCC_ACK
//#define DEBUG_DCC_MSG

// This is the default DCC Address
#define DEFAULT_DECODER_ADDRESS 3

// Define the Arduino input Pin number for the DCC Signal 
#define DCC_PIN     2

typedef struct
{
  uint16_t locoAddr;
  bool isShort;
  bool isForward;
  uint8_t speed;
  bool func[29];
} LocoInfo;

LocoInfo oldInfo, currentInfo;

NmraDcc  Dcc;

// This call-back function is called whenever we receive a DCC Speed packet for our address 
void notifyDccSpeed( uint16_t Addr, DCC_ADDR_TYPE AddrType, uint8_t Speed, DCC_DIRECTION Dir, DCC_SPEED_STEPS SpeedSteps )
{
  currentInfo.locoAddr = Addr;
  currentInfo.isShort = (AddrType == DCC_ADDR_SHORT)?true:false;
  currentInfo.speed = Speed;
  currentInfo.isForward = (Dir == DCC_DIR_FWD)?true:false;
};

// This call-back function is called whenever we receive a DCC Function packet for our address 
void notifyDccFunc(uint16_t Addr, DCC_ADDR_TYPE AddrType, FN_GROUP FuncGrp, uint8_t FuncState)
{
  switch(FuncGrp)
  {
    case FN_0_4:
      currentInfo.func[0] = (FuncState & FN_BIT_00);
      currentInfo.func[1] = (FuncState & FN_BIT_01);
      currentInfo.func[2] = (FuncState & FN_BIT_02);
      currentInfo.func[3] = (FuncState & FN_BIT_03);
      currentInfo.func[4] = (FuncState & FN_BIT_04);
      break;

    case FN_5_8:
      currentInfo.func[5] = (FuncState & FN_BIT_05);
      currentInfo.func[6] = (FuncState & FN_BIT_06);
      currentInfo.func[7] = (FuncState & FN_BIT_07);
      currentInfo.func[8] = (FuncState & FN_BIT_08);
      break;

    case FN_9_12:
      currentInfo.func[9] = (FuncState & FN_BIT_09);
      currentInfo.func[10] = (FuncState & FN_BIT_10);
      currentInfo.func[11] = (FuncState & FN_BIT_11);
      currentInfo.func[12] = (FuncState & FN_BIT_12);
      break;

    case FN_13_20:
      currentInfo.func[13] = (FuncState & FN_BIT_13);
      currentInfo.func[14] = (FuncState & FN_BIT_14);
      currentInfo.func[15] = (FuncState & FN_BIT_15);
      currentInfo.func[16] = (FuncState & FN_BIT_16);
      currentInfo.func[17] = (FuncState & FN_BIT_17);
      currentInfo.func[18] = (FuncState & FN_BIT_18);
      currentInfo.func[19] = (FuncState & FN_BIT_19);
      currentInfo.func[20] = (FuncState & FN_BIT_20);
      break;

    case FN_21_28:
      currentInfo.func[21] = (FuncState & FN_BIT_21);
      currentInfo.func[22] = (FuncState & FN_BIT_22);
      currentInfo.func[23] = (FuncState & FN_BIT_23);
      currentInfo.func[24] = (FuncState & FN_BIT_24);
      currentInfo.func[25] = (FuncState & FN_BIT_25);
      currentInfo.func[26] = (FuncState & FN_BIT_26);
      currentInfo.func[27] = (FuncState & FN_BIT_27);
      currentInfo.func[28] = (FuncState & FN_BIT_28);
      break;
    case FN_LAST:
      break;
  }
}

// This function is called whenever a normal DCC Turnout Packet is received and we're in Board Addressing Mode
void notifyDccAccTurnoutBoard (uint16_t BoardAddr, uint8_t OutputPair, uint8_t Direction, uint8_t OutputPower)
{
  char buffer[256];
  snprintf(buffer, sizeof(buffer), "A:%04d %d %d", (BoardAddr-1)*4 + OutputPair + 1, Direction,OutputPower);
  Serial.println(buffer);
}

char inputBuffer[32];
uint8_t inputBufferUsed = 0;
bool cmdReady = false;

void setup()
{
  Serial.begin(115200);
  uint8_t maxWaitLoops = 255;
  while(!Serial && maxWaitLoops--)
    delay(20);

  Serial.println("Motorman Test Harness");
  memset(&currentInfo, 0, sizeof(LocoInfo));
  memset(&oldInfo, 0, sizeof(LocoInfo));
  
  // Setup which External Interrupt, the Pin it's associated with that we're using and enable the Pull-Up
  // Many Arduino Cores now support the digitalPinToInterrupt() function that makes it easier to figure out the
  // Interrupt Number for the Arduino Pin number, which reduces confusion. 
#ifdef digitalPinToInterrupt
  Dcc.pin(DCC_PIN, 0);
#else
  Dcc.pin(0, DCC_PIN, 1);
#endif
  
  Dcc.init( MAN_ID_DIY, 10, FLAGS_DCC_ACCESSORY_DECODER, 0 );

  digitalWrite(11, LOW);
  pinMode(11, INPUT);
  digitalWrite(12, LOW);
  pinMode(12, INPUT);

  digitalWrite(13, LOW);
  pinMode(13, INPUT);
  digitalWrite(14, LOW);
  pinMode(14, INPUT);


}


void loop()
{
  // You MUST call the NmraDcc.process() method frequently from the Arduino loop() function for correct library operation
  Dcc.process();

  // Handle Speed changes
  if (0 != memcmp(&oldInfo, &currentInfo, sizeof(LocoInfo)))
  {
    char buffer[256];
    snprintf(buffer, sizeof(buffer), "M:%04d %c%03d F00=[%c%c%c%c%c:%c%c%c%c%c] F10=[%c%c%c%c%c:%c%c%c%c%c] F20=[%c%c%c%c%c:%c%c%c%c]", currentInfo.locoAddr, currentInfo.isForward?'F':'R', currentInfo.speed,
      currentInfo.func[0]?'0':'-',
      currentInfo.func[1]?'1':'-',
      currentInfo.func[2]?'2':'-',
      currentInfo.func[3]?'3':'-',
      currentInfo.func[4]?'4':'-',
      currentInfo.func[5]?'5':'-',
      currentInfo.func[6]?'6':'-',
      currentInfo.func[7]?'7':'-',
      currentInfo.func[8]?'8':'-',
      currentInfo.func[9]?'9':'-',

      currentInfo.func[10]?'0':'-',
      currentInfo.func[11]?'1':'-',
      currentInfo.func[12]?'2':'-',
      currentInfo.func[13]?'3':'-',
      currentInfo.func[14]?'4':'-',
      currentInfo.func[15]?'5':'-',
      currentInfo.func[16]?'6':'-',
      currentInfo.func[17]?'7':'-',
      currentInfo.func[18]?'8':'-',
      currentInfo.func[19]?'9':'-',

      currentInfo.func[21]?'0':'-',
      currentInfo.func[21]?'1':'-',
      currentInfo.func[22]?'2':'-',
      currentInfo.func[23]?'3':'-',
      currentInfo.func[24]?'4':'-',
      currentInfo.func[25]?'5':'-',
      currentInfo.func[26]?'6':'-',
      currentInfo.func[27]?'7':'-',
      currentInfo.func[28]?'8':'-'
    );
    Serial.println(buffer);
    memcpy(&oldInfo, &currentInfo, sizeof(LocoInfo));
  }

  if (cmdReady)
  {
    switch(inputBuffer[0])
    {
      case 'R':
        digitalWrite(14, LOW);
        pinMode(14, OUTPUT);
        break;

      case 'r':
        digitalWrite(14, LOW);
        pinMode(14, INPUT);
        break;

      case 'L':
        digitalWrite(13, LOW);
        pinMode(13, OUTPUT);
        break;

      case 'l':
        digitalWrite(13, LOW);
        pinMode(13, INPUT);
        break;

      // Intermediates

      case 'A':
        digitalWrite(12, LOW);
        pinMode(12, OUTPUT);
        break;

      case 'a':
        digitalWrite(12, LOW);
        pinMode(12, INPUT);
        break;

      case 'B':
        digitalWrite(11, LOW);
        pinMode(11, OUTPUT);
        break;

      case 'b':
        digitalWrite(11, LOW);
        pinMode(11, INPUT);
        break;

      case 'x':
        digitalWrite(11, LOW);
        pinMode(11, INPUT);
        digitalWrite(12, LOW);
        pinMode(12, INPUT);

        digitalWrite(13, LOW);
        pinMode(13, INPUT);
        digitalWrite(14, LOW);
        pinMode(14, INPUT);

        break;

    }
    inputBufferUsed = 0;
    cmdReady = false;
  }

  if (!cmdReady && Serial.available() > 0)
  {
    while (Serial.available()) 
    {
      char c = Serial.read();  //gets one byte from serial buffer
      if ('\n' == c)
      {
        cmdReady = true;
        break;
      } else {
        inputBuffer[inputBufferUsed++] = c;
        if (inputBufferUsed >= sizeof(inputBuffer))
          inputBufferUsed--;
      }
    } 
  }
}
