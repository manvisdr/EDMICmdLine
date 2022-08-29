#include <EDMICmdLine.h>

#if defined(TYPE_MK10E)
#include <Type/MK10E.h>
#elif defined(TYPE_MK6N)
#include <Type/MK6N.h>
#endif

#define NUL 0x00  // NULL termination character
#define STX 0x02  // START OF TEXT
#define ETX 0x03  // END OF TEXT
#define EOT 0x04  // END OF TRANSMISSION
#define ENQ 0x05  // ASCII 0x05 == ENQUIRY
#define ACK 0x06  // ACKNOWLEDGE
#define LF 0x0A   // LINE FEED \n
#define CR 0x0D   // CARRIAGE RETURN \r
#define DLE 0x10  // DATA LINE ESCAPE
#define XON 0x11  // XON Resume transmission
#define XOFF 0x13 // XOFF Pause transmission
#define CAN 0x18  // CANCEL

#define R_FUNC 0x52 // READ REGISTER
#define L_FUNC 0x4C // LOGIN REGISTER

// LOGIN - LEDMI,IMDEIMDE0
static uint8_t register_login[15] = {L_FUNC, 0x45, 0x44, 0x4D, 0x49, 0x2C, 0x49, 0x4D, 0x44, 0x45, 0x49, 0x4D, 0x44, 0x45, 0x00};
static uint8_t register_serialNum[] = {0xF0, 0x02};
static uint8_t register_logout[] = {0x58};

static const uint16_t registerBank[] PROGMEM =
    {
        REG_INSTAN_VOLTR,   // VOLT R
        REG_INSTAN_VOLTS,   // VOLT S
        REG_INSTAN_VOLTT,   // VOLT T
        REG_INSTAN_CURRR,   // CURRENT R
        REG_INSTAN_CURRS,   // CURRENT S
        REG_INSTAN_CURRT,   // CURRENT T
        REG_INSTAN_WATTR,   // WATT R
        REG_INSTAN_WATTS,   // WATT S
        REG_INSTAN_WATTT,   // WATT T
        REG_INSTAN_PFACT,   // pF
        REG_INSTAN_FREQU,   // FREQUENCY
        REG_ENERGY_KVARH,   // kVarh
        REG_ENERGY_KWHLBP,  // kwhLBP
        REG_ENERGY_KWHBP,   // kwhBP
        REG_ENERGY_KWHTOT}; // kwhTotal

typedef enum
{
  REG_INS_VOLTR,     // VOLT R
  REG_INS_VOLTS,     // VOLT S
  REG_INS_VOLTT,     // VOLT T
  REG_INS_CURRR,     // CURRENT R
  REG_INS_CURRS,     // CURRENT S
  REG_INS_CURRT,     // CURRENT T
  REG_INS_WATTR,     // WATT R
  REG_INS_WATTS,     // WATT S
  REG_INS_WATTT,     // WATT T
  REG_INS_PFACT,     // pF
  REG_INS_FREQU,     // FREQUENCY
  REG_ENG_KVARH,     // kVarh
  REG_ENG_KWHLBP,    // kwhLBP
  REG_ENG_KWHBP,     // kwhBP
  REG_ENG_KWHTOT,    // kwhTotal
  REG_SYS_SERNUMBER, // SERIAL NUMBER
  REG_MEAS_ALL       // ALL MEASURE REGISTER
} register_read;

enum class EdmiCMDReader::Step : uint8_t
{
  Ready,
  Started,
  Login,
  NotLogin,
  Send,
  Read,
  Logout,
  Finished
};

static uint16_t ccitt_16[256] = {
    0x0000, 0x1021, 0x2042, 0x3063, 0x4084, 0x50A5, 0x60C6, 0x70E7, 0x8108,
    0x9129, 0xA14A, 0xB16B, 0xC18C, 0xD1AD, 0xE1CE, 0xF1EF, 0x1231, 0x0210,
    0x3273, 0x2252, 0x52B5, 0x4294, 0x72F7, 0x62D6, 0x9339, 0x8318, 0xB37B,
    0xA35A, 0xD3BD, 0xC39C, 0xF3FF, 0xE3DE, 0x2462, 0x3443, 0x0420, 0x1401,
    0x64E6, 0x74C7, 0x44A4, 0x5485, 0xA56A, 0xB54B, 0x8528, 0x9509, 0xE5EE,
    0xF5CF, 0xC5AC, 0xD58D, 0x3653, 0x2672, 0x1611, 0x0630, 0x76D7, 0x66F6,
    0x5695, 0x46B4, 0xB75B, 0xA77A, 0x9719, 0x8738, 0xF7DF, 0xE7FE, 0xD79D,
    0xC7BC, 0x48C4, 0x58E5, 0x6886, 0x78A7, 0x0840, 0x1861, 0x2802, 0x3823,
    0xC9CC, 0xD9ED, 0xE98E, 0xF9AF, 0x8948, 0x9969, 0xA90A, 0xB92B, 0x5AF5,
    0x4AD4, 0x7AB7, 0x6A96, 0x1A71, 0x0A50, 0x3A33, 0x2A12, 0xDBFD, 0xCBDC,
    0xFBBF, 0xEB9E, 0x9B79, 0x8B58, 0xBB3B, 0xAB1A, 0x6CA6, 0x7C87, 0x4CE4,
    0x5CC5, 0x2C22, 0x3C03, 0x0C60, 0x1C41, 0xEDAE, 0xFD8F, 0xCDEC, 0xDDCD,
    0xAD2A, 0xBD0B, 0x8D68, 0x9D49, 0x7E97, 0x6EB6, 0x5ED5, 0x4EF4, 0x3E13,
    0x2E32, 0x1E51, 0x0E70, 0xFF9F, 0xEFBE, 0xDFDD, 0xCFFC, 0xBF1B, 0xAF3A,
    0x9F59, 0x8F78, 0x9188, 0x81A9, 0xB1CA, 0xA1EB, 0xD10C, 0xC12D, 0xF14E,
    0xE16F, 0x1080, 0x00A1, 0x30C2, 0x20E3, 0x5004, 0x4025, 0x7046, 0x6067,
    0x83B9, 0x9398, 0xA3FB, 0xB3DA, 0xC33D, 0xD31C, 0xE37F, 0xF35E, 0x02B1,
    0x1290, 0x22F3, 0x32D2, 0x4235, 0x5214, 0x6277, 0x7256, 0xB5EA, 0xA5CB,
    0x95A8, 0x8589, 0xF56E, 0xE54F, 0xD52C, 0xC50D, 0x34E2, 0x24C3, 0x14A0,
    0x0481, 0x7466, 0x6447, 0x5424, 0x4405, 0xA7DB, 0xB7FA, 0x8799, 0x97B8,
    0xE75F, 0xF77E, 0xC71D, 0xD73C, 0x26D3, 0x36F2, 0x0691, 0x16B0, 0x6657,
    0x7676, 0x4615, 0x5634, 0xD94C, 0xC96D, 0xF90E, 0xE92F, 0x99C8, 0x89E9,
    0xB98A, 0xA9AB, 0x5844, 0x4865, 0x7806, 0x6827, 0x18C0, 0x08E1, 0x3882,
    0x28A3, 0xCB7D, 0xDB5C, 0xEB3F, 0xFB1E, 0x8BF9, 0x9BD8, 0xABBB, 0xBB9A,
    0x4A75, 0x5A54, 0x6A37, 0x7A16, 0x0AF1, 0x1AD0, 0x2AB3, 0x3A92, 0xFD2E,
    0xED0F, 0xDD6C, 0xCD4D, 0xBDAA, 0xAD8B, 0x9DE8, 0x8DC9, 0x7C26, 0x6C07,
    0x5C64, 0x4C45, 0x3CA2, 0x2C83, 0x1CE0, 0x0CC1, 0xEF1F, 0xFF3E, 0xCF5D,
    0xDF7C, 0xAF9B, 0xBFBA, 0x8FD9, 0x9FF8, 0x6E17, 0x7E36, 0x4E55, 0x5E74,
    0x2E93, 0x3EB2, 0x0ED1, 0x1EF0};

static unsigned short
CalculateCharacterCRC16(unsigned short crc, uint8_t c)
{
  return ((crc << 8) ^ ccitt_16[((crc >> 8) ^ c)]);
}

float ConvertB32ToFloat(uint32_t b32)
{
  float
      result;
  int32_t
      shift;
  uint16_t
      bias;

  if (b32 == 0)
    return 0.0;
  // pull significand
  result = (b32 & 0x7fffff); // mask significand
  result /= (0x800000);      // convert back to float
  result += 1.0f;            // add one back
  // deal with the exponent
  bias = 0x7f;
  shift = ((b32 >> 23) & 0xff) - bias;
  while (shift > 0)
  {
    result *= 2.0;
    shift--;
  }
  while (shift < 0)
  {
    result /= 2.0;
    shift++;
  }
  // sign
  result *= (b32 >> 31) & 1 ? -1.0 : 1.0;
  return result;
}

uint32_t bytearrayto32byte(uint8_t *data)
{
  return ((long)data[0]) << 24 |
         ((long)data[1]) << 16 |
         ((long)data[2]) << 8 |
         ((long)data[3]);
}

static uint16_t
CRC16buf(const uint8_t *data, uint16_t len)
{
  uint8_t nTemp = STX; // CRC table index
  uint16_t crc = 0;    // Default value
  while (len--)
  {
    crc = (crc << 8) ^ ccitt_16[((crc >> 8) ^ nTemp)];
    nTemp = *data++;
  }
  crc = (crc << 8) ^ ccitt_16[((crc >> 8) ^ nTemp)];
  return crc;
}

bool checkCRC(uint8_t *buf, uint16_t len)
{
  if (len <= 3)
    return false;

  uint16_t crc = CRC16buf(buf, len - 2); // Compute CRC of data
  return ((uint16_t)buf[len - 2] << 8 | (uint16_t)buf[len - 1]) == crc;
}

void EdmiCMDReader::begin(unsigned long baud)
{
  serial_.begin(baud, SERIAL_8N1, rx_, tx_);
}

void EdmiCMDReader::TX_raw(uint8_t ch)
{
  serial_.write(ch);
  serial_.flush();
}

void EdmiCMDReader::TX_byte(uint8_t d)
{
  switch (d)
  {
  case STX:
  case ETX:
  case DLE:
  case XON:
  case XOFF:
    EdmiCMDReader::TX_raw(DLE);
    EdmiCMDReader::TX_raw(d | 0x40);
    break;
  default:
    EdmiCMDReader::TX_raw(d);
  }
}

void EdmiCMDReader::TX_cmd(uint8_t *cmd, unsigned short len)
{
  unsigned short i;
  unsigned short crc;
  /*
     Add the STX and start the CRC calc.
  */
  EdmiCMDReader::TX_raw(STX);
  crc = CalculateCharacterCRC16(0, STX);
  /*
     Send the data, computing CRC as we go.
  */
  for (i = 0; i < len; i++)
  {
    EdmiCMDReader::TX_byte(*cmd);
    crc = CalculateCharacterCRC16(crc, *cmd++);
  }
  /*
     Add the CRC
  */
  EdmiCMDReader::TX_byte((uint8_t)(crc >> 8));
  EdmiCMDReader::TX_byte((uint8_t)crc);
  /*
     Add the ETX
  */
  EdmiCMDReader::TX_raw(ETX);
}

void EdmiCMDReader::send_cmdR(const byte *reg)
{
  uint8_t registers[3] = {R_FUNC, reg[0], reg[1]};
  TX_cmd(registers, sizeof(registers));
}

bool EdmiCMDReader::RX_char(unsigned int timeout, byte *inChar)
{
  unsigned long currentMillis = millis();
  unsigned long previousMillis = currentMillis;

  while (currentMillis - previousMillis < timeout)
  {
    if (serial_.available())
    {
      (*inChar) = (byte)serial_.read();
      return true;
    }
    currentMillis = millis();
  } // while
  return false;
}

uint8_t EdmiCMDReader::RX_message(uint8_t *message, int maxMessageSize, unsigned int timeout)
{
  byte inChar;
  int index = 0;
  bool completeMsg = false;
  bool goodCheksum = false;
  bool dlechar = false;
  static unsigned short crc;

  unsigned long currentMillis = millis();
  unsigned long previousMillis = currentMillis;

  while (currentMillis - previousMillis < timeout)
  {
    if (serial_.available())
    {
      inChar = (byte)serial_.read();

      if (inChar == STX) // start of message?
      {
        while (RX_char(INTERCHAR_TIMEOUT, &inChar))
        {
          if (inChar != ETX)
          {
            if (inChar == DLE)
              dlechar = true;
            else
            {
              if (dlechar)
                inChar &= 0xBF;
              dlechar = false;
              message[index] = inChar;
              index++;
              if (index == maxMessageSize) // buffer full
                break;
            }
          }
          else // got ETX, next character is the checksum
          {
            if (index > 6)
            {
              if (checkCRC(message, index))
              {
                message[index] = '\0'; // good checksum, null terminate message
                goodCheksum = true;
                completeMsg = true;
              }
            }
            else
            {
              message[index] = '\0';
              completeMsg = true;
            }
          } // next character is the checksum
        }   // while read until ETX or timeout
      }     // inChar == STX
    }       // serial_.available()
    if (index > 6)
    {
      if (completeMsg and goodCheksum)
        break;
    }
    else
    {
      if (completeMsg)
        break;
    }
    currentMillis = millis();
  } // while
  return index - 3;
}

void EdmiCMDReader::change_status(Status to)
{
  if (to == Status::ProtocolError || to == Status::TimeoutError || to == Status::Disconnect)
    ++errors_;
  else if (to == Status::ChecksumError)
    ++checksum_errors_;
  else if (to == Status::Finish)
    ++successes_;

  status_ = to;
}

void EdmiCMDReader::keepAlive()
{
  Serial.println("KEEPALIVE");
  if (status_ == Status::Busy or status_ == Status::Ready)
    return;

  uint8_t
      charAck[CHAR_RX_ACK];

  TX_raw(STX);
  TX_raw(ETX);
  RX_message(charAck, CHAR_RX_ACK, RX_TIMEOUT);
  if (charAck[0] == ACK)
    status_ = Status::Connect;
  else
    status_ = Status::Disconnect;
}

void EdmiCMDReader::step_start()
{
  if (status_ == Status::Disconnect or status_ == Status::Busy)
    return;

  status_ = Status::Busy;
  step_ = Step::Started;
}

void EdmiCMDReader::step_login()
{
  if (status_ != Status::Busy)
    return;

  uint8_t
      charAck[CHAR_RX_ACK];
  size_t
      len;

  TX_cmd(register_login, sizeof(register_login));
  len = RX_message(charAck, CHAR_RX_ACK, RX_TIMEOUT);
  Serial.print("step_login() - len - ");
  Serial.println(len);
  if (charAck[0] == ACK)
  {
    Serial.println("step_login() -> ACK");
    if (step_ == Step::Started)
      step_ = Step::Read;
    else
      status_ = Status::LoggedIn;
  }
  else if (charAck[0] == CAN)
  {
    regError_ = (ErrorCode)charAck[1];
  }
}

void EdmiCMDReader::step_logout()
{
  if (status_ != Status::Busy)
    return;

  uint8_t
      charAck[CHAR_RX_ACK];
  size_t
      len;

  TX_cmd(register_logout, sizeof(register_logout));
  len = RX_message(charAck, CHAR_RX_ACK, RX_TIMEOUT);
  Serial.print("step_logout() - len - ");
  Serial.println(len);
  if (charAck[0] == ACK)
  {
    step_ = Step::Finished;
    status_ = Status::Finish;
    return change_status(Status::Finish);
  }
}

bool EdmiCMDReader::check_RXReg(uint8_t *buff, uint8_t typeReg, uint8_t compare)
{
  return (((uint16_t)buff[0] << 8) == ((uint16_t)typeReg << 8 |
                                       (uint16_t)compare));
}

String EdmiCMDReader::read_Serialnumber(/*char *output, int len*/)
{
  uint8_t
      charData[CHAR_RX_DATA];
  size_t
      len;
  String output;

  status_ = Status::Busy;

  step_login();
  if (status_ == Status::LoggedIn)
  {
    send_cmdR(register_serialNum);
    len = RX_message(charData, CHAR_RX_DATA, RX_TIMEOUT);
    Serial.println(len);
    if (charData[0] == CAN)
    {
      regError_ = (ErrorCode)charData[1];
      output = "";
    }
    else if (charData[0] == R_FUNC and charData[1] == register_serialNum[0] and charData[2] == register_serialNum[1])
    {
      for (size_t i = 0; i < len - 3; i++)
      {
        output += (char)charData[i + 3];
        // Serial.printf("%2X", charData[i]);
        // Serial.printf("%s", output);
      }
      Serial.println(output);
      _currentValues.serialNumber = output;
      Serial.println(_currentValues.serialNumber);
    }
    else
      output = "";
  }
  status_ = Status::Connect;
  return output;
}

void EdmiCMDReader::read_looping()
{

  if (status_ != Status::Busy or status_ == Status::Disconnect)
    return;
  Serial.println("looping");
  switch (step_)
  {
  case Step::Ready:
    break;
  case Step::Started:
    step_login();
    break;
  case Step::Read:
    // step_read();
    read_default();
    break;
  case Step::Logout:
    step_logout();
    break;
  default:
    break;
  }
}

bool EdmiCMDReader::read_default()
{
  unsigned short
      len;
  len = sizeof(registerBank) / sizeof(registerBank[0]);
  uint32_t
      allData[len];
  uint8_t
      charData[CHAR_RX_DATA];
  uint8_t
      charBuffer[CHAR_DATA_FLOAT];
  size_t
      i = 0;
  Serial.print("read_default() - len - ");
  Serial.println(len);

  if (status_ == Status::Busy)
  {
    while (i < len)
    {
      uint16_t r = pgm_read_word(&registerBank[i++]);

      uint8_t singlereg[2];
      singlereg[0] = (r >> 8) & 0xFF;
      singlereg[1] = (r)&0xFF;
      uint8_t buff[] = {0x52, singlereg[0], singlereg[1]};
      TX_cmd(buff, sizeof(buff));
      unsigned short datalen = RX_message(charData, CHAR_RX_DATA, RX_TIMEOUT);

      if (charData[0] == CAN)
      {
        regError_ = (ErrorCode)charData[1];
        return NAN;
      }
      else
      {

        allData[i - 1] = ((long)charData[3]) << 24 |
                         ((long)charData[4]) << 16 |
                         ((long)charData[5]) << 8 |
                         ((long)charData[6]);
        // // }
        Serial.printf("register ke - %d\n", i);
        // Serial.printf("%.4f\n", ConvertB32ToFloat(allData[i - 1]));
      }
      _currentValues.voltR = ConvertB32ToFloat(allData[0]) / 1000.0;
      _currentValues.voltS = ConvertB32ToFloat(allData[1]) / 1000.0;
      _currentValues.voltT = ConvertB32ToFloat(allData[2]) / 1000.0;
      _currentValues.currentR = ConvertB32ToFloat(allData[3]) / 1000.0;
      _currentValues.currentS = ConvertB32ToFloat(allData[4]) / 1000.0;
      _currentValues.currentT = ConvertB32ToFloat(allData[5]) / 1000.0;
      _currentValues.wattR = ConvertB32ToFloat(allData[6]) / 1000.0;
      _currentValues.wattS = ConvertB32ToFloat(allData[7]) / 1000.0;
      _currentValues.wattT = ConvertB32ToFloat(allData[8]) / 1000.0;
      _currentValues.pf = ConvertB32ToFloat(allData[9]);
      _currentValues.frequency = ConvertB32ToFloat(allData[10]);
      _currentValues.kVarh = ConvertB32ToFloat(allData[11]);
      _currentValues.kwhLWBP = ConvertB32ToFloat(allData[12]);
      _currentValues.kwhWBP = ConvertB32ToFloat(allData[13]);
      _currentValues.kwhTotal = ConvertB32ToFloat(allData[14]);
    }
    Serial.printf("%.4f\n", _currentValues.voltR);
    Serial.printf("%.4f\n", _currentValues.voltS);
    Serial.printf("%.4f\n", _currentValues.voltT);
    Serial.printf("%.4f\n", _currentValues.currentR);
    Serial.printf("%.4f\n", _currentValues.currentS);
    Serial.printf("%.4f\n", _currentValues.currentT);
    Serial.printf("%.4f\n", _currentValues.wattR);
    Serial.printf("%.4f\n", _currentValues.wattS);
    Serial.printf("%.4f\n", _currentValues.wattT);
    Serial.printf("%.4f\n", _currentValues.pf);
    Serial.printf("%.4f\n", _currentValues.frequency);
    Serial.printf("%.4f\n", _currentValues.kVarh);
    Serial.printf("%.4f\n", _currentValues.kwhLWBP);
    Serial.printf("%.4f\n", _currentValues.kwhWBP);
    Serial.printf("%.4f\n", _currentValues.kwhTotal);
  }

  step_ = Step::Logout;
  return true;
}

String EdmiCMDReader::serialNumber()
{
  if (strcmp(read_Serialnumber().c_str(), "") == 0)
  {
    Serial.println("KOSONG");
    return String("");
  }

  return _currentValues.serialNumber;
}

float EdmiCMDReader::voltR() { return _currentValues.voltR; }
float EdmiCMDReader::voltS() { return _currentValues.voltS; }
float EdmiCMDReader::voltT() { return _currentValues.voltT; }
float EdmiCMDReader::currentR() { return _currentValues.currentR; }
float EdmiCMDReader::currentS() { return _currentValues.currentS; }
float EdmiCMDReader::currentT() { return _currentValues.currentT; }
float EdmiCMDReader::wattR() { return _currentValues.wattR; }
float EdmiCMDReader::wattS() { return _currentValues.wattS; }
float EdmiCMDReader::wattT() { return _currentValues.wattT; }
float EdmiCMDReader::pf() { return _currentValues.pf; }
float EdmiCMDReader::frequency() { return _currentValues.frequency; }
float EdmiCMDReader::kVarh() { return _currentValues.kVarh; }
float EdmiCMDReader::kwhWBP() { return _currentValues.kwhWBP; }
float EdmiCMDReader::kwhLWBP() { return _currentValues.kwhLWBP; }
float EdmiCMDReader::kwhTotal() { return _currentValues.kwhTotal; }

float EdmiCMDReader::read_voltR()
{
  uint32_t
      oneData;
  uint8_t
      charData[CHAR_RX_DATA];

  if (status_ == Status::Busy)
  {
    uint16_t r = pgm_read_word(&registerBank[0]);
    uint8_t buff[] = {0x52, (uint8_t)(r >> 8), (uint8_t)(r)};
    TX_cmd(buff, sizeof(buff));
    unsigned short datalen = RX_message(charData, CHAR_RX_DATA, RX_TIMEOUT);
    if (charData[0] == CAN)
    {
      regError_ = (ErrorCode)charData[1];
      return NAN;
    }
    else
    {
      oneData = ((long)charData[3]) << 24 |
                ((long)charData[4]) << 16 |
                ((long)charData[5]) << 8 |
                ((long)charData[6]);
      Serial.printf("%.4f\n", ConvertB32ToFloat(oneData));
      _currentValues.voltR = ConvertB32ToFloat(oneData) / 1000.0;
      Serial.printf("%.4f\n", _currentValues.voltR);
      step_ = Step::Logout;
      return _currentValues.voltR;
    }
  }
  else
    return NAN;
}

float EdmiCMDReader::read_voltS()
{
  uint32_t
      oneData;
  uint8_t
      charData[CHAR_RX_DATA];

  if (status_ == Status::Busy)
  {
    uint16_t r = pgm_read_word(&registerBank[1]);
    uint8_t buff[] = {0x52, (uint8_t)(r >> 8), (uint8_t)(r)};
    TX_cmd(buff, sizeof(buff));
    unsigned short datalen = RX_message(charData, CHAR_RX_DATA, RX_TIMEOUT);
    if (charData[0] == CAN)
    {
      regError_ = (ErrorCode)charData[1];
      return NAN;
    }
    else
    {
      oneData = ((long)charData[3]) << 24 |
                ((long)charData[4]) << 16 |
                ((long)charData[5]) << 8 |
                ((long)charData[6]);
      Serial.printf("%.4f\n", ConvertB32ToFloat(oneData));
      _currentValues.voltS = ConvertB32ToFloat(oneData) / 1000.0;
      Serial.printf("%.4f\n", _currentValues.voltS);
      step_ = Step::Logout;
      return _currentValues.voltS;
    }
  }
  else
    return NAN;
}

float EdmiCMDReader::read_voltT()
{
  uint32_t
      oneData;
  uint8_t
      charData[CHAR_RX_DATA];

  if (status_ == Status::Busy)
  {
    uint16_t r = pgm_read_word(&registerBank[2]);
    uint8_t buff[] = {0x52, (uint8_t)(r >> 8), (uint8_t)(r)};
    TX_cmd(buff, sizeof(buff));
    unsigned short datalen = RX_message(charData, CHAR_RX_DATA, RX_TIMEOUT);
    if (charData[0] == CAN)
    {
      regError_ = (ErrorCode)charData[1];
      return NAN;
    }
    else
    {
      oneData = ((long)charData[3]) << 24 |
                ((long)charData[4]) << 16 |
                ((long)charData[5]) << 8 |
                ((long)charData[6]);
      Serial.printf("%.4f\n", ConvertB32ToFloat(oneData));
      _currentValues.voltT = ConvertB32ToFloat(oneData) / 1000.0;
      Serial.printf("%.4f\n", _currentValues.voltT);
      step_ = Step::Logout;
      return _currentValues.voltT;
    }
  }
  else
    return NAN;
}

float EdmiCMDReader::read_currentR()
{
  uint32_t
      oneData;
  uint8_t
      charData[CHAR_RX_DATA];

  if (status_ == Status::Busy)
  {
    uint16_t r = pgm_read_word(&registerBank[3]);
    uint8_t buff[] = {0x52, (uint8_t)(r >> 8), (uint8_t)(r)};
    TX_cmd(buff, sizeof(buff));
    unsigned short datalen = RX_message(charData, CHAR_RX_DATA, RX_TIMEOUT);
    if (charData[0] == CAN)
    {
      regError_ = (ErrorCode)charData[1];
      return NAN;
    }
    else
    {
      oneData = ((long)charData[3]) << 24 |
                ((long)charData[4]) << 16 |
                ((long)charData[5]) << 8 |
                ((long)charData[6]);
      Serial.printf("%.4f\n", ConvertB32ToFloat(oneData));
      _currentValues.currentR = ConvertB32ToFloat(oneData) / 1000.0;
      Serial.printf("%.4f\n", _currentValues.currentR);
      step_ = Step::Logout;
      return _currentValues.currentR;
    }
  }
  else
    return NAN;
}

float EdmiCMDReader::read_currentS()
{
  uint32_t
      oneData;
  uint8_t
      charData[CHAR_RX_DATA];

  if (status_ == Status::Busy)
  {
    uint16_t r = pgm_read_word(&registerBank[4]);
    uint8_t buff[] = {0x52, (uint8_t)(r >> 8), (uint8_t)(r)};
    TX_cmd(buff, sizeof(buff));
    unsigned short datalen = RX_message(charData, CHAR_RX_DATA, RX_TIMEOUT);
    if (charData[0] == CAN)
    {
      regError_ = (ErrorCode)charData[1];
      return NAN;
    }
    else
    {
      oneData = ((long)charData[3]) << 24 |
                ((long)charData[4]) << 16 |
                ((long)charData[5]) << 8 |
                ((long)charData[6]);
      Serial.printf("%.4f\n", ConvertB32ToFloat(oneData));
      _currentValues.currentS = ConvertB32ToFloat(oneData) / 1000.0;
      Serial.printf("%.4f\n", _currentValues.currentS);
      step_ = Step::Logout;
      return _currentValues.currentS;
    }
  }
  else
    return NAN;
}

float EdmiCMDReader::read_currentT()
{
  uint32_t
      oneData;
  uint8_t
      charData[CHAR_RX_DATA];

  if (status_ == Status::Busy)
  {
    uint16_t r = pgm_read_word(&registerBank[5]);
    uint8_t buff[] = {0x52, (uint8_t)(r >> 8), (uint8_t)(r)};
    TX_cmd(buff, sizeof(buff));
    unsigned short datalen = RX_message(charData, CHAR_RX_DATA, RX_TIMEOUT);
    if (charData[0] == CAN)
    {
      regError_ = (ErrorCode)charData[1];
      return NAN;
    }
    else
    {
      oneData = ((long)charData[3]) << 24 |
                ((long)charData[4]) << 16 |
                ((long)charData[5]) << 8 |
                ((long)charData[6]);
      Serial.printf("%.4f\n", ConvertB32ToFloat(oneData));
      _currentValues.currentT = ConvertB32ToFloat(oneData) / 1000.0;
      Serial.printf("%.4f\n", _currentValues.currentT);
      step_ = Step::Logout;
      return _currentValues.currentT;
    }
  }
  else
    return NAN;
}

float EdmiCMDReader::read_wattR()
{
  uint32_t
      oneData;
  uint8_t
      charData[CHAR_RX_DATA];

  if (status_ == Status::Busy)
  {
    uint16_t r = pgm_read_word(&registerBank[6]);
    uint8_t buff[] = {0x52, (uint8_t)(r >> 8), (uint8_t)(r)};
    TX_cmd(buff, sizeof(buff));
    unsigned short datalen = RX_message(charData, CHAR_RX_DATA, RX_TIMEOUT);
    if (charData[0] == CAN)
    {
      regError_ = (ErrorCode)charData[1];
      return NAN;
    }
    else
    {
      oneData = ((long)charData[3]) << 24 |
                ((long)charData[4]) << 16 |
                ((long)charData[5]) << 8 |
                ((long)charData[6]);
      Serial.printf("%.4f\n", ConvertB32ToFloat(oneData));
      _currentValues.wattR = ConvertB32ToFloat(oneData) / 1000.0;
      Serial.printf("%.4f\n", _currentValues.wattR);
      step_ = Step::Logout;
      return _currentValues.wattR;
    }
  }
  else
    return NAN;
}

float EdmiCMDReader::read_wattS()
{
  uint32_t
      oneData;
  uint8_t
      charData[CHAR_RX_DATA];

  if (status_ == Status::Busy)
  {
    uint16_t r = pgm_read_word(&registerBank[7]);
    uint8_t buff[] = {0x52, (uint8_t)(r >> 8), (uint8_t)(r)};
    TX_cmd(buff, sizeof(buff));
    unsigned short datalen = RX_message(charData, CHAR_RX_DATA, RX_TIMEOUT);
    if (charData[0] == CAN)
    {
      regError_ = (ErrorCode)charData[1];
      return NAN;
    }
    else
    {
      oneData = ((long)charData[3]) << 24 |
                ((long)charData[4]) << 16 |
                ((long)charData[5]) << 8 |
                ((long)charData[6]);
      Serial.printf("%.4f\n", ConvertB32ToFloat(oneData));
      _currentValues.wattR = ConvertB32ToFloat(oneData) / 1000.0;
      Serial.printf("%.4f\n", _currentValues.wattR);
      step_ = Step::Logout;
      return _currentValues.wattR;
    }
  }
  else
    return NAN;
}

float EdmiCMDReader::read_wattT()
{
  uint32_t
      oneData;
  uint8_t
      charData[CHAR_RX_DATA];

  if (status_ == Status::Busy)
  {
    uint16_t r = pgm_read_word(&registerBank[8]);
    uint8_t buff[] = {0x52, (uint8_t)(r >> 8), (uint8_t)(r)};
    TX_cmd(buff, sizeof(buff));
    unsigned short datalen = RX_message(charData, CHAR_RX_DATA, RX_TIMEOUT);
    if (charData[0] == CAN)
    {
      regError_ = (ErrorCode)charData[1];
      return NAN;
    }
    else
    {
      oneData = ((long)charData[3]) << 24 |
                ((long)charData[4]) << 16 |
                ((long)charData[5]) << 8 |
                ((long)charData[6]);
      Serial.printf("%.4f\n", ConvertB32ToFloat(oneData));
      _currentValues.wattT = ConvertB32ToFloat(oneData) / 1000.0;
      Serial.printf("%.4f\n", _currentValues.wattT);
      step_ = Step::Logout;
      return _currentValues.wattT;
    }
  }
  else
    return NAN;
}

float EdmiCMDReader::read_pf()
{
  uint32_t
      oneData;
  uint8_t
      charData[CHAR_RX_DATA];

  if (status_ == Status::Busy)
  {
    uint16_t r = pgm_read_word(&registerBank[9]);
    uint8_t buff[] = {0x52, (uint8_t)(r >> 8), (uint8_t)(r)};
    TX_cmd(buff, sizeof(buff));
    unsigned short datalen = RX_message(charData, CHAR_RX_DATA, RX_TIMEOUT);
    if (charData[0] == CAN)
    {
      regError_ = (ErrorCode)charData[1];
      return NAN;
    }
    else
    {
      oneData = ((long)charData[3]) << 24 |
                ((long)charData[4]) << 16 |
                ((long)charData[5]) << 8 |
                ((long)charData[6]);
      Serial.printf("%.4f\n", ConvertB32ToFloat(oneData));
      _currentValues.pf = ConvertB32ToFloat(oneData);
      Serial.printf("%.4f\n", _currentValues.pf);
      step_ = Step::Logout;
      return _currentValues.pf;
    }
  }
  else
    return NAN;
}

float EdmiCMDReader::read_frequency()
{
  uint32_t
      oneData;
  uint8_t
      charData[CHAR_RX_DATA];

  if (status_ == Status::Busy)
  {
    uint16_t r = pgm_read_word(&registerBank[10]);
    uint8_t buff[] = {0x52, (uint8_t)(r >> 8), (uint8_t)(r)};
    TX_cmd(buff, sizeof(buff));
    unsigned short datalen = RX_message(charData, CHAR_RX_DATA, RX_TIMEOUT);
    if (charData[0] == CAN)
    {
      regError_ = (ErrorCode)charData[1];
      return NAN;
    }
    else
    {
      oneData = ((long)charData[3]) << 24 |
                ((long)charData[4]) << 16 |
                ((long)charData[5]) << 8 |
                ((long)charData[6]);
      Serial.printf("%.4f\n", ConvertB32ToFloat(oneData));
      _currentValues.frequency = ConvertB32ToFloat(oneData);
      Serial.printf("%.4f\n", _currentValues.frequency);
      step_ = Step::Logout;
      return _currentValues.frequency;
    }
  }
  else
    return NAN;
}

float EdmiCMDReader::read_kVarh()
{
  uint32_t
      oneData;
  uint8_t
      charData[CHAR_RX_DATA];

  if (status_ == Status::Busy)
  {
    uint16_t r = pgm_read_word(&registerBank[11]);
    uint8_t buff[] = {0x52, (uint8_t)(r >> 8), (uint8_t)(r)};
    TX_cmd(buff, sizeof(buff));
    unsigned short datalen = RX_message(charData, CHAR_RX_DATA, RX_TIMEOUT);
    if (charData[0] == CAN)
    {
      regError_ = (ErrorCode)charData[1];
      return NAN;
    }
    else
    {
      oneData = ((long)charData[3]) << 24 |
                ((long)charData[4]) << 16 |
                ((long)charData[5]) << 8 |
                ((long)charData[6]);
      Serial.printf("%.4f\n", ConvertB32ToFloat(oneData));
      _currentValues.kVarh = ConvertB32ToFloat(oneData);
      Serial.printf("%.4f\n", _currentValues.kVarh);
      step_ = Step::Logout;
      return _currentValues.kVarh;
    }
  }
  else
    return NAN;
}

float EdmiCMDReader::read_kwhWBP()
{
  uint32_t
      oneData;
  uint8_t
      charData[CHAR_RX_DATA];

  if (status_ == Status::Busy)
  {
    uint16_t r = pgm_read_word(&registerBank[12]);
    uint8_t buff[] = {0x52, (uint8_t)(r >> 8), (uint8_t)(r)};
    TX_cmd(buff, sizeof(buff));
    unsigned short datalen = RX_message(charData, CHAR_RX_DATA, RX_TIMEOUT);
    if (charData[0] == CAN)
    {
      regError_ = (ErrorCode)charData[1];
      return NAN;
    }
    else
    {
      oneData = ((long)charData[3]) << 24 |
                ((long)charData[4]) << 16 |
                ((long)charData[5]) << 8 |
                ((long)charData[6]);
      Serial.printf("%.4f\n", ConvertB32ToFloat(oneData));
      _currentValues.kwhWBP = ConvertB32ToFloat(oneData);
      Serial.printf("%.4f\n", _currentValues.kwhWBP);
      step_ = Step::Logout;
      return _currentValues.kwhWBP;
    }
  }
  else
    return NAN;
}

float EdmiCMDReader::read_kwhLWBP()
{
  uint32_t
      oneData;
  uint8_t
      charData[CHAR_RX_DATA];

  if (status_ == Status::Busy)
  {
    uint16_t r = pgm_read_word(&registerBank[13]);
    uint8_t buff[] = {0x52, (uint8_t)(r >> 8), (uint8_t)(r)};
    TX_cmd(buff, sizeof(buff));
    unsigned short datalen = RX_message(charData, CHAR_RX_DATA, RX_TIMEOUT);
    if (charData[0] == CAN)
    {
      regError_ = (ErrorCode)charData[1];
      return NAN;
    }
    else
    {
      oneData = ((long)charData[3]) << 24 |
                ((long)charData[4]) << 16 |
                ((long)charData[5]) << 8 |
                ((long)charData[6]);
      Serial.printf("%.4f\n", ConvertB32ToFloat(oneData));
      _currentValues.kwhLWBP = ConvertB32ToFloat(oneData);
      Serial.printf("%.4f\n", _currentValues.kwhLWBP);
      step_ = Step::Logout;
      return _currentValues.kwhLWBP;
    }
  }
  else
    return NAN;
}

float EdmiCMDReader::read_kwhTotal()
{
  uint32_t
      oneData;
  uint8_t
      charData[CHAR_RX_DATA];

  if (status_ == Status::Busy)
  {
    uint16_t r = pgm_read_word(&registerBank[14]);
    uint8_t buff[] = {0x52, (uint8_t)(r >> 8), (uint8_t)(r)};
    TX_cmd(buff, sizeof(buff));
    unsigned short datalen = RX_message(charData, CHAR_RX_DATA, RX_TIMEOUT);
    if (charData[0] == CAN)
    {
      regError_ = (ErrorCode)charData[1];
      return NAN;
    }
    else
    {
      oneData = ((long)charData[3]) << 24 |
                ((long)charData[4]) << 16 |
                ((long)charData[5]) << 8 |
                ((long)charData[6]);
      Serial.printf("%.4f\n", ConvertB32ToFloat(oneData));
      _currentValues.kwhTotal = ConvertB32ToFloat(oneData);
      Serial.printf("%.4f\n", _currentValues.kwhTotal);
      step_ = Step::Logout;
      return _currentValues.kwhTotal;
    }
  }
  else
    return NAN;
}
