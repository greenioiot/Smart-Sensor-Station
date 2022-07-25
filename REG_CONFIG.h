// Device Address (Modbus)
#define ID_SOIL     1
#define ID_RAIN     10
#define ID_PM25     255

// Outdoor Air Quality Register Address (Modbus)
#define PM25      0x0007
#define PM10      0x0008
#define TEMP      0x0009
#define HUMIDITY  0x000A

// Rain Gauges Register Address (Modbus)Type RD-RGP-05
#define ACCUMULATE  0x0000
#define COUNT       0x0001

// 7 in 1 Register Address (Modbus)
#define _MOISTURE 0x0000
#define _TEMP     0x0001
#define _EC       0x0002
#define _PH       0x0003
#define _Nit      0x0004
#define _Pho      0x0005
#define _Pot      0x0006

uint16_t const Address_PM2510[4] = {
  PM25,
  PM10,
  TEMP,
  HUMIDITY
};

uint16_t const Address_RDRGP05[2] = {
  ACCUMULATE,
  COUNT
};

uint16_t const Address_7IN1[7] = {
  _MOISTURE,
  _TEMP,
  _EC,
  _PH,
  _Nit,
  _Pho,
  _Pot
};
