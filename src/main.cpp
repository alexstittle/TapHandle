#include <Arduino.h>
#include <Adafruit_NeoPixel.h>
#include <Wire.h>
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>

#define LED_PIN       8

#define PIXEL_COUNT   60

#define FULL_KEG_LEVEL      30

/*
   ACCELEROMETER REGISTERS
*/
#define REG_CARD_ID      0x0F     ///<The chip id
#define REG_CTRL_REG1    0x20     ///<Control register 1
#define REG_CTRL_REG4    0x23     ///<Control register 2
#define REG_CTRL_REG2    0x21     ///<Control register 3
#define REG_CTRL_REG3    0x22     ///<Control register 4
#define REG_CTRL_REG5    0x24     ///<Control register 5
#define REG_CTRL_REG6    0x25     ///<Control register 6
#define REG_CTRL_REG7    0x3F     ///<Control register 7
#define REG_STATUS_REG   0x27     ///<Status register
#define REG_OUT_X_L      0x28     ///<The low order of the X-axis acceleration register
#define REG_OUT_X_H      0x29     ///<The high point of the X-axis acceleration register
#define REG_OUT_Y_L      0x2A     ///<The low order of the Y-axis acceleration register
#define REG_OUT_Y_H      0x2B     ///<The high point of the Y-axis acceleration register
#define REG_OUT_Z_L      0x2C     ///<The low order of the Z-axis acceleration register
#define REG_OUT_Z_H      0x2D     ///<The high point of the Z-axis acceleration register
#define REG_WAKE_UP_DUR  0x35     ///<Wakeup and sleep duration configuration register
#define REG_FREE_FALL    0x36     ///<Free fall event register
#define REG_STATUS_DUP   0x37     ///<Interrupt event status register
#define REG_WAKE_UP_SRC  0x38     ///<Wakeup source register
#define REG_TAP_SRC      0x39     ///<Tap source register
#define REG_SIXD_SRC     0x3A     ///<6D source register
#define REG_ALL_INT_SRC  0x3B     ///<Reading this register, all related interrupt function flags routed to the INT pads are reset simultaneously

#define REG_TAP_THS_X    0x30     ///<4D configuration enable and TAP threshold configuration .
#define REG_TAP_THS_Y    0x31     ///<Threshold for tap recognition @ FS = ±2 g on Y direction
#define REG_TAP_THS_Z    0x32     ///<Threshold for tap recognition @ FS = ±2 g on Z direction
#define REG_INT_DUR      0x33     ///<Interrupt duration register
#define REG_WAKE_UP_THS  0x34     ///<Wakeup threshold register


#define ONE_G           0x0FFF

#define   I2C_SDA_PIN         6
#define   I2C_SCL_PIN         7
#define   I2C_SPEED           400000

#define   I2C_DEV_ADDR        0x19

/**************
   FUNCTION PROTOTYPES
*/
void readRegister (  uint8_t address, uint8_t *regData, uint8_t size );
void writeRegister ( uint8_t address, uint8_t *regData, uint8_t size );
void initAccelerometer ( void );

bool pourColdOne ( void );
void showFill ( void );
void kegDelivery( void );

/*
    NeoPixel
*/
Adafruit_NeoPixel pixels(PIXEL_COUNT + 1, LED_PIN, NEO_GRB + NEO_KHZ800);



/*
    BLUETOOTH
*/
#define   DEVICE_NAME   "Tap"

BLECharacteristic   *pBeerLevelCharacteristic;
BLECharacteristic   *pRGBCharacteristic;

BLEDescriptor        BeerLevelDescriptor(BLEUUID((uint16_t)0x2901));
BLEDescriptor        RGBDescriptor(BLEUUID((uint16_t)0x2901));
BLEDescriptor        BeerLevelCCCD(BLEUUID((uint16_t)0x2902));
BLEDescriptor        RGBCCCD(BLEUUID((uint16_t)0x2902));

BLEServer *pServer;

uint32_t    ledColour = pixels.Color(0, 10, 0);

bool  deviceConnected = false;

// See the following for generating UUIDs:
// https://www.uuidgenerator.net/

#define SERVICE_UUID                      "01150f0d-dab8-499a-b387-c163a0e78a4c"
#define CHARACTERISTIC_BEERLEVEL_UUID     "94297fba-1f8e-4d16-acb0-d4eb7f4eadf4"
#define CHARACTERISTIC_RGB_UUID           "30000096-fc55-42c3-acf7-3356a42d302b"

class MyServerCallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer* pServer, esp_ble_gatts_cb_param_t *param) {
      pServer->updateConnParams(param->connect.remote_bda, 0x06, 0x06, 0, 100);

      Serial.println("Device Connected!");
      pServer->getAdvertising()->stop();
      deviceConnected = true;

    };

    void onDisconnect(BLEServer* pServer) {
      deviceConnected = false;

      // Restart advertising
      pServer->getAdvertising()->start();
    }
};

class CharacteristicsBeerLevelCallbacks: public BLECharacteristicCallbacks {
    void onWrite( BLECharacteristic* pChar )
    {
      kegDelivery();

    }
};

class CharacteristicsRGBCallbacks: public BLECharacteristicCallbacks {
    void onWrite( BLECharacteristic* pChar )
    {
      std::string  temp;

      temp = pRGBCharacteristic->getValue();

      ledColour = *(uint32_t*)&temp[0];

      showFill();
    }
};




int  currentFillLevel = 30;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);

  BLEDevice::init(DEVICE_NAME);
  BLEDevice::setMTU((PIXEL_COUNT * 4) + 10);
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());
  BLEService *pService = pServer->createService(SERVICE_UUID);
  ///////////////
  pBeerLevelCharacteristic = pService->createCharacteristic(
                               CHARACTERISTIC_BEERLEVEL_UUID,
                               BLECharacteristic::PROPERTY_WRITE |
                               BLECharacteristic::PROPERTY_READ |
                               BLECharacteristic::PROPERTY_NOTIFY |
                               BLECharacteristic::PROPERTY_INDICATE
                             );



  BeerLevelDescriptor.setValue("Beerlevel of current keg - (0-30)");
  BeerLevelDescriptor.setAccessPermissions(ESP_GATT_PERM_READ);

  pBeerLevelCharacteristic->addDescriptor(&BeerLevelDescriptor);
  pBeerLevelCharacteristic->addDescriptor(&BeerLevelCCCD);

  pBeerLevelCharacteristic->setValue(currentFillLevel);
  pBeerLevelCharacteristic->setCallbacks(new CharacteristicsBeerLevelCallbacks());
  ///////////
  pRGBCharacteristic = pService->createCharacteristic(
                         CHARACTERISTIC_RGB_UUID,
                         BLECharacteristic::PROPERTY_WRITE |
                         BLECharacteristic::PROPERTY_READ
                       );



  RGBDescriptor.setValue("Current RGB Colour");
  RGBDescriptor.setAccessPermissions(ESP_GATT_PERM_READ);

  pRGBCharacteristic->addDescriptor(&RGBDescriptor);
  pRGBCharacteristic->addDescriptor(&RGBCCCD);

  pRGBCharacteristic->setValue(ledColour);
  pRGBCharacteristic->setCallbacks(new CharacteristicsRGBCallbacks());
  ////////////
  pService->start();
  // BLEAdvertising *pAdvertising = pServer->getAdvertising();  // this still is working for backward compatibility
  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->setScanResponse(true);
  pAdvertising->setMinPreferred(0x06);  // functions that help with iPhone connections issue
  pAdvertising->setMinPreferred(0x12);
  BLEDevice::startAdvertising();
  Serial.println("Server started, characteristics can be written to");

  initAccelerometer();

  pixels.begin();

  showFill();
}


bool  currentTiltStatus = false;

void loop() {
  // put your main code here, to run repeatedly:
  int16_t x, y, z;
  float angle;

  readRegister(REG_OUT_X_L, (uint8_t*)&x, 2 );
  readRegister(REG_OUT_Y_L, (uint8_t*)&y, 2 );
  readRegister(REG_OUT_Z_L, (uint8_t*)&z, 2 );

  // Data is 14-bits, stored in the upper bits, so shift it over
  x = x >> 2;
  y = y >> 2;
  z = z >> 2;

  angle = (atan2(-z, -x) * 180 / M_PI);

  currentTiltStatus = (angle > 20);

  if ( currentTiltStatus  )
  {
    if ( pourColdOne() )
    {
      showFill();


    }
    delay(1000);
  }
  else
  {
    delay(500);
  }


}

/*
    decrements the fill level by 1.

    Returns true if successful, false if keg was empty
*/
bool pourColdOne ( void )
{
  if (currentFillLevel > 0)
  {
    currentFillLevel--;
    pBeerLevelCharacteristic->setValue(currentFillLevel);
    pBeerLevelCharacteristic->notify();
    return true;
  }
  else
  {
    return false;
  }
}

void kegDelivery( void )
{
  currentFillLevel = FULL_KEG_LEVEL;
  pBeerLevelCharacteristic->setValue(currentFillLevel);
  pBeerLevelCharacteristic->notify();
  showFill();
}

void showFill ( void )
{
  pixels.clear();

  int upperLimit, lowerLimit;

  upperLimit = currentFillLevel + 30;
  lowerLimit = 30 - currentFillLevel;

  for ( int x = lowerLimit; x <= upperLimit; x++ )
  {
    pixels.setPixelColor(x, ledColour);
  }

  pixels.show();
}



/*******************************
   Accelerometer functions
 *******************************/
void readRegister (  uint8_t address, uint8_t *regData, uint8_t size )
{
  Wire.beginTransmission(I2C_DEV_ADDR);
  Wire.write(address);
  Wire.endTransmission();

  Wire.requestFrom(I2C_DEV_ADDR, size);

  for (int x = 0; x < size; x++)
  {
    regData[x] = Wire.read();
  }
}


void writeRegister ( uint8_t address, uint8_t *regData, uint8_t size )
{
  Wire.beginTransmission(I2C_DEV_ADDR);
  Wire.write(&address, 1);

  for ( int x = 0; x < size; x++)
  {
    Wire.write(regData[x]);
  }

  Wire.endTransmission();
}

void initAccelerometer ( void )
{
  uint8_t reg;

  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN, (uint32_t)I2C_SPEED );


  readRegister(REG_CARD_ID, &reg, 1);

  Serial.print("ID is ");
  Serial.println(reg, HEX);

  if ( reg != 0x44 )      // Accelerometer is not correct chip
  {
    Serial.println("Accelerometer problem, please reboot!");
    while (1);
  }

  // Soft Reset the accelerometer
  readRegister(REG_CTRL_REG2, &reg, 1);
  reg = reg | 0b01000000;
  writeRegister(REG_CTRL_REG2, &reg, 1);

  // Set continuous refresh
  readRegister(REG_CTRL_REG2, &reg, 1);
  reg = reg | (1 << 3);         // enabled
  writeRegister(REG_CTRL_REG2, &reg, 1);

  // Set the data collection rate
  readRegister(REG_CTRL_REG1, &reg, 1);
  reg = reg | 0b01010000;       // 100Hz
  writeRegister(REG_CTRL_REG1, &reg, 1);

  // Set up I2C triggered single conversion
  readRegister(REG_CTRL_REG3, &reg, 1);
  reg = reg | 0b00000010;       // enabled
  writeRegister(REG_CTRL_REG3, &reg, 1);

  // Set the sensing range, and filter settings
  reg = reg | 0b11000100;       // ODR/20, +/-2G, Low-pass filter enabled, low-noise config enabled
  writeRegister(REG_CTRL_REG6, &reg, 1);

  // set the power mode
  reg = 0b01010001;       // 100Hz
  writeRegister(REG_CTRL_REG1, &reg, 1);


}