#include <Arduino.h>
#include <bluefruit.h>
#include <nrfx_spis.h>

// define SERIAL_DEBUG_ENABLED

#ifdef SERIAL_DEBUG_ENABLED
#define DEBUG_PRINT(x) Serial.print(x)
#define DEBUG_PRINTDEC(x) Serial.print(x, DEC)
#define DEBUG_PRINTLN(x) Serial.println(x)
#define DEBUG_PRINTLNF(x, y) Serial.println(x, y)
#else
#define DEBUG_PRINT(x)
#define DEBUG_PRINTDEC(x)
#define DEBUG_PRINTLN(x)
#define DEBUG_PRINTLNF(x, y)
#endif

#define DIO 25   // P0.13
#define SCLK 24  // P0.15
#define STB 1    // P0.24
#define BUT_P 19 // P0.03
#define BUT_T 17 // P0.28
#define LED_L 18 // P0.02
#define LED_R 4  // P1.10

#define SCALES_NAME "Tencent Scales"
#define UUID16_SVC_SCALES 0xFFF0
#define UUID16_CHR_SCALES_READ 0x36F5
#define UUID16_CHR_SCALES_WRITE 0xFFF4

BLEDis bledis; // DIS (Device Information Service) helper class instance
BLEBas blebas; // BAS (Battery Service) helper class instance

BLEService ss = BLEService(UUID16_SVC_SCALES);
BLECharacteristic rc = BLECharacteristic(UUID16_CHR_SCALES_READ);
BLECharacteristic wc = BLECharacteristic(UUID16_CHR_SCALES_WRITE);

bool isConnected;
bool isNotifyEnabled;

bool weightChanged, timeChanged;
int16_t previousWeight, currentWeight;
int16_t previousTime, currentTime;

#define SPIS_CS_PIN ((uint32_t)24)   // STB
#define SPIS_MOSI_PIN ((uint32_t)13) // DIO
#define SPIS_SCK_PIN ((uint32_t)15)  // SCLK

#define SPIS_INSTANCE 0                                            /**< SPIS instance index. */
static const nrfx_spis_t spis = NRFX_SPIS_INSTANCE(SPIS_INSTANCE); /**< SPIS instance. */

static const uint8_t m_length = 16; /**< Transfer length. */
static uint8_t m_rx_buf[m_length];  /**< RX buffer. */

static volatile bool spis_xfer_done; /**< Flag used to indicate that SPIS instance completed the transfer. */
static volatile int m_rx_bufPos = 0;

void spis_event_handler(nrfx_spis_evt_t const *event, void *context)
{
  if ((*event).evt_type == NRFX_SPIS_XFER_DONE)
  {
    m_rx_bufPos = (*event).rx_amount - 1;

    spis_xfer_done = true;
  }
}

void setup_spis(void)
{
  nrfx_spis_config_t spis_config = NRFX_SPIS_DEFAULT_CONFIG(SPIS_SCK_PIN, SPIS_MOSI_PIN, NRFX_SPIS_PIN_NOT_USED, SPIS_CS_PIN);

  // spis_config.sck_pin = SPIS_SCK_PIN;
  // spis_config.mosi_pin = SPIS_MOSI_PIN;
  // spis_config.csn_pin = SPIS_CS_PIN;

  spis_config.bit_order = NRF_SPIS_BIT_ORDER_LSB_FIRST;
  spis_config.mode = NRF_SPIS_MODE_0;

  nrfx_err_t result = nrfx_spis_init(&spis, &spis_config, spis_event_handler, (void *)NULL);

  if (result > NRFX_ERROR_BASE_NUM)
  {
    DEBUG_PRINT(" Error:");
    DEBUG_PRINTLN(result);
  }

  spis_xfer_done = true;
}

void connect_callback(uint16_t conn_handle)
{
  // Get the reference to current connection
  BLEConnection *connection = Bluefruit.Connection(conn_handle);

  char central_name[32] = {0};
  connection->getPeerName(central_name, sizeof(central_name));

  isConnected = true;

  DEBUG_PRINT("Connected to ");
  DEBUG_PRINTLN(central_name);
}

/**
 * Callback invoked when a connection is dropped
 * @param conn_handle connection where this event happens
 * @param reason is a BLE_HCI_STATUS_CODE which can be found in ble_hci.h
 */
void disconnect_callback(uint16_t conn_handle, uint8_t reason)
{
  (void)conn_handle;
  (void)reason;

  isConnected = false;
  isNotifyEnabled = false;

  DEBUG_PRINT("Disconnected, reason = 0x");
  DEBUG_PRINTLNF(reason, HEX);
  DEBUG_PRINTLN("Advertising!");
}

bool isInTare;
bool isInTimer;

uint8_t timerState;

void write_callback(uint16_t conn_hdl, BLECharacteristic *chr, uint8_t *data, uint16_t len)
{
  if (len == 7)
  {
    if (data[0] == 0x03)
    {
      if (data[1] == 0x0F)
      {
        if (!isInTare)
        {
          isInTare = true;
        }
      }
      else if (data[1] == 0x0B)
      {
        if (!isInTimer)
        {
          timerState = data[2];
          isInTimer = true;
        }
      }
    }
  }
}

void cccd_callback(uint16_t conn_hdl, BLECharacteristic *chr, uint16_t cccd_value)
{
  // Display the raw request packet
  DEBUG_PRINT("CCCD Updated: ");
  // Serial.printBuffer(request->data, request->len);
  DEBUG_PRINTLN(cccd_value);

  // Check the characteristic this CCCD update is associated with in case
  // this handler is used for multiple CCCD records.
  if (chr->uuid == wc.uuid)
  {
    if (chr->notifyEnabled(conn_hdl))
    {
      isNotifyEnabled = true;
      // TODO            loadcell.power_up();
      DEBUG_PRINTLN("Weight Measurement 'Notify' enabled");
    }
    else
    {
      isNotifyEnabled = false;
      // TODO            loadcell.power_down();
      DEBUG_PRINTLN("Weight Measurement 'Notify' disabled");
    }
  }
}

void setupWS(void)
{
  ss.begin();

  wc.setProperties(CHR_PROPS_READ | CHR_PROPS_NOTIFY);

  wc.setPermission(SECMODE_OPEN, SECMODE_NO_ACCESS);
  wc.setFixedLen(7);

  wc.setCccdWriteCallback(cccd_callback); // Optionally capture CCCD updates
  wc.begin();

  rc.setProperties(CHR_PROPS_WRITE_WO_RESP);

  rc.setPermission(SECMODE_OPEN, SECMODE_OPEN);
  rc.setFixedLen(7);

  rc.setWriteCallback(write_callback);
  rc.begin();
}

void startAdv(void)
{
  // Advertising packet
  Bluefruit.Advertising.addFlags(BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE);
  Bluefruit.Advertising.addTxPower();

  // Include WS Service UUID
  Bluefruit.Advertising.addService(ss);

  // Bluefruit.Advertising.addService(bleuart);

  // Include Name
  Bluefruit.Advertising.addName();

  /* Start Advertising
   * - Enable auto advertising if disconnected
   * - Interval:  fast mode = 20 ms, slow mode = 152.5 ms
   * - Timeout for fast mode is 30 seconds
   * - Start(timeout) with timeout = 0 will advertise forever (until connected)
   *
   * For recommended advertising interval
   * https://developer.apple.com/library/content/qa/qa1931/_index.html
   */
  Bluefruit.Advertising.restartOnDisconnect(true);
  Bluefruit.Advertising.setInterval(32, 244); // in unit of 0.625 ms
  Bluefruit.Advertising.setFastTimeout(30);   // number of seconds in fast mode
  Bluefruit.Advertising.start(0);             // 0 = Don't stop advertising after n seconds
}

void setup_ble(void)
{

  // Initialise the Bluefruit module
  Bluefruit.begin();
  Bluefruit.setName(SCALES_NAME);

  // Set the connect/disconnect callback handlers
  Bluefruit.Periph.setConnectCallback(connect_callback);
  Bluefruit.Periph.setDisconnectCallback(disconnect_callback);

  Bluefruit.setConnLedInterval(250);

  // Configure and Start the Device Information Service
  DEBUG_PRINTLN("Configuring the Device Information Service");
  bledis.setManufacturer("Adafruit Industries");
  bledis.setModel(SCALES_NAME);
  bledis.begin();

  blebas.begin();
  blebas.write(100);

  // bleuart.begin();

  // Setup the Scales service using
  // BLEService and BLECharacteristic classes
  DEBUG_PRINTLN("Configuring the Scales Service");
  setupWS();

  // Setup the advertising packet(s)
  DEBUG_PRINTLN("Setting up the advertising payload(s)");
  startAdv();

  DEBUG_PRINTLN("Ready Player One!!!");
  DEBUG_PRINTLN("\nAdvertising");
}

void setup(void)
{

#ifdef SERIAL_DEBUG_ENABLED
  Serial.begin(921600);
  while (!Serial)
    delay(10); // for nrf52840 with native usb
  // delay(10000);
  DEBUG_PRINTLN("Starting...");
#endif

  pinMode(BUT_P, INPUT);
  pinMode(BUT_T, INPUT);
  pinMode(LED_L, OUTPUT_D0H1);
  pinMode(LED_R, OUTPUT_D0H1);
  pinMode(PIN_VBAT, INPUT);

  digitalWrite(LED_R, LOW);
  digitalWrite(LED_L, LOW);

  setup_spis();

  setup_ble();
}

void parseC0(uint8_t buf[], char segments[])
{

  u_int16_t rows[7];

  for (int i = 0; i < 7; i++)
  {
    rows[i] = 0;
    rows[i] |= buf[i * 2 + 1];
    rows[i] |= buf[i * 2 + 2] << 8;
  }

  for (int i = 0; i < 10; i++)
  {
    u_int8_t byte = 0U;

    for (int j = 0; j < 7; j++)
    {
      bool hasBit = (rows[j] & (1 << i)) > 0;

      if (hasBit)
      {
        byte |= 1 << j;
      }
    }

    segments[i] = byte;
  }

  for (int i = 0; i < 10; i++)
  {
    u_int8_t byte = segments[i];

    switch (byte)
    {
    case 0b00111111:
      segments[i] = 0;
      break;

    case 0b00000110:
      segments[i] = 1;
      break;

    case 0b01011011:
      segments[i] = 2;
      break;

    case 0b01001111:
      segments[i] = 3;
      break;

    case 0b01100110:
      segments[i] = 4;
      break;

    case 0b01101101:
      segments[i] = 5;
      break;

    case 0b01111101:
      segments[i] = 6;
      break;

    case 0b00000111:
      segments[i] = 7;
      break;

    case 0b01111111:
      segments[i] = 8;
      break;

    case 0b01101111:
      segments[i] = 9;
      break;

    case 0b01000000:
      segments[i] = 0x0F;
      break;

    default:
      segments[i] = 0;
      break;
    }
  }
}

void parseWeightTime(char segments[])
{
  int time = segments[0] * 600 + segments[1] * 60 + segments[2] * 10 + segments[3];
  int weight = segments[5] * 1000 + segments[6] * 100 + segments[7] * 10 + segments[8];
  if (segments[4] == 0x0F)
  {
    weight *= (-1);
  }
  else
  {
    weight += segments[4] * 10000;
  }

  currentWeight = weight;
  currentTime = time;

  if (previousWeight != currentWeight)
  {
    weightChanged = true;
  }

  if (previousTime != currentTime)
  {
    timeChanged = true;
  }
}

void parseBuf(uint8_t buf[], int len)
{
  if (buf[0] == 0x03)
  {
    DEBUG_PRINTLN("0x03");
  }
  else if (buf[0] == 0x40)
  {
    DEBUG_PRINTLN("0x40");
  }
  else if (buf[0] == 0x8C)
  {
    DEBUG_PRINTLN("0x8C");
  }
  else if (buf[0] == 0xC0)
  {
    DEBUG_PRINTLN("0xC0");

    char segments[10];
    parseC0(buf, segments);

    if (segments[7] == 0x0F || segments[8] == 0x0F)
    {
      // in charging mode
    }
    else
    {
      parseWeightTime(segments);
    }
  }
  else
  {
    DEBUG_PRINTLN(buf[0]);
  }
}

uint32_t lastSpisUpdate = 0UL;

void read_spis(void)
{
  if (spis_xfer_done)
  {
    parseBuf(m_rx_buf, m_rx_bufPos + 1);

    spis_xfer_done = false;
    nrfx_err_t result = nrfx_spis_buffers_set(&spis, (uint8_t *)NULL, 0, m_rx_buf, m_length);

    if (result > NRFX_ERROR_BASE_NUM)
    {
      DEBUG_PRINT(" Error:");
      DEBUG_PRINTLN(result);
    }

    lastSpisUpdate = millis();
  }
}

unsigned long clockMillisOffset = 0;

void notifyWeight(int16_t weight)
{
  DEBUG_PRINTLN("---");
  DEBUG_PRINTLNF(highByte(weight), BIN);
  DEBUG_PRINTLNF(lowByte(weight), BIN);
  DEBUG_PRINTLN(weight);

  unsigned long currentMillis = millis() - clockMillisOffset;

  uint8_t minutesOn = currentMillis / 60000;
  uint8_t secondsOn = (currentMillis - (minutesOn * 60000)) / 1000;
  uint8_t millis100On = (currentMillis - (minutesOn * 60000) - (secondsOn * 1000)) / 100;

  DEBUG_PRINT(minutesOn);
  DEBUG_PRINT(":");
  DEBUG_PRINT(secondsOn);
  DEBUG_PRINT(":");
  DEBUG_PRINTLN(millis100On);

  // uint8_t wsdata[10] = {0x03, 0xCA, highByte(weight), lowByte(weight), minutesOn, secondsOn, millis100On, 0, 0, 0};
  // wsdata[9] = wsdata[0] ^ wsdata[1] ^ wsdata[2] ^ wsdata[3] ^ wsdata[4] ^ wsdata[5] ^ wsdata[6] ^ wsdata[7] ^ wsdata[8];

  uint8_t wsdata[7] = {0x03, 0xCA, highByte(weight), lowByte(weight), 0, 0, 0};
  wsdata[6] = wsdata[0] ^ wsdata[1] ^ wsdata[2] ^ wsdata[3] ^ wsdata[4] ^ wsdata[5];

  if (isNotifyEnabled)
  {
    wc.notify(wsdata, sizeof(wsdata));
  }
}

uint8_t tareCounter = 0;

void notifyTareDone()
{
  uint8_t wsdata[7] = {0x03, 0x0F, tareCounter, 0, 0, 0xFE, 0};
  wsdata[6] = wsdata[0] ^ wsdata[1] ^ wsdata[2] ^ wsdata[3] ^ wsdata[4] ^ wsdata[5];

  if (isNotifyEnabled)
  {
    wc.notify(wsdata, sizeof(wsdata));
  }
}

uint32_t weightUpdateMillis = 0UL;
uint32_t inTareUpdateMillis = 0UL;
uint32_t batteryUpdateMillis = 0UL;

void loop()
{
  digitalWrite(LED_L, HIGH);

  read_spis();

  if (isInTare)
  {
    if (inTareUpdateMillis == 0UL)
    {
      pinMode(BUT_T, OUTPUT_S0D1);
      digitalWrite(BUT_T, LOW);
      inTareUpdateMillis = millis() + 100UL;
    }
    else if (inTareUpdateMillis < millis())
    {
      isInTare = false;
      digitalWrite(BUT_T, HIGH);
      pinMode(BUT_T, INPUT);
      tareCounter++;
      notifyTareDone();
    }
  }
  else
  {
    inTareUpdateMillis = 0UL;
  }

  if (isNotifyEnabled)
  {
    if (weightUpdateMillis < millis())
    {
      notifyWeight(currentWeight);
      weightUpdateMillis = millis() + 100UL;
    }
  }
  else
  {
    weightUpdateMillis = 0;
  }

  if (isConnected)
  {
    if (batteryUpdateMillis < millis())
    {
      uint32_t battery = analogRead(PIN_VBAT);

      blebas.write((battery - 666) / 2.34);

      batteryUpdateMillis = millis() + 1000UL;
    }
  }

  if (millis() > (lastSpisUpdate + 3000))
  {
    if (isConnected)
    {
      Bluefruit.disconnect(Bluefruit.connHandle());
    }
    else
    {
      digitalWrite(LED_L, LOW);
      digitalWrite(LED_R, LOW);
      // this code will change SPIS pin setup so it is expected for the MCU to go into a reset after the wake up
      (void)nrf_gpio_pin_read(SPIS_SCK_PIN);
      if (nrf_gpio_pin_read(SPIS_SCK_PIN))
      {
        nrf_gpio_cfg_sense_set(SPIS_SCK_PIN, NRF_GPIO_PIN_SENSE_LOW);
      }
      else
      {
        nrf_gpio_cfg_sense_set(SPIS_SCK_PIN, NRF_GPIO_PIN_SENSE_HIGH);
      }
      if (sd_power_system_off() == NRF_SUCCESS)
      {
        while (1)
          ;
      }
    }
  }
}
