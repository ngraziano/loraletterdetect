
#include <Arduino.h>
#include <SPI.h>
#include <hal/hal_io.h>
#include <hal/print_debug.h>
#include <keyhandler.h>
#include <lmic.h>

#include <SparkFun_APDS9960.h>
#include <sleepandwatchdog.h>
#include <algorithm>
#define DEVICE_POSTDETECT
#include "lorakeys.h"
#include "powersave.h"

void do_send();
void reset_and_do_send();

SparkFun_APDS9960 apds = SparkFun_APDS9960();

// Schedule TX every this many seconds (might become longer due to duty
// cycle limitations).
const OsDeltaTime TX_INTERVAL = OsDeltaTime::from_sec(60 * 60);
// keep ON for one minute
const OsDeltaTime TX_ONLENGTH = OsDeltaTime::from_sec(1 * 60);

const unsigned int BAUDRATE = 19200;

// Pin mapping
const lmic_pinmap lmic_pins = {
    .nss = 10,
    .prepare_antenna_tx = nullptr,
    .rst = 14,
    .dio = {9, 8},
};

RadioSx1276 radio{lmic_pins};
LmicEu868 LMIC(radio);

OsTime nextSend;

const uint8_t NUMBERTIME_TO_SEND = 3;
uint8_t apds_tosend = 0;
bool apds_new = false;

uint16_t readBat() { return analogRead(A1) * (6.6 / 1024 * 100); }

void initProximitySensor(uint8_t threshold, uint8_t ledDrive, uint8_t gain) {

  if (apds.init()) {
    PRINT_DEBUG(1,F("APDS-9960 initialization complete"));
  } else {
    PRINT_DEBUG(1,F("Something went wrong during APDS-9960 init!"));
  }

  // Set proximity interrupt thresholds
  apds.setProximityIntLowThreshold(0);
  apds.setProximityIntHighThreshold(threshold);
  apds.setProximityGain(gain); // DEFAULT_PGAIN
  apds.setLEDDrive(ledDrive);  // LED_DRIVE_50MA
  apds.setProximityIntEnable(1);
  apds.enablePower();
  apds.setMode(PROXIMITY, 1);
}

void disableProximitySensor() {
  apds.setMode(PROXIMITY, 0);
  apds.disablePower();
}

void enableProximitySensor() {
  apds.enablePower();
  apds.setMode(PROXIMITY, 1);
}

void waitBatOk() {
  auto batLevel = readBat();
  // wait here for battery to gain a little of charge
  if (batLevel < 360 && batLevel>100) {
    disableProximitySensor();
    while (batLevel < 370) {
      PRINT_DEBUG(1, F("Bat level %i"), batLevel);
      powersave(OsDeltaTime::from_sec(30), []() { return false; });
      delay(200);
      batLevel = readBat();
    }
    enableProximitySensor();
  }
}

void apdsInterrupt() {
  // If we are still sending detect event
  // do not try do detect another one.
  if (apds_tosend > 0)
    return;
  if (!digitalRead(3)) {
    apds_tosend = NUMBERTIME_TO_SEND;
    apds_new = true;
  }
}

void onEvent(EventType ev) {
  rst_wdt();
  
  switch (ev) {
  case EventType::JOINED:
    PRINT_DEBUG(2, F("EV_JOINED"));
    LMIC.setDutyRate(11);
    break;
  case EventType::TXCOMPLETE:
    PRINT_DEBUG(2, F("EV_TXCOMPLETE (includes waiting for RX windows)"));
//    waitBatOk();
    if (LMIC.getTxRxFlags().test(TxRxStatus::ACK))
      PRINT_DEBUG(1, F("Received ack"));
    if (LMIC.getDataLen() >= 3) {
      PRINT_DEBUG(1, F("Received %d  bytes of payload"), LMIC.getDataLen());

      auto data = LMIC.getData();
      if (data) {
        auto port = LMIC.getPort();
        if (port == 9) {
          disableProximitySensor();
          delay(500);
          initProximitySensor(data[0], data[1], data[2]);
        }
      }
    }
    break;
  }
}

void reset_and_do_send() {
  // we have sent one
  if (apds_tosend > 0) {
    apds_tosend--;
    if (apds_tosend == 0)
      apds.clearProximityInt();
  } 

  do_send();
  nextSend = os_getTime() + (apds_tosend ? TX_ONLENGTH :TX_INTERVAL);
}

void do_send() {
  const uint8_t pinCmd = 6;
  pinMode(pinCmd, OUTPUT);
  digitalWrite(pinCmd, 1);
  delay(100);
  uint8_t data[5];
  // battery
  uint8_t i=0;
  uint16_t val = readBat();
  data[i++] = val >> 8;
  data[i++] = val;

  if(LMIC.getTxRxFlags().test(TxRxStatus::NEED_BATTERY_LEVEL)) {
    LMIC.setBatteryLevel( 255 * (val - 270 )/(420-270) );
  }

  uint8_t prox;
  apds.readProximity(prox);
  val = prox * 100;
  data[i++] = val >> 8;
  data[i++] = val;
  // signal
  data[i++] = apds_tosend > 0 ? 1 : 0;

  // Prepare upstream data transmission at the next possible time.
  LMIC.setTxData2(10, (uint8_t *)data, 5, false);
  PRINT_DEBUG(1,F("Packet queued"));
  
}

// lmic_pins.dio[0]  = 9 => PCINT1
// lmic_pins.dio[1]  = 8 => PCINT0
// PCI2 PCINT[23:16]
// PCI1 PCINT[14:8]
// PCI0 PCINT[7:0]

ISR(PCINT0_vect) {
  // one of pins D8 to D13 has changed
  // store time, will be check in OSS.runloopOnce()
  LMIC.store_trigger();
}

void pciSetup(byte pin) {
  *digitalPinToPCMSK(pin) |= bit(digitalPinToPCMSKbit(pin)); // enable pin
  PCIFR |= bit(digitalPinToPCICRbit(pin)); // clear any outstanding interrupt
  PCICR |= bit(digitalPinToPCICRbit(pin)); // enable interrupt for the group
}

void testDuration(int32_t ms) {
  const auto delta = OsDeltaTime::from_ms(ms);
  PRINT_DEBUG(1, F("Test sleep time for %i ms."), ms);
  const OsTime start = os_getTime();
  PRINT_DEBUG(1, F("Start Test sleep time."));
  powersave(delta, []() { return false; });
  const OsTime end = os_getTime();
  PRINT_DEBUG(1, F("End Test sleep time."));
  PRINT_DEBUG(1, F("Test Time should be : %d ms"), (end - start).to_ms());
}

void setup() {
#if LMIC_DEBUG_LEVEL > 0
  Serial.begin(BAUDRATE);
  Serial.println(F("Starting"));
#endif

  pinMode(3, INPUT);
  attachInterrupt(digitalPinToInterrupt(3), apdsInterrupt, FALLING);
  initProximitySensor(40, LED_DRIVE_50MA, DEFAULT_PGAIN);

  pciSetup(lmic_pins.dio[0]);
  pciSetup(lmic_pins.dio[1]);

  SPI.begin();
  // LMIC init
  os_init();
  LMIC.init();
  // Reset the MAC state. Session and pending data transfers will be discarded.
  LMIC.reset();
  LMIC.setEventCallBack(onEvent);
  SetupLmicKey<appEui, devEui, appKey>::setup(LMIC);

  // set clock error to allow good connection.
  LMIC.setClockError(MAX_CLOCK_ERROR * 5 / 100);
  //    LMIC.setAntennaPowerAdjustment(-10);

  testDuration(15000);

  waitBatOk();
  configure_wdt();
  // Start job (sending automatically starts OTAA too)
  nextSend = os_getTime();
}

void loop() {
  rst_wdt();
  OsDeltaTime freeTimeBeforeNextCall = LMIC.run();

  // we have more than 10 ms to do some work.
  // the test must be adapted from the time spend in other task
 if (freeTimeBeforeNextCall > OsDeltaTime::from_ms(10)) {
    // if time to send or wakeup
    if (nextSend < os_getTime() || apds_new) {
      if (LMIC.getOpMode().test(OpState::TXRXPEND)) {
        PRINT_DEBUG(1, F("OpState::TXRXPEND, not sending"));
      } else {
        apds_new =false;
        reset_and_do_send();
      }
    } else {
      waitBatOk();

      OsDeltaTime freeTimeBeforeSend = nextSend - os_getTime();
      OsDeltaTime to_wait =
          std::min(freeTimeBeforeNextCall, freeTimeBeforeSend);
      // Go to sleep if we have nothing to do.
      powersave(to_wait, []() {
        // Check if we are wakeup by external pin.
        apdsInterrupt();
        return apds_new;
      });
    }
  }
}