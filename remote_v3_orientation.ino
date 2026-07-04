#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <WiFiUdp.h>
#include <SPI.h>
#include <Adafruit_BNO08x.h>

// ---- Transmission method (compile-time switch) ----
// Use ESP-NOW normally; fall back to UDP-over-WiFi if the ESP-NOW receiver
// is giving us trouble. Flip this to TRANSMIT_UDP and reflash to switch.
#define TRANSMIT_ESPNOW 0
#define TRANSMIT_UDP 1
#define TRANSMIT_MODE TRANSMIT_ESPNOW

// ---- UDP transmission settings (only used when TRANSMIT_MODE == TRANSMIT_UDP) ----
static const char* WIFI_SSID = "aleph";
static const char* WIFI_PASSWORD = "shibboleth";
static const IPAddress UDP_SERVER_IP(192, 168, 1, 52);
static const uint16_t UDP_SERVER_PORT = 5005;
static const uint16_t UDP_LOCAL_PORT = 5004;

// Note we do not disable WiFi modem sleep because this device only transmits
WiFiUDP udp;

// ---- Non-BNO pin assignments ----
static const int PIN_LED = 5;       // This is the configurable LED which we will use to communicate
static const int PIN_BUTTON1 = 3;   // This is button 1
static const int PIN_BOOT_BTN = 9;  // This is also button 2
static const int PIN_SWITCH = 6;    // Transmit switch - toggling this should turn off any data transmission

// ---- BNO085 SPI pin assignments ----
static const int PIN_BNO_SCK = 10;  // IO10 -> SCL_SCK
static const int PIN_BNO_MISO = 1;  // IO1  -> SDA_MISO
static const int PIN_BNO_MOSI = 4;  // IO4  -> MOSI
static const int PIN_BNO_CS = 8;    // IO8  -> BNO_CS
static const int PIN_BNO_INT = 7;   // IO7  -> BNO_INT
static const int PIN_BNO_RST = 0;   // IO0  -> NRESET_BNO

// Broadcast is fine for our use case here
static const uint8_t receiver_mac[] = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF };

Adafruit_BNO08x bno08x(PIN_BNO_RST);
sh2_SensorValue_t sensorValue;

const int DATAGRAM_REPEATS = 10;

// ---- Button handling ----
// Buttons are wired with internal pull-ups: LOW = pressed, HIGH = released.
const unsigned long BUTTON_DEBOUNCE_MS = 30;     // ignore edges faster than this (mechanical bounce)
const unsigned long CALIBRATION_HOLD_MS = 1250;  // both buttons held this long triggers calibration

struct Button {
  int pin;
  bool pressed;                  // debounced state: true while held down
  bool raw_pressed;              // last raw reading
  unsigned long last_change_ms;  // when the raw reading last flipped
};

Button button_1 = { PIN_BUTTON1, false, false, 0 };
Button button_2 = { PIN_BOOT_BTN, false, false, 0 };

bool button_1_was_pressed = false;  // debounced state on the previous update (for edge detection)
bool button_2_was_pressed = false;
bool combo_engaged = false;  // both buttons were down together at some point this gesture
bool combo_fired = false;    // calibration already fired; wait for full release before re-arming
unsigned long combo_hold_start = 0;

// Neither this device nor the receiver joins a WiFi AP (there's no wireless
// network in this environment), so nothing else fixes the channel for us --
// both sides must be pinned to the same one explicitly.
const uint8_t WIFI_CHANNEL = 1;

// Identifies this device's kind to the receiver (orientation remote v3 [2026] = 6).
static const uint8_t DEVICE_TYPE = 6;

typedef struct __attribute__((packed)) {
  uint8_t id;
  uint32_t time;
  uint8_t device_type;
  uint8_t seq;
  int16_t w;
  int16_t x;
  int16_t y;
  int16_t z;
  uint8_t action_flag;
} orientation_packet_t;

uint8_t id;
uint8_t packet_seq = 0;  // increments on every packet sent; wraps at 255
uint8_t action_flag = 0;
int action_flag_repeats = 0;

bool led_state = false;
unsigned long last_led_toggle_time = 0;
const unsigned long LED_BLINK_SLOW_MS = 500;  // accuracy 1: toggling every 500ms gives a 1-second blink cycle
const unsigned long LED_BLINK_FAST_MS = 125;  // accuracy 0: toggle quickly for a rapid blink

// A setup step that hasn't succeeded after this long starts blinking an error
// code, so a dead sensor / down AP is visibly different from a dead battery.
const unsigned long SETUP_ERROR_TIMEOUT_MS = 5000;

// On-site LED error legend (short blinks, then a longer gap):
//   2 blinks -> BNO085 sensor not responding (still retrying)
//   3 blinks -> fatal ESP-NOW init failure (halted)
//   4 blinks -> WiFi/AP unreachable (still retrying)
void blinkErrorCode(int count) {
  for (int i = 0; i < count; i++) {
    digitalWrite(PIN_LED, HIGH);
    delay(150);
    digitalWrite(PIN_LED, LOW);
    delay(200);
  }
  delay(600);
}

// Fatal setup failure: never return to loop() with a half-initialized
// transmission stack. Halt here and blink a distinct code so a dead radio is
// visibly different from a dead battery.
void haltWithError() {
  for (;;) {
    blinkErrorCode(3);
  }
}

void setup(void) {
  Serial.begin(115200);

  // Configure pins first so the LED can signal errors and the buttons/switch
  // read correctly even if a later init step fails and we halt.
  pinMode(PIN_BUTTON1, INPUT_PULLUP);
  pinMode(PIN_BOOT_BTN, INPUT_PULLUP);
  pinMode(PIN_SWITCH, INPUT_PULLUP);
  pinMode(PIN_LED, OUTPUT);

  SPI.begin(PIN_BNO_SCK, PIN_BNO_MISO, PIN_BNO_MOSI, PIN_BNO_CS);
  unsigned long sensor_start = millis();
  while (!bno08x.begin_SPI(PIN_BNO_CS, PIN_BNO_INT)) {
    if (millis() - sensor_start >= SETUP_ERROR_TIMEOUT_MS) {
      blinkErrorCode(2);  // sensor not responding
    } else {
      delay(10);
    }
  }
  // We use SH2_ARVR_STABILIZED_RV because we want full fusion and this is an outdoors application
  // so magnetic interference should be low (i think)
  bno08x.enableReport(SH2_ARVR_STABILIZED_RV, 2500);

  WiFi.mode(WIFI_STA);
  uint8_t own_mac[6];
  WiFi.macAddress(own_mac);
  id = own_mac[5];

#if TRANSMIT_MODE == TRANSMIT_ESPNOW
  esp_wifi_set_channel(WIFI_CHANNEL, WIFI_SECOND_CHAN_NONE);

  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    haltWithError();
  }

  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, receiver_mac, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;
  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Failed to add ESP-NOW peer");
    haltWithError();
  }
  Serial.println("ESP-NOW peer added");
#else
  Serial.println("Connecting to WiFi");
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  unsigned long wifi_start = millis();
  while (WiFi.status() != WL_CONNECTED) {
    if (millis() - wifi_start >= SETUP_ERROR_TIMEOUT_MS) {
      blinkErrorCode(4);  // WiFi/AP unreachable
    } else {
      delay(250);
      Serial.println(".");
    }
  }
  Serial.println("Connected to WiFi");
  udp.begin(UDP_LOCAL_PORT);
  Serial.println("UDP socket created");
#endif

  Serial.println("Setup completed");
}

void sendPacket(const orientation_packet_t& packet) {
#if TRANSMIT_MODE == TRANSMIT_ESPNOW
  esp_now_send(receiver_mac, reinterpret_cast<const uint8_t*>(&packet), sizeof(packet));
#else
  udp.beginPacket(UDP_SERVER_IP, UDP_SERVER_PORT);
  udp.write(reinterpret_cast<const uint8_t*>(&packet), sizeof(packet));
  udp.endPacket();
#endif
}

// Read a button with debounce; returns its debounced pressed-state.
bool readButton(Button& b) {
  bool raw = (digitalRead(b.pin) == LOW);  // pull-up: LOW means pressed
  if (raw != b.raw_pressed) {
    b.raw_pressed = raw;
    b.last_change_ms = millis();
  }
  if (millis() - b.last_change_ms >= BUTTON_DEBOUNCE_MS) {
    b.pressed = raw;
  }
  return b.pressed;
}

// Translate button activity into action_flag:
//   1 = button 1 clicked (pressed and released on its own)
//   2 = button 2 clicked
//   4 = both buttons held together for CALIBRATION_HOLD_MS (calibration)
// Single clicks are emitted on *release* so the start of a two-button hold
// never looks like a stray single click. Calibration fires once per hold and
// re-arms only after both buttons are fully released.
void updateButtons() {
  bool b1 = readButton(button_1);
  bool b2 = readButton(button_2);

  // Only assign a new action while none is in flight: an in-flight flag is
  // repeated across several packets and must not be clobbered mid-send.
  if (action_flag == 0) {
    if (b1 && b2) {
      combo_engaged = true;
      if (combo_hold_start == 0) {
        combo_hold_start = millis();
      }
      if (!combo_fired && millis() - combo_hold_start >= CALIBRATION_HOLD_MS) {
        action_flag = 4;
        combo_fired = true;
      }
    } else {
      combo_hold_start = 0;
    }

    // Falling edge (released): emit the single-button click, unless this
    // button was part of a two-button gesture.
    if (button_1_was_pressed && !b1 && !combo_engaged) {
      action_flag = 1;
    }
    if (button_2_was_pressed && !b2 && !combo_engaged) {
      action_flag = 2;
    }
  }

  // Both buttons up: the gesture is over, re-arm everything.
  if (!b1 && !b2) {
    combo_engaged = false;
    combo_fired = false;
    combo_hold_start = 0;
  }

  button_1_was_pressed = b1;
  button_2_was_pressed = b2;
}

void loop(void) {
  if (bno08x.wasReset()) {
    Serial.println("sensor was reset ");
    bno08x.enableReport(SH2_ARVR_STABILIZED_RV, 2500);
  }

  if (!bno08x.getSensorEvent(&sensorValue)) {
    return;
  }

  // Only the ARVR-stabilized RV report carries the quaternion we read below.
  // The sensor can deliver other reports (especially around resets/startup);
  // reading the wrong union member would send garbage quaternions.
  if (sensorValue.sensorId != SH2_ARVR_STABILIZED_RV) {
    return;
  }

  // HIGH means it's in the "off" position, LOW means it's in the "on" position
  int transmit_switch_state = digitalRead(PIN_SWITCH);

  if (transmit_switch_state == HIGH) {
    // Transmit switch is toggled off -- skip sending packets and keep the LED dark.
    digitalWrite(PIN_LED, LOW);
    led_state = false;
  } else {
    // Actions (see updateButtons):
    // 1: Button 1 clicked
    // 2: Button 2 clicked
    // 4: press and hold buttons 1 and 2 for a bit (triggers calibration)
    // Once the action flag is set it holds that value until it has been sent in
    // a series of packets, since we can't guarantee any single packet arrives.
    updateButtons();

    // Use the LED to report the BNO085 calibration accuracy (low 2 bits of
    // status, 0 = unreliable .. 3 = high accuracy). The ARVR-stabilized RV
    // rarely advertises 3, so medium accuracy counts as calibrated:
    //   >= 2  -> solid on
    //   1     -> blink once a second
    //   0     -> blink rapidly
    uint8_t accuracy = sensorValue.status & 0x03;
    if (accuracy >= 2) {
      led_state = true;
      digitalWrite(PIN_LED, HIGH);
    } else {
      unsigned long blink_interval = (accuracy == 1) ? LED_BLINK_SLOW_MS : LED_BLINK_FAST_MS;
      if (millis() - last_led_toggle_time >= blink_interval) {
        led_state = !led_state;
        digitalWrite(PIN_LED, led_state ? HIGH : LOW);
        last_led_toggle_time = millis();
      }
    }

    int16_t w = static_cast<int16_t>(sensorValue.un.arvrStabilizedRV.real * (1 << 14));
    int16_t x = static_cast<int16_t>(sensorValue.un.arvrStabilizedRV.i * (1 << 14));
    int16_t y = static_cast<int16_t>(sensorValue.un.arvrStabilizedRV.j * (1 << 14));
    int16_t z = static_cast<int16_t>(sensorValue.un.arvrStabilizedRV.k * (1 << 14));
    // The transmit switch now gates transmission, so send on every sensor event
    // regardless of whether the orientation changed.
    orientation_packet_t packet;
    packet.id = id;
    packet.time = millis();
    packet.device_type = DEVICE_TYPE;
    packet.seq = packet_seq++;
    packet.w = w;
    packet.x = x;
    packet.y = y;
    packet.z = z;
    packet.action_flag = action_flag;
    sendPacket(packet);
    if (action_flag != 0 && action_flag_repeats > DATAGRAM_REPEATS) {
      action_flag = 0;
      action_flag_repeats = 0;
    } else if (action_flag != 0) {
      action_flag_repeats++;
    }
  }
}
