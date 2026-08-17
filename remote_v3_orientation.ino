// bno08x.enableReport(SH2_ARVR_STABILIZED_RV, 2500); sets 400 Hz polling,
// downgrade to bno08x.enableReport(SH2_ARVR_STABILIZED_RV, 5000); if noticeable wireless
// or battery life issues
// static const uint8_t receiver_mac[] defines the MAC of the receiver dongle

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

// Unicast to the receiver rather than broadcast: unicast frames get MAC-level
// ACK and automatic retransmission, so a frame lost to a collision is retried
// instead of silently dropped. Must match the receiver's MAC.
static const uint8_t receiver_mac[] = { 0x64, 0xE8, 0x33, 0x86, 0xD6, 0xDC };

Adafruit_BNO08x bno08x(PIN_BNO_RST);
sh2_SensorValue_t sensorValue;

const int DATAGRAM_REPEATS = 10;
const int ACTION_DATAGRAMS = DATAGRAM_REPEATS + 1;  // original transmission plus repeats

// ---- Button handling ----
// Buttons are wired with internal pull-ups: LOW = pressed, HIGH = released.
const unsigned long BUTTON_DEBOUNCE_MS = 30;  // ignore edges faster than this (mechanical bounce)
const unsigned long SWITCH_DEBOUNCE_MS = 30;

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
uint32_t action_generation = 0;
int action_flag_successful_sends = 0;

// Track the transmit switch so we can fire calibration on the off->on edge.
// setup() initializes these from the physical switch so booting in either
// position never fires calibration; only a debounced off->on transition does.
int transmit_switch_state = LOW;
int raw_transmit_switch_state = LOW;
int prev_transmit_switch_state = LOW;
unsigned long transmit_switch_last_change_ms = 0;

// Report activation can fail transiently, especially after a sensor reset.
// Retry without blocking loop() so the transmit switch continues to be sampled.
bool orientation_report_enabled = false;
unsigned long orientation_report_retry_start_ms = 0;
unsigned long orientation_report_last_attempt_ms = 0;
unsigned long orientation_report_last_error_ms = 0;
const unsigned long REPORT_ENABLE_RETRY_MS = 100;

#if TRANSMIT_MODE == TRANSMIT_ESPNOW
// ESP-NOW send completion runs on the WiFi task. Keep only one packet in flight
// so callback results cannot be associated with the wrong action packet.
portMUX_TYPE esp_now_send_mux = portMUX_INITIALIZER_UNLOCKED;
volatile bool esp_now_send_in_flight = false;
volatile bool esp_now_send_result_ready = false;
volatile bool esp_now_last_send_succeeded = false;
uint8_t esp_now_in_flight_action_flag = 0;
uint32_t esp_now_in_flight_action_generation = 0;
#endif

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

bool enableOrientationReport() {
  return bno08x.enableReport(SH2_ARVR_STABILIZED_RV, 2500);
}

// Called from loop() after a BNO085 reset. Unlike setup-time recovery, this is
// non-blocking so switch transitions are still observed while the report is
// being restored.
bool serviceOrientationReportRecovery() {
  if (orientation_report_enabled) {
    return true;
  }

  unsigned long now = millis();
  if (orientation_report_last_attempt_ms != 0 &&
      now - orientation_report_last_attempt_ms < REPORT_ENABLE_RETRY_MS) {
    return false;
  }

  orientation_report_last_attempt_ms = now;
  if (enableOrientationReport()) {
    orientation_report_enabled = true;
    Serial.println("Orientation report enabled");
    return true;
  }

  if (now - orientation_report_retry_start_ms >= SETUP_ERROR_TIMEOUT_MS &&
      (orientation_report_last_error_ms == 0 ||
       now - orientation_report_last_error_ms >= SETUP_ERROR_TIMEOUT_MS)) {
    Serial.println("Failed to enable orientation report; retrying");
    orientation_report_last_error_ms = now;
  }
  return false;
}

int readTransmitSwitch() {
  int raw_state = digitalRead(PIN_SWITCH);
  if (raw_state != raw_transmit_switch_state) {
    raw_transmit_switch_state = raw_state;
    transmit_switch_last_change_ms = millis();
  }
  if (millis() - transmit_switch_last_change_ms >= SWITCH_DEBOUNCE_MS) {
    transmit_switch_state = raw_transmit_switch_state;
  }
  return transmit_switch_state;
}

void beginAction(uint8_t new_action_flag) {
  action_flag = new_action_flag;
  action_generation++;
  action_flag_successful_sends = 0;
}

void recordSuccessfulActionSend(uint8_t sent_action_flag, uint32_t sent_action_generation) {
  // A higher-priority action may have replaced the one that was in flight.
  if (sent_action_flag == 0 || sent_action_flag != action_flag ||
      sent_action_generation != action_generation) {
    return;
  }

  action_flag_successful_sends++;
  if (action_flag_successful_sends >= ACTION_DATAGRAMS) {
    action_flag = 0;
    action_flag_successful_sends = 0;
  }
}

#if TRANSMIT_MODE == TRANSMIT_ESPNOW
void onPacketSent(const esp_now_send_info_t* tx_info, esp_now_send_status_t status) {
  (void)tx_info;
  portENTER_CRITICAL(&esp_now_send_mux);
  esp_now_last_send_succeeded = (status == ESP_NOW_SEND_SUCCESS);
  esp_now_send_result_ready = true;
  esp_now_send_in_flight = false;
  portEXIT_CRITICAL(&esp_now_send_mux);
}

void processEspNowSendResult() {
  bool result_ready;
  bool succeeded = false;
  uint8_t sent_action_flag = 0;
  uint32_t sent_action_generation = 0;

  portENTER_CRITICAL(&esp_now_send_mux);
  result_ready = esp_now_send_result_ready;
  if (result_ready) {
    succeeded = esp_now_last_send_succeeded;
    sent_action_flag = esp_now_in_flight_action_flag;
    sent_action_generation = esp_now_in_flight_action_generation;
    esp_now_send_result_ready = false;
  }
  portEXIT_CRITICAL(&esp_now_send_mux);

  if (result_ready && succeeded) {
    recordSuccessfulActionSend(sent_action_flag, sent_action_generation);
  }
}
#endif

void setup(void) {
  Serial.begin(115200);

  // Configure pins first so the LED can signal errors and the buttons/switch
  // read correctly even if a later init step fails and we halt.
  pinMode(PIN_BUTTON1, INPUT_PULLUP);
  pinMode(PIN_BOOT_BTN, INPUT_PULLUP);
  pinMode(PIN_SWITCH, INPUT_PULLUP);
  pinMode(PIN_LED, OUTPUT);

  raw_transmit_switch_state = digitalRead(PIN_SWITCH);
  transmit_switch_state = raw_transmit_switch_state;
  prev_transmit_switch_state = transmit_switch_state;
  transmit_switch_last_change_ms = millis();

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
  unsigned long report_start = millis();
  while (!enableOrientationReport()) {
    if (millis() - report_start >= SETUP_ERROR_TIMEOUT_MS) {
      blinkErrorCode(2);  // sensor responds, but the orientation report could not be enabled
    } else {
      delay(10);
    }
  }
  orientation_report_enabled = true;

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

  if (esp_now_register_send_cb(onPacketSent) != ESP_OK) {
    Serial.println("Failed to register ESP-NOW send callback");
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

  // ESP-NOW otherwise transmits at 1 Mbps 802.11b, where our frames each burn
  // ~750 us of airtime and four remotes at 200 Hz saturate the channel. MCS0
  // (BPSK, 6.5/7.2 Mbps) cuts that to ~115 us while keeping the most
  // noise-tolerant modulation, i.e. the longest range. If channel occupancy
  // ever becomes the constraint again (more remotes / higher rates), switch to
  // phymode WIFI_PHY_MODE_11G with rate WIFI_PHY_RATE_24M: ~45 us per frame,
  // at the cost of ~5-7 dB of link margin (range).
  esp_now_rate_config_t rate_config = {};
  rate_config.phymode = WIFI_PHY_MODE_HT20;
  rate_config.rate = WIFI_PHY_RATE_MCS0_SGI;
  if (esp_now_set_peer_rate_config(receiver_mac, &rate_config) != ESP_OK) {
    // Not fatal: the link still works at the default 1 Mbps rate.
    Serial.println("Failed to set ESP-NOW PHY rate");
  }
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

bool sendPacket(const orientation_packet_t& packet) {
#if TRANSMIT_MODE == TRANSMIT_ESPNOW
  bool can_send = false;
  portENTER_CRITICAL(&esp_now_send_mux);
  if (!esp_now_send_in_flight && !esp_now_send_result_ready) {
    esp_now_send_in_flight = true;
    esp_now_in_flight_action_flag = packet.action_flag;
    esp_now_in_flight_action_generation = action_generation;
    can_send = true;
  }
  portEXIT_CRITICAL(&esp_now_send_mux);

  if (!can_send) {
    return false;
  }

  esp_err_t result =
    esp_now_send(receiver_mac, reinterpret_cast<const uint8_t*>(&packet), sizeof(packet));
  if (result != ESP_OK) {
    portENTER_CRITICAL(&esp_now_send_mux);
    esp_now_send_in_flight = false;
    portEXIT_CRITICAL(&esp_now_send_mux);
    return false;
  }
  return true;
#else
  if (!udp.beginPacket(UDP_SERVER_IP, UDP_SERVER_PORT)) {
    return false;
  }
  size_t bytes_written = udp.write(reinterpret_cast<const uint8_t*>(&packet), sizeof(packet));
  bool packet_sent = (udp.endPacket() == 1);
  return bytes_written == sizeof(packet) && packet_sent;
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
//   1 = button 1 clicked
//   2 = button 2 clicked
// Single clicks are emitted on *press*. (Calibration, flag 4, is no longer a
// button gesture -- it fires when the transmit switch is toggled on; see loop.)
void updateButtons() {
  bool b1 = readButton(button_1);
  bool b2 = readButton(button_2);

  // Only assign a new action while none is in flight: an in-flight flag is
  // repeated across several packets and must not be clobbered mid-send.
  if (action_flag == 0) {
    // Rising edge (pressed): emit the single-button click.
    if (!button_1_was_pressed && b1) {
      beginAction(1);
    }
    if (!button_2_was_pressed && b2) {
      beginAction(2);
    }
  }

  button_1_was_pressed = b1;
  button_2_was_pressed = b2;
}

void loop(void) {
#if TRANSMIT_MODE == TRANSMIT_ESPNOW
  processEspNowSendResult();
#endif

  // Poll and debounce the switch independently of sensor events so an IMU
  // outage cannot hide an off->on calibration transition.
  int current_transmit_switch_state = readTransmitSwitch();
  if (current_transmit_switch_state == LOW && prev_transmit_switch_state == HIGH) {
    beginAction(4);
  }
  prev_transmit_switch_state = current_transmit_switch_state;

  if (current_transmit_switch_state == HIGH) {
    digitalWrite(PIN_LED, LOW);
    led_state = false;
  }

  if (bno08x.wasReset()) {
    Serial.println("sensor was reset ");
    orientation_report_enabled = false;
    orientation_report_retry_start_ms = millis();
    orientation_report_last_attempt_ms = 0;
    orientation_report_last_error_ms = 0;
    digitalWrite(PIN_LED, LOW);
    led_state = false;
  }

  if (!serviceOrientationReportRecovery()) {
    return;
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

  if (current_transmit_switch_state == LOW) {
    // Actions:
    // 1: Button 1 clicked (see updateButtons)
    // 2: Button 2 clicked (see updateButtons)
    // 4: transmit switch toggled on (calibration; set on the switch edge above)
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
    packet.seq = packet_seq;
    packet.w = w;
    packet.x = x;
    packet.y = y;
    packet.z = z;
    packet.action_flag = action_flag;
    if (sendPacket(packet)) {
      packet_seq++;
#if TRANSMIT_MODE == TRANSMIT_UDP
      // UDP has no asynchronous MAC-level send callback; endPacket() success is
      // the strongest local confirmation available in this transport mode.
      recordSuccessfulActionSend(packet.action_flag, action_generation);
#endif
    }
  }
}
