// =====================================================================
//  iGreenhouse - ESP32 + WiFi + MQTT (điều khiển từ xa qua web)
//  Thư viện cần cài thêm: PubSubClient (Nick O'Leary)
// =====================================================================

#include <WiFi.h>
#include <PubSubClient.h>
#include <SPI.h>
#include <MFRC522.h>
#include <ESP32Servo.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SH110X.h>   // Màn OLED 1.3 inch dùng chip SH1106
#include <DHT.h>
#include <LiquidCrystal_I2C.h> // Màn LCD 16x2 I2C

// ================== CẤU HÌNH MẠNG (SỬA 4 DÒNG NÀY) ==================
const char* WIFI_SSID    = "ACTVN-TA1";
const char* WIFI_PASS    = "12345679";
const char* MQTT_HOST    = "broker.hivemq.com";   // broker công cộng, miễn phí
const uint16_t MQTT_PORT = 1883;                  // web dùng WebSocket 8884 (wss)
const char* TOPIC_PREFIX  = "igreenhouse/hwa-mlbvzm3eo5yi";
const char* DOOR_PIN_CODE = "1234";             // mã PIN khi mở cửa từ xa

// --- OLED 1.3 inch (SH1106, I2C, 128x64) ---
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_ADDR    0x3C      // nếu không lên hình, thử 0x3D
Adafruit_SH1106G display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

// --- LCD 16x2 I2C: hiển thị kết quả quét thẻ (địa chỉ 0x27, nếu không lên thử 0x3F) ---
LiquidCrystal_I2C lcd(0x27, 16, 2);

// --- Chân tín hiệu ---
#define LED_PIN      2    // Đèn báo nhiệt độ thấp
#define SS_PIN       5
#define RST_PIN      33

// SPI phần cứng ESP32
#define SPI_SCK      18
#define SPI_MISO     19
#define SPI_MOSI     23
#define SERVO_PIN    32
#define BUZZER_PIN   17
#define BUZZER_ACTIVE_LOW false
#define SOIL_PIN     34   // Cảm biến độ ẩm đất
#define DHTPIN       15
#define DHTTYPE      DHT11
#define FAN_PIN      27   // Relay IN2 điều khiển quạt
#define DOOR_RELAY_PIN 26 // Relay IN1: cấp nguồn cho servo cửa
#define RELAY_ACTIVE_LOW false

// --- Cảm biến mực nước bể của máy bơm (water sensor, ngõ ra analog) ---
// LƯU Ý: GPIO25 thuộc ADC2, ESP32 KHÔNG đọc analog được khi WiFi đang bật (luôn ra 0).
// Vì code này dùng WiFi nên dùng chân ADC1 (GPIO35, 36, 39). Cấp nguồn cảm biến 3.3V.
#define WATER_PIN        35     // chân tín hiệu (S) của water sensor
#define WATER_RAW_EMPTY  0      // raw khi bể cạn (cảm biến ngoài không khí)
#define WATER_RAW_FULL   2000   // raw khi nước đầy tới đỉnh cảm biến -> HIỆU CHUẨN LẠI theo Serial
// Cảm biến chỉ nằm ở 2/3 phía TRÊN của bể, không chạm đáy -> nó không thấy 1/3 nước phía dưới.
// Mức đo của cảm biến (0-100%) được quy đổi về mực nước thật của bể: 0% cảm biến = 33% bể, 100% = 100% bể.
#define WATER_BLIND_PCT  33     // % chiều cao bể nằm dưới đáy cảm biến (chỉnh theo bể thật)
#define WATER_LOW_ON     40     // mực nước bể <= 40% -> cảnh báo cấp thêm nước
#define WATER_LOW_OFF    50     // mực nước bể >= 50% -> tắt cảnh báo (trễ để khỏi nhấp nháy)

// --- Thời gian ---
#define RESULT_SHOW_MS 10000         // Hiển thị kết quả quét thẻ 10 giây
#define SENSOR_INTERVAL_MS 5000UL    // Đo cảm biến mỗi 5 giây
#define TELEMETRY_MS 5000UL          // Gửi telemetry lên web mỗi 5 giây

bool fanState = false;

// --- Hiệu chuẩn độ ẩm đất (thay bằng số đo thực của bạn) ---
#define SOIL_DRY     4095 // Giá trị raw khi khô (ngoài không khí)
#define SOIL_WET     1500 // Giá trị raw khi ướt (nhúng nước)

// --- Máy bơm qua module L298N ---
#define PUMP_ENA     13   // ENA (bỏ jumper ENA trên module)
#define PUMP_IN1     16   // IN1
#define PUMP_IN2     14   // IN2
#define PUMP_PWM_FREQ  5000
#define PUMP_PWM_RES   8    // 8 bit: duty 0-255
#define RUN_LIMIT_MS 60000  // Mỗi thiết bị chạy tối đa 60 giây (chế độ tự động & bơm)

// --- Ngưỡng điều khiển: nay là biến, chỉnh được từ web (lệnh cfg:...) ---
int tempLimit = 30;   // Nhiệt độ ngưỡng (độ C)
int soilOn    = 30;   // Độ ẩm đất < soilOn% thì bơm
int soilOff   = 40;   // Độ ẩm đất > soilOff% thì ngừng bơm
int pumpSpeed = 50;   // Công suất bơm (%), 20-100

bool pumpState = false;
bool pumpWant = false;
bool manualStop = false;   // true = dừng khẩn cấp, giữ quạt và bơm tắt

// Ghi đè từ web: -1 = tự động, 0 = ép tắt, 1 = ép bật
int ovLed = -1, ovFan = -1, ovPump = -1;

// Trạng thái giới hạn thời gian cho từng thiết bị
bool ledRunning = false, ledLocked = false;
bool fanRunning = false, fanLocked = false;
bool pumpRunning = false, pumpLocked = false;
unsigned long ledStart = 0, fanStart = 0, pumpStart = 0;

MFRC522 rfid(SS_PIN, RST_PIN);
bool rfidReady = false;

Servo doorServo;
DHT dht(DHTPIN, DHTTYPE);

// Mã thẻ hợp lệ
String validCardUID = "2E 71 B2 E3";
unsigned long lastSensorUpdate = 0;
bool ledState = false;

// --- Giá trị cảm biến gần nhất (để vẽ lại màn hình chính) ---
float lastTemp = NAN;
int lastMoisture = 0;
int soilRaw = 0;
bool sensorsReady = false;
int dhtFail = 0;          // số lần DHT11 đọc lỗi liên tiếp   // đã đọc cảm biến ít nhất một lần

// --- Mực nước bể bơm ---
int waterRaw = 0;
int waterPercent = 100;      // mặc định 100% để không báo sai trước lần đo đầu tiên
bool waterBlind = false;     // true = nước đã xuống dưới đáy cảm biến (chỉ biết "<= WATER_BLIND_PCT %")
bool waterLow = false;       // true = đang cảnh báo thiếu nước

// --- Nhiệt độ chip ESP32 (cảm biến nội bộ, chỉ mang tính tham khảo) ---
float cpuTemp = NAN;
#define CPU_TEMP_WARN 70.0   // °C: vượt mức này thì cảnh báo trên Serial

void readCpuTemp() {
  float t = temperatureRead();           // °C, cảm biến tích hợp trong chip
  cpuTemp = (t > -40 && t < 150) ? t : NAN;   // loại giá trị vô lý
}

// --- Hiển thị kết quả quét thẻ ---
bool showingResult = false;
unsigned long resultStart = 0;

// --- ƯU TIÊN THẺ TỪ ---
bool cardBusy = false;

// --- Cửa: máy trạng thái không chặn ---
enum DoorStage { DOOR_IDLE, DOOR_POWERUP, DOOR_OPEN, DOOR_CLOSING };
DoorStage doorStage = DOOR_IDLE;
unsigned long doorTimer = 0;
bool doorRemote = false;     // cửa đang mở do lệnh từ web

// --- MQTT ---
WiFiClient netClient;
PubSubClient mqtt(netClient);
String tTele, tStatus, tEvent, tCmd;

// ==========================================
// GIỚI HẠN THỜI GIAN CHẠY (RUN_LIMIT_MS)
// ==========================================
bool timedOutput(bool want, bool &running, bool &locked, unsigned long &startedAt) {
  if (!want) {
    running = false;
    locked = false;
    return false;
  }
  if (locked) return false;
  if (!running) {
    running = true;
    startedAt = millis();
  }
  if (millis() - startedAt >= RUN_LIMIT_MS) {
    running = false;
    locked = true;
    return false;
  }
  return true;
}

// ==========================================
// RC522: KHỞI TẠO MỘT LẦN
// ==========================================
bool rfidStart() {
  SPI.begin(SPI_SCK, SPI_MISO, SPI_MOSI, SS_PIN);

  pinMode(SS_PIN, OUTPUT);
  digitalWrite(SS_PIN, HIGH);

  delay(50);

  rfid.PCD_Init();
  delay(50);

  rfid.PCD_SetAntennaGain(MFRC522::RxGain_max);
  rfid.PCD_AntennaOn();

  delay(50);

  byte version = rfid.PCD_ReadRegister(MFRC522::VersionReg);

  Serial.print("RC522 Version: 0x");
  Serial.println(version, HEX);

  if (version == 0x00 || version == 0xFF) {
    Serial.println("LOI: Khong giao tiep duoc voi RC522!");
    return false;
  }

  return true;
}

void setup() {
  Serial.begin(115200);

  pinMode(BUZZER_PIN, OUTPUT);
  buzzerWrite(false);

  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  if (!display.begin(OLED_ADDR, true)) {
    Serial.println(F("Khong tim thay man hinh OLED"));
    for (;;);
  }

  lcd.init();
  lcd.backlight();
  updateLCD("He thong san", "sang doc the...");

  rfidReady = rfidStart();

  if (rfidReady) {
    Serial.println("RC522 OK");
  } else {
    Serial.println("RC522 FAILED - Kiem tra day/nguon RC522!");
  }

  Serial.println("Tap an RFID/NFC tag on the RFID-RC522 reader");

  pinMode(DOOR_RELAY_PIN, OUTPUT);
  setDoorPower(false);
  doorServo.setPeriodHertz(50);
  doorServo.attach(SERVO_PIN, 500, 2400);
  doorServo.write(90);

  dht.begin();

  digitalWrite(LED_PIN, HIGH);
  delay(500);
  digitalWrite(LED_PIN, LOW);

  pinMode(FAN_PIN, OUTPUT);
  setFan(false);

  pinMode(WATER_PIN, INPUT);   // water sensor (analog)

  ledcAttach(PUMP_ENA, PUMP_PWM_FREQ, PUMP_PWM_RES);
  pinMode(PUMP_IN1, OUTPUT);
  pinMode(PUMP_IN2, OUTPUT);
  setPump(false);

  // Lần đọc cảm biến đầu tiên sau ~2 giây (để web có dữ liệu sớm)
  lastSensorUpdate = millis() - SENSOR_INTERVAL_MS + 2000;

  drawMainScreen(lastTemp, lastMoisture, ledState, fanState);
  lcdIdle();

  // --- WiFi + MQTT (không chặn: kết nối ở loop) ---
  String p = String(TOPIC_PREFIX);
  tTele = p + "/telemetry";
  tStatus = p + "/status";
  tEvent = p + "/event";
  tCmd = p + "/cmd";
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  mqtt.setServer(MQTT_HOST, MQTT_PORT);
  mqtt.setBufferSize(640);
  mqtt.setSocketTimeout(2);   // mặc định 15 giây -> chặn quét thẻ khi mạng lỗi
  mqtt.setKeepAlive(15);
  mqtt.setCallback(onMqtt);

  Serial.println("He thong da san sang!");
}

// ==========================================
// KHÔI PHỤC RC522 SAU KHI CẢM BIẾN/RELAY HOẠT ĐỘNG
// ==========================================
void recoverRFID() {
  Serial.println("[RFID] Khoi phuc RC522...");

  digitalWrite(SS_PIN, HIGH);

  rfid.PCD_Init();
  delay(20);

  rfid.PCD_SetAntennaGain(MFRC522::RxGain_max);
  rfid.PCD_AntennaOn();

  delay(20);

  byte version = rfid.PCD_ReadRegister(MFRC522::VersionReg);

  Serial.print("[RFID] Version sau khoi phuc: 0x");
  Serial.println(version, HEX);

  if (version != 0x00 && version != 0xFF) {
    rfidReady = true;
    Serial.println("[RFID] RC522 san sang quet lai");
  } else {
    rfidReady = false;
    Serial.println("[RFID] RC522 van mat ket noi!");
  }
}

// ==========================================
// ĐỌC VÀ XỬ LÝ THẺ RFID
// ==========================================
bool scanRFID() {
  if (!rfidReady) {
    return false;
  }

  static unsigned long lastScan = 0;

  if (millis() - lastScan < 500) {
    return false;
  }

  if (!rfid.PICC_IsNewCardPresent()) {
    return false;
  }

  cardBusy = true;

  Serial.println("[RFID] Phat hien the!");

  if (!rfid.PICC_ReadCardSerial()) {
    Serial.println("[RFID] Phat hien the nhung khong doc duoc UID!");
    cardBusy = false;
    return false;
  }

  lastScan = millis();

  String scannedUID = "";

  for (byte i = 0; i < rfid.uid.size; i++) {
    if (rfid.uid.uidByte[i] < 0x10) {
      scannedUID += "0";
    }

    scannedUID += String(rfid.uid.uidByte[i], HEX);

    if (i < rfid.uid.size - 1) {
      scannedUID += " ";
    }
  }

  scannedUID.toUpperCase();

  Serial.print("[RFID] UID: ");
  Serial.println(scannedUID);

  showingResult = true;
  resultStart = millis();

  bool ok = (scannedUID == validCardUID);
  String cardEvt = ("{\"type\":\"card\",\"uid\":\"" + scannedUID + "\",\"ok\":" + (ok ? "true" : "false") + "}");

  if (ok) {
    Serial.println("[RFID] THE HOP LE!");
    updateLCD("THE HOP LE!", "Dang mo cua...");
    doorRemote = false;
    if (doorStage == DOOR_IDLE) {
      startDoor();
    } else if (doorStage == DOOR_OPEN) {
      doorTimer = millis();                 // quet lai khi cua dang mo -> gia han them 5 giay
    } else if (doorStage == DOOR_CLOSING) {
      doorServo.write(0);                   // dang dong -> mo lai ngay (servo van con nguon)
      doorStage = DOOR_OPEN;
      doorTimer = millis();
    }
    // DOOR_POWERUP: cua sap mo roi, khong can lam gi them
    beepBuzzer(1, 200);
  } else {
    Serial.println("[RFID] THE KHONG HOP LE!");
    updateLCD("KHONG HOP LE!", "Tu choi...");
    beepBuzzer(3, 150);
  }

  rfid.PICC_HaltA();
  rfid.PCD_StopCrypto1();

  publishEvent(cardEvt);   // gửi MQTT SAU khi đã mở cửa, tránh mạng chậm làm trễ cửa

  return true;
}

// ==========================================
// LOOP CHÍNH
// ==========================================
void loop() {

  // 0. MẠNG (không chặn)
  netLoop();

  // 1. SERIAL
  handleSerial();

  // 2. CẬP NHẬT CỬA + CÒI (không chặn)
  updateDoor();
  updateBuzzer();

  // 3. LCD tự quay về màn hình chờ khi hết thời gian hiển thị kết quả
  if (showingResult &&
      doorStage == DOOR_IDLE &&
      millis() - resultStart >= RESULT_SHOW_MS) {

    showingResult = false;
    lcdIdle();
  }

  // 4. KẾT THÚC XỬ LÝ THẺ -> nhả ưu tiên, 30s sau mới đo cảm biến
  if (cardBusy && !showingResult && doorStage == DOOR_IDLE) {
    cardBusy = false;
    lastSensorUpdate = millis();
    Serial.println("[RFID] Xu ly the xong -> tiep tuc do cam bien (30s/lan)");
  }

  // 5. RC522 ĐƯỢC ƯU TIÊN
  if (rfidReady) {
    scanRFID();
  }

  // 6. CẢM BIẾN ĐỌC MỖI 30 GIÂY (chỉ khi không bận xử lý thẻ)
  if (!cardBusy &&
      doorStage == DOOR_IDLE &&
      millis() - lastSensorUpdate >= SENSOR_INTERVAL_MS) {

    lastSensorUpdate = millis();

    Serial.println();
    Serial.println("================================");
    Serial.println("[CAM BIEN] Bat dau doc DHT + Soil");
    Serial.println("================================");

    updateSensors();

    drawMainScreen(lastTemp, lastMoisture, ledState, fanState);
    publishTelemetry();
    delay(5);
    recoverRFID();
  }

  // 6b. Mỗi giây áp lại logic thiết bị (để giới hạn 60s chính xác, và lệnh web có hiệu lực ngay)
  static unsigned long lastApply = 0;
  if (sensorsReady && millis() - lastApply >= 1000) {
    lastApply = millis();
    bool pl = ledState, pf = fanState, pp = pumpState;
    applyOutputs();
    if (pl != ledState || pf != fanState || pp != pumpState) {
      drawMainScreen(lastTemp, lastMoisture, ledState, fanState);
      publishTelemetry();
    }
  }

  // 7. TỰ KIỂM TRA RC522 ĐỊNH KỲ (bỏ qua khi đang xử lý thẻ)
  static unsigned long lastRFIDCheck = 0;

  if (!cardBusy && millis() - lastRFIDCheck >= 10000) {

    lastRFIDCheck = millis();

    byte version = rfid.PCD_ReadRegister(MFRC522::VersionReg);

    if (version == 0x00 || version == 0xFF) {

      Serial.println("[RFID] Mat giao tiep RC522!");

      rfidReady = false;

      recoverRFID();
    }
  }
}

// ==========================================
// MẠNG: WiFi + MQTT (không chặn loop quá lâu)
// ==========================================
void netLoop() {
  static unsigned long lastWifiTry = 0, lastMqttTry = 0, lastPub = 0, mqttRetryMs = 5000;
  static bool ipShown = false;

  if (WiFi.status() != WL_CONNECTED) {
    ipShown = false;
    if (millis() - lastWifiTry > 10000) {
      lastWifiTry = millis();
      Serial.println("[WIFI] Dang ket noi lai...");
      WiFi.disconnect();
      WiFi.begin(WIFI_SSID, WIFI_PASS);
    }
    return;
  }

  if (!ipShown) {   // in IP mỗi lần WiFi kết nối thành công
    ipShown = true;
    Serial.print("[WIFI] Da ket noi. IP ESP32: ");
    Serial.println(WiFi.localIP());
  }

  if (!mqtt.connected()) {
    if (!cardBusy && doorStage == DOOR_IDLE && millis() - lastMqttTry > mqttRetryMs) {   // không kết nối MQTT (có thể chặn vài giây) khi đang xử lý thẻ/cửa
      lastMqttTry = millis();
      String cid = "igh-" + String((uint32_t)ESP.getEfuseMac(), HEX);
      // Last Will: nếu mất kết nối đột ngột, broker tự gửi "offline"
      if (mqtt.connect(cid.c_str(), NULL, NULL, tStatus.c_str(), 1, true, "offline")) {
        Serial.println("[MQTT] Da ket noi broker");
        mqttRetryMs = 5000;
        mqtt.publish(tStatus.c_str(), "online", true);
        mqtt.subscribe(tCmd.c_str());
        publishTelemetry();
      } else {
        Serial.printf("[MQTT] Loi ket noi, rc=%d\n", mqtt.state());
        if (mqttRetryMs < 30000) mqttRetryMs *= 2;   // thử thưa dần khi broker không với tới
      }
    }
    return;
  }

  mqtt.loop();

  if (millis() - lastPub >= TELEMETRY_MS) {
    lastPub = millis();
    publishTelemetry();
  }
}

void publishTelemetry() {
  if (!mqtt.connected()) return;
  char tbuf[12];
  if (isnan(lastTemp)) strcpy(tbuf, "null");
  else dtostrf(lastTemp, 0, 1, tbuf);

  readCpuTemp();
  char cbuf[12];
  if (isnan(cpuTemp)) strcpy(cbuf, "null");
  else dtostrf(cpuTemp, 0, 1, cbuf);

  char buf[520];
  snprintf(buf, sizeof(buf),
    "{\"t\":%s,\"soil\":%d,\"raw\":%d,\"led\":%d,\"fan\":%d,\"pump\":%d,\"door\":%d,"
    "\"ovLed\":%d,\"ovFan\":%d,\"ovPump\":%d,\"stop\":%d,"
    "\"tl\":%d,\"so\":%d,\"sf\":%d,\"ps\":%d,\"rssi\":%d,\"up\":%lu,\"ok\":%d,\"water\":%d,\"wraw\":%d,\"wlow\":%d,\"wb\":%d,\"cpu\":%s}",
    tbuf, lastMoisture, soilRaw, ledState ? 1 : 0, fanState ? 1 : 0, pumpState ? 1 : 0, (int)doorStage,
    ovLed, ovFan, ovPump, manualStop ? 1 : 0,
    tempLimit, soilOn, soilOff, pumpSpeed, WiFi.RSSI(), millis() / 1000UL, sensorsReady ? 1 : 0,
    waterPercent, waterRaw, waterLow ? 1 : 0, waterBlind ? 1 : 0, cbuf);
  mqtt.publish(tTele.c_str(), buf, true);
}

void publishEvent(const String& json) {
  if (mqtt.connected()) mqtt.publish(tEvent.c_str(), json.c_str());
}

// ==========================================
// NHẬN LỆNH TỪ WEB (văn bản thuần, không cần thư viện JSON)
//   read | all:stop | all:run | all:auto
//   led:auto|1|0   fan:auto|1|0   pump:auto|1|0
//   door:open:<PIN>
//   cfg:temp=30  cfg:soilon=30  cfg:soiloff=40  cfg:pump=50
// ==========================================
void onMqtt(char* topic, byte* payload, unsigned int len) {
  String m;
  for (unsigned int i = 0; i < len && i < 64; i++) m += (char)payload[i];
  m.trim();
  handleCommand(m);
}

int parseOv(String v) {
  v.trim();
  if (v == "1") return 1;
  if (v == "0") return 0;
  return -1;   // "auto"
}

void handleCommand(String m) {
  Serial.print("[MQTT] Lenh: ");
  Serial.println(m.startsWith("door") ? String("door:open:***") : m);   // không in PIN ra Serial

  if (m == "read") {
    lastSensorUpdate = millis() - SENSOR_INTERVAL_MS;   // đo ngay ở vòng loop tiếp theo
    return;
  } else if (m == "all:stop") {
    manualStop = true;
    ovLed = -1; ovFan = -1; ovPump = -1;
    ledState = false;
    digitalWrite(LED_PIN, LOW);
    setFan(false);
    setPump(false);
  } else if (m == "all:run") {
    manualStop = false;
  } else if (m == "all:auto") {
    ovLed = ovFan = ovPump = -1;
    manualStop = false;
  } else if (m.startsWith("led:")) {
    ovLed = parseOv(m.substring(4));
  } else if (m.startsWith("fan:")) {
    ovFan = parseOv(m.substring(4));
  } else if (m.startsWith("pump:")) {
    ovPump = parseOv(m.substring(5));
    pumpLocked = false;   // cho phép chạy lại chu kỳ 60s mới
  } else if (m.startsWith("door:open")) {
    String pin = (m.length() > 10) ? m.substring(10) : "";
    if (pin == String(DOOR_PIN_CODE) && doorStage == DOOR_IDLE) {
      remoteOpenDoor();
    } else if (pin != String(DOOR_PIN_CODE)) {
      publishEvent("{\"type\":\"pin_denied\"}");
      beepBuzzer(2, 100);
    }
  } else if (m.startsWith("cfg:")) {
    int eq = m.indexOf('=');
    if (eq > 4) {
      String k = m.substring(4, eq);
      int v = m.substring(eq + 1).toInt();
      if (k == "temp" && v >= 10 && v <= 50) tempLimit = v;
      else if (k == "soilon" && v >= 0 && v <= 100) soilOn = v;
      else if (k == "soiloff" && v >= 0 && v <= 100) soilOff = v;
      else if (k == "pump" && v >= 20 && v <= 100) pumpSpeed = v;
    }
  }

  if (sensorsReady) applyOutputs();
  drawMainScreen(lastTemp, lastMoisture, ledState, fanState);
  publishTelemetry();
}

void remoteOpenDoor() {
  cardBusy = true;        // ưu tiên cửa, tạm dừng đo cảm biến
  doorRemote = true;
  showingResult = true;
  resultStart = millis();
  updateLCD("MO CUA TU XA", "Dang mo cua...");
  startDoor();
  beepBuzzer(1, 200);
}

// ==========================================
// RELAY IN1: CẤP NGUỒN SERVO CỬA
// ==========================================
void setDoorPower(bool on) {
  bool level = RELAY_ACTIVE_LOW ? !on : on;
  digitalWrite(DOOR_RELAY_PIN, level ? HIGH : LOW);
}

// ==========================================
// CỬA (KHÔNG DÙNG delay)
// Cấp nguồn -> 300ms -> mở -> 5s -> khóa -> 1s -> ngắt nguồn
// ==========================================
void startDoor() {
  setDoorPower(true);
  doorStage = DOOR_POWERUP;
  doorTimer = millis();
}

void updateDoor() {
  switch (doorStage) {
    case DOOR_IDLE:
      break;
    case DOOR_POWERUP:
      if (millis() - doorTimer >= 300) {
        doorServo.write(0);
        Serial.println("[CUA] Dang mo cua");
        doorStage = DOOR_OPEN;
        doorTimer = millis();
        publishEvent(String("{\"type\":\"door\",\"state\":\"open\",\"by\":\"") + (doorRemote ? "remote" : "card") + "\"}");
        publishTelemetry();
      }
      break;
    case DOOR_OPEN:
      if (millis() - doorTimer >= 5000) {
        doorServo.write(90);
        doorStage = DOOR_CLOSING;
        doorTimer = millis();
        publishTelemetry();
      }
      break;
    case DOOR_CLOSING:
      if (millis() - doorTimer >= 1000) {
        setDoorPower(false);
        Serial.println("[CUA] Da khoa cua");
        doorStage = DOOR_IDLE;

        updateLCD(doorRemote ? "MO CUA TU XA" : "THE HOP LE!", "Da khoa cua");
        showingResult = true;
        resultStart = millis();
        publishEvent("{\"type\":\"door\",\"state\":\"closed\"}");
        publishTelemetry();
      }
      break;
  }
}

// ==========================================
// LỆNH TỪ SERIAL MONITOR: '0' = dừng quạt + bơm, '1' = cho chạy lại
// ==========================================
void handleSerial() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '0') {
      manualStop = true;
      setFan(false);
      setPump(false);
      Serial.println(">> DUNG quat va bom");
    } else if (c == '1') {
      manualStop = false;
      Serial.println(">> Cho phep quat va bom chay lai");
    }
  }
}

// ==========================================
// QUẠT (RELAY IN2 - GPIO27)
// ==========================================
void setFan(bool on) {
  fanState = on;
  bool level = RELAY_ACTIVE_LOW ? !on : on;
  digitalWrite(FAN_PIN, level ? HIGH : LOW);
}

// Nhiệt độ > tempLimit thì bật, xuống dưới tempLimit-1 thì tắt (trễ 1 độ)
bool fanWant = false;
void updateFan(float temp) {
  if (isnan(temp)) {
    fanWant = false;
  } else if (temp > tempLimit) {
    fanWant = true;
  } else if (temp < tempLimit - 1) {
    fanWant = false;
  }
  if (manualStop) {
    setFan(timedOutput(false, fanRunning, fanLocked, fanStart));
  } else if (ovFan >= 0) {          // ghi đè từ web: không giới hạn 60s
    fanRunning = false; fanLocked = false;
    setFan(ovFan == 1);
  } else {
    setFan(timedOutput(fanWant, fanRunning, fanLocked, fanStart));
  }
}

bool ledWant = false;
void updateTempLED(float temp) {
  if (isnan(temp)) {
    ledWant = false;
  } else if (temp >= tempLimit) {
    ledWant = false;
  } else if (temp < tempLimit - 1) {
    ledWant = true;
  }
  if (manualStop) {                 // dừng khẩn cấp: tắt cả đèn
    ledRunning = false; ledLocked = false;
    ledState = false;
  } else if (ovLed >= 0) {
    ledRunning = false; ledLocked = false;
    ledState = (ovLed == 1);
  } else {
    ledState = timedOutput(ledWant, ledRunning, ledLocked, ledStart);
  }
  digitalWrite(LED_PIN, ledState ? HIGH : LOW);
}

// Áp lại toàn bộ logic thiết bị với số liệu gần nhất
void applyOutputs() {
  updateTempLED(lastTemp);
  updateFan(lastTemp);
  updatePump(lastMoisture);
}

// ==========================================
// ĐỌC CẢM BIẾN + ĐIỀU KHIỂN THIẾT BỊ + IN SERIAL (không vẽ OLED)
// ==========================================
void updateSensors() {
  lastMoisture = getSoilMoisture();
  float t = dht.readTemperature();
  if (!isnan(t)) {
    lastTemp = t;
    dhtFail = 0;
  } else if (++dhtFail < 5) {
    // DHT11 hay lỗi ngay sau khi cấp nguồn/khi WiFi bận: giữ số cũ, thử lại sau 2 giây
    Serial.println("[DHT] Doc loi, thu lai sau 2 giay");
    lastSensorUpdate = millis() - SENSOR_INTERVAL_MS + 2000;
  } else {
    lastTemp = NAN;   // lỗi 5 lần liên tiếp: báo mất cảm biến
  }
  updateWater();
  sensorsReady = true;

  applyOutputs();

  Serial.printf("[CAM BIEN] Nhiet do: %.1f C | Do am dat: %d%% (raw %d) | Den: %s | Quat: %s | Bom: %s\n",
                lastTemp, lastMoisture, soilRaw,
                ledState ? "ON" : "OFF",
                fanState ? "ON" : "OFF",
                pumpState ? "ON" : "OFF");
  readCpuTemp();
  Serial.printf("[CPU] Nhiet do chip: %.1f C%s\n", cpuTemp,
                (!isnan(cpuTemp) && cpuTemp >= CPU_TEMP_WARN) ? "  <-- QUA NONG!" : "");
  Serial.printf("[NUOC] Muc nuoc be: %s%d%% (raw %d)%s\n", waterBlind ? "<=" : "", waterPercent, waterRaw,
                waterLow ? "  <-- THIEU NUOC, CAN CAP THEM" : "");
}

// ==========================================
// Ô TRẠNG THÁI (Đèn / Quạt / Bơm): ON = ô đặc, OFF = ô viền
// ==========================================
void drawStatusBox(int x, const char* label, bool on) {
  if (on) {
    display.fillRect(x, 46, 40, 18, SH110X_WHITE);
    display.setTextColor(SH110X_BLACK);
  } else {
    display.drawRect(x, 46, 40, 18, SH110X_WHITE);
    display.setTextColor(SH110X_WHITE);
  }
  display.setTextSize(1);
  display.setCursor(x + 4, 48);
  display.print(label);
  display.setCursor(x + 4, 56);
  display.print(on ? "ON" : "OFF");
  display.setTextColor(SH110X_WHITE);
}

// ==========================================
// MÀN HÌNH CHÍNH: nhiệt độ, độ ẩm đất, đèn, quạt, máy bơm
// ==========================================
void drawMainScreen(float temp, int moisture, bool led, bool fan) {
  display.clearDisplay();
  display.setTextColor(SH110X_WHITE);

  display.setTextSize(1);
  if (waterLow) {
    // Thiếu nước: thay tiêu đề bằng thanh cảnh báo (nền trắng, chữ đen)
    display.fillRect(0, 0, SCREEN_WIDTH, 10, SH110X_WHITE);
    display.setTextColor(SH110X_BLACK);
    display.setCursor(4, 1);
    display.print("CAN CAP NUOC ");
    display.print(waterPercent);
    display.print("%");
    display.setTextColor(SH110X_WHITE);
  } else {
    display.setCursor((SCREEN_WIDTH - 66) / 2, 0);
    display.print("iGreenhouse");
  }

  display.setTextSize(1);
  display.setCursor(0, 15);
  display.print("Nhiet do");
  display.setTextSize(2);
  display.setCursor(56, 11);
  if (isnan(temp)) {
    display.print("--.-");
  } else {
    display.print(temp, 1);
  }
  display.setTextSize(1);
  display.write(247);
  display.print("C");

  display.setTextSize(1);
  display.setCursor(0, 33);
  display.print("Do am");
  display.setTextSize(2);
  display.setCursor(56, 29);
  display.print(moisture);
  display.setTextSize(1);
  display.print("%");

  drawStatusBox(0,  "Den",  led);
  drawStatusBox(44, "Quat", fan);
  drawStatusBox(88, "Bom",  pumpState);

  display.display();
}

// ==========================================
// ĐỌC ĐỘ ẨM ĐẤT
// ==========================================
int getSoilMoisture() {
  soilRaw = analogRead(SOIL_PIN);
  int percent = map(soilRaw, SOIL_DRY, SOIL_WET, 0, 100);
  return constrain(percent, 0, 100);
}

// ==========================================
// MỰC NƯỚC BỂ BƠM (water sensor, GPIO WATER_PIN)
// ==========================================
// Đọc trung bình 8 mẫu cho đỡ nhiễu -> trả về % (0-100)
int getWaterLevel() {
  long sum = 0;
  for (int i = 0; i < 8; i++) {
    sum += analogRead(WATER_PIN);
    delayMicroseconds(200);
  }
  waterRaw = sum / 8;
  int sensorPct = constrain(map(waterRaw, WATER_RAW_EMPTY, WATER_RAW_FULL, 0, 100), 0, 100);
  waterBlind = (sensorPct <= 2);   // cảm biến khô: nước nằm dưới tầm đo
  return WATER_BLIND_PCT + sensorPct * (100 - WATER_BLIND_PCT) / 100;   // quy đổi về mực nước thật của bể
}


// Đo mực nước + cảnh báo: <= 30% báo thiếu, >= 40% mới hết báo
void updateWater() {
  waterPercent = getWaterLevel();
  bool was = waterLow;

  if (waterPercent <= WATER_LOW_ON) {
    waterLow = true;
  } else if (waterPercent >= WATER_LOW_OFF) {
    waterLow = false;
  }

  if (waterLow && !was) {
    Serial.printf("[NUOC] CANH BAO: muc nuoc %d%% - CAN CAP THEM NUOC!\n", waterPercent);
    publishEvent(String("{\"type\":\"water\",\"low\":true,\"pct\":") + waterPercent + "}");
  } else if (!waterLow && was) {
    Serial.printf("[NUOC] Da cap nuoc, muc nuoc %d%%\n", waterPercent);
    publishEvent(String("{\"type\":\"water\",\"low\":false,\"pct\":") + waterPercent + "}");
  }
}

// ==========================================
// MÁY BƠM (L298N: ENA, IN1, IN2)
// ==========================================
void setPump(bool on) {
  pumpState = on;
  digitalWrite(PUMP_IN1, on ? HIGH : LOW);
  digitalWrite(PUMP_IN2, LOW);
  ledcWrite(PUMP_ENA, on ? (255 * pumpSpeed / 100) : 0);
}

// Đất khô thì muốn bơm (có độ trễ). Bơm luôn bị giới hạn RUN_LIMIT_MS,
// kể cả khi ép bật từ web (an toàn chống tràn nước).
void updatePump(int moisture) {
  if (moisture < soilOn) {
    pumpWant = true;
  } else if (moisture > soilOff) {
    pumpWant = false;
  }
  bool want = manualStop ? false : (ovPump >= 0 ? (ovPump == 1) : pumpWant);
  setPump(timedOutput(want, pumpRunning, pumpLocked, pumpStart));
}

// ==========================================
// LCD 16x2: kết quả quét thẻ (mỗi dòng tối đa 16 ký tự)
// ==========================================
void updateLCD(String line1, String line2) {
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print(line1);
  lcd.setCursor(0, 1);
  lcd.print(line2);
}

void lcdIdle() {
  updateLCD("Moi quet the...", "");
}

// ==========================================
// CÒI
// ==========================================
void buzzerWrite(bool on) {
  bool level = BUZZER_ACTIVE_LOW ? !on : on;
  digitalWrite(BUZZER_PIN, level ? HIGH : LOW);
}

// Còi không chặn: beepBuzzer() chỉ đặt lịch, updateBuzzer() (gọi trong loop) điều khiển còi
int buzzTimes = 0;
int buzzDur = 0;
bool buzzOn = false;
unsigned long buzzTimer = 0;

void beepBuzzer(int times, int duration) {
  buzzTimes = times;
  buzzDur = duration;
  buzzOn = true;
  buzzTimer = millis();
  buzzerWrite(true);
}

void updateBuzzer() {
  if (buzzTimes <= 0) return;
  unsigned long now = millis();
  if (buzzOn) {
    if (now - buzzTimer >= (unsigned long)buzzDur) {
      buzzerWrite(false);
      buzzOn = false;
      buzzTimer = now;
      buzzTimes--;
    }
  } else if (now - buzzTimer >= 100) {   // nghỉ 100ms giữa các tiếng bíp
    buzzerWrite(true);
    buzzOn = true;
    buzzTimer = now;
  }
}
