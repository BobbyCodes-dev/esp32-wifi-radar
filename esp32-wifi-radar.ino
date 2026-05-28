// esp32-wifi-radar — WiFi Motion/Presence Detector for LilyGo T-Display (ESP32, 135x240)
// Libraries: TFT_eSPI by Bodmer
// In TFT_eSPI/User_Setup_Select.h: uncomment Setup25_TTGO_T_Display.h
//
// Approach: RSSI-based motion sensing
//   - Connects to your home WiFi router
//   - Samples RSSI every 100ms, keeps a 50-sample rolling buffer
//   - Calculates rolling mean + stddev; deviation > SENSITIVITY * stddev = motion event
//   - Display: top = live RSSI oscilloscope waveform, bottom = motion status + stats
//   - Serial: logs every motion event with millis() timestamp

#include <TFT_eSPI.h>
#include <WiFi.h>
#include <math.h>

// ============================================================
//  USER CONFIG — fill these in before flashing
// ============================================================
#define WIFI_SSID       "YOUR_SSID_HERE"
#define WIFI_PASSWORD   "YOUR_PASSWORD_HERE"

// Sensitivity: how many standard deviations from the rolling mean
// triggers a motion event. Lower = more sensitive (more false positives).
// Higher = less sensitive (may miss subtle movement).
//   1.5 = very sensitive (twitchy, good for detecting distant/subtle motion)
//   2.0 = balanced default (recommended starting point)
//   2.5 = less sensitive (only strong disturbances)
//   3.0 = high threshold (only dramatic movement near the router)
#define SENSITIVITY     2.0f

// ============================================================
//  Display / hardware
// ============================================================
#define W   240
#define H   135
#define BL_PIN  4

// ============================================================
//  Motion detection config
// ============================================================
#define SAMPLE_INTERVAL_MS   100    // sample RSSI every 100ms
#define BUFFER_SIZE          50     // rolling window of samples
#define MOTION_HOLD_MS       3000   // keep "MOTION" state visible for 3s after last trigger
#define MIN_STDDEV           0.5f   // ignore micro-noise below this stddev (flat signal)

// ============================================================
//  Display layout
// ============================================================
#define GRAPH_Y      0           // oscilloscope graph starts at top
#define GRAPH_H      72          // top 72 rows = waveform area
#define DIVIDER_Y    73
#define STATUS_Y     78          // bottom area starts here
#define GRAPH_W      W           // full width

// ============================================================
//  Colors
// ============================================================
#define COL_BG          TFT_BLACK
#define COL_GRID        0x1082    // very dark grey
#define COL_WAVE        0x07E0    // bright green (RSSI line)
#define COL_WAVE_OLD    0x0320    // dim green (older samples)
#define COL_CLEAR       0x07E0    // green text for CLEAR
#define COL_MOTION      TFT_RED
#define COL_TEXT        TFT_WHITE
#define COL_LABEL       0xAD75    // muted grey-blue for labels
#define COL_DIVIDER     0x2945
#define COL_MEAN        0x04BF    // cyan tint for mean line
#define COL_BAR_BG      0x2104    // dark bar background
#define COL_BAR_FG      TFT_GREEN

// ============================================================
//  Globals
// ============================================================
TFT_eSPI    tft;
TFT_eSprite graphSprite(&tft);
TFT_eSprite statusSprite(&tft);

int32_t  rssiBuffer[BUFFER_SIZE];
int      bufHead      = 0;       // next write index (circular)
int      bufCount     = 0;       // samples filled so far
float    rollingMean  = 0.0f;
float    rollingStddev = 0.0f;

uint32_t motionCount   = 0;
uint32_t lastMotionMs  = 0;      // millis() of last detected motion event
bool     inMotion      = false;
char     lastMotionStr[32] = "None";

uint32_t lastSampleMs  = 0;
uint32_t lastDrawMs    = 0;

// ============================================================
//  Rolling statistics
// ============================================================
void computeStats() {
  if (bufCount == 0) { rollingMean = 0; rollingStddev = 0; return; }

  float sum = 0;
  int   count = min(bufCount, BUFFER_SIZE);
  for (int i = 0; i < count; i++) sum += rssiBuffer[i];
  rollingMean = sum / count;

  float varSum = 0;
  for (int i = 0; i < count; i++) {
    float d = rssiBuffer[i] - rollingMean;
    varSum += d * d;
  }
  rollingStddev = sqrtf(varSum / count);
}

// ============================================================
//  WiFi connect (blocks until connected)
// ============================================================
void connectWiFi() {
  tft.fillScreen(COL_BG);
  tft.setTextColor(COL_TEXT, COL_BG);
  tft.setTextFont(2);
  tft.setCursor(8, 50);
  tft.print("Connecting to WiFi...");

  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  uint32_t start = millis();
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    tft.print(".");
    if (millis() - start > 20000) {
      tft.fillScreen(COL_BG);
      tft.setCursor(8, 50);
      tft.setTextColor(TFT_RED, COL_BG);
      tft.print("WiFi FAILED. Check creds.");
      while (1) delay(5000);
    }
  }

  Serial.printf("[BOOT] Connected to %s  IP: %s\n",
                WIFI_SSID, WiFi.localIP().toString().c_str());
}

// ============================================================
//  Sample RSSI and check for motion
// ============================================================
void takeSample() {
  int32_t rssi = WiFi.RSSI();

  // Store in circular buffer
  rssiBuffer[bufHead] = rssi;
  bufHead = (bufHead + 1) % BUFFER_SIZE;
  if (bufCount < BUFFER_SIZE) bufCount++;

  // Need at least 10 samples before declaring motion
  if (bufCount < 10) return;

  computeStats();

  // Only fire if there's meaningful variance (not a perfectly flat signal)
  if (rollingStddev < MIN_STDDEV) return;

  float deviation = fabsf((float)rssi - rollingMean);
  if (deviation > SENSITIVITY * rollingStddev) {
    uint32_t now = millis();

    // Debounce: don't log a new event if we just logged one < 1s ago
    if (now - lastMotionMs > 1000) {
      motionCount++;
      lastMotionMs = now;

      // Format timestamp as H:M:S since boot
      uint32_t secs   = now / 1000;
      uint32_t mins   = secs / 60;
      uint32_t hours  = mins / 60;
      snprintf(lastMotionStr, sizeof(lastMotionStr),
               "%02lu:%02lu:%02lu", hours, mins % 60, secs % 60);

      Serial.printf("[MOTION] Event #%lu at %s  RSSI=%d  mean=%.1f  stddev=%.2f  dev=%.1f\n",
                    motionCount, lastMotionStr, rssi,
                    rollingMean, rollingStddev, deviation);
    }
    inMotion = true;
  }

  // Clear motion state after hold period
  if (inMotion && (millis() - lastMotionMs > MOTION_HOLD_MS)) {
    inMotion = false;
  }
}

// ============================================================
//  Draw oscilloscope waveform (top half)
// ============================================================
void drawGraph() {
  graphSprite.fillSprite(COL_BG);

  // Grid lines
  for (int y = 0; y < GRAPH_H; y += GRAPH_H / 4) {
    graphSprite.drawFastHLine(0, y, GRAPH_W, COL_GRID);
  }
  graphSprite.drawFastHLine(0, GRAPH_H - 1, GRAPH_W, COL_GRID);

  // Draw the mean line
  if (bufCount >= 10) {
    // Map mean RSSI to Y pixel
    int meanY = map((int)rollingMean, -100, -30, GRAPH_H - 2, 2);
    meanY = constrain(meanY, 2, GRAPH_H - 2);
    graphSprite.drawFastHLine(0, meanY, GRAPH_W, COL_MEAN);
  }

  // Draw RSSI samples as a waveform — oldest on left, newest on right
  // We have up to BUFFER_SIZE samples. Map them across GRAPH_W pixels.
  int count = min(bufCount, BUFFER_SIZE);
  if (count < 2) { graphSprite.pushSprite(0, GRAPH_Y); return; }

  // Walk from oldest to newest
  // oldest sample index: if buffer is full, it's bufHead (next write = oldest)
  //                      if not full, it's 0
  int startIdx = (bufCount >= BUFFER_SIZE) ? bufHead : 0;

  int prevX = -1, prevY = -1;
  for (int i = 0; i < count; i++) {
    int idx  = (startIdx + i) % BUFFER_SIZE;
    int32_t rssi = rssiBuffer[idx];

    int px = map(i, 0, count - 1, 0, GRAPH_W - 1);
    int py = map((int)rssi, -100, -30, GRAPH_H - 2, 2);
    py = constrain(py, 2, GRAPH_H - 2);

    // Older samples dimmer, newer brighter
    uint16_t col = (i > count - 10) ? COL_WAVE : COL_WAVE_OLD;

    if (prevX >= 0) {
      graphSprite.drawLine(prevX, prevY, px, py, col);
    }
    prevX = px;
    prevY = py;
  }

  // Label: current RSSI in top-left
  graphSprite.setTextFont(1);
  graphSprite.setTextColor(COL_TEXT, COL_BG);
  graphSprite.setCursor(2, 2);
  if (bufCount > 0) {
    int32_t latest = rssiBuffer[(bufHead + BUFFER_SIZE - 1) % BUFFER_SIZE];
    graphSprite.printf("%d dBm", latest);
  }

  // Label: "RSSI" header on right
  graphSprite.setTextColor(COL_LABEL, COL_BG);
  graphSprite.setCursor(GRAPH_W - 36, 2);
  graphSprite.print("RSSI");

  graphSprite.pushSprite(0, GRAPH_Y);
}

// ============================================================
//  Draw status panel (bottom half)
// ============================================================
void drawStatus() {
  statusSprite.fillSprite(COL_BG);

  int panelW = GRAPH_W;

  // --- Motion status badge ---
  if (inMotion) {
    statusSprite.fillRoundRect(0, 0, 90, 22, 4, COL_MOTION);
    statusSprite.setTextFont(4);
    statusSprite.setTextColor(TFT_WHITE, COL_MOTION);
    statusSprite.setCursor(5, 3);
    statusSprite.print("MOTION");
  } else {
    statusSprite.fillRoundRect(0, 0, 68, 22, 4, 0x0340);
    statusSprite.setTextFont(4);
    statusSprite.setTextColor(COL_CLEAR, 0x0340);
    statusSprite.setCursor(5, 3);
    statusSprite.print("CLEAR");
  }

  // --- Motion confidence bar (right of badge) ---
  // Shows how close current RSSI is to triggering threshold
  float confidence = 0;
  if (bufCount >= 10 && rollingStddev >= MIN_STDDEV) {
    int32_t latest = rssiBuffer[(bufHead + BUFFER_SIZE - 1) % BUFFER_SIZE];
    float dev = fabsf((float)latest - rollingMean);
    confidence = dev / (SENSITIVITY * rollingStddev);  // 1.0 = at threshold
    confidence = constrain(confidence, 0.0f, 1.0f);
  }
  int barX   = 96;
  int barW   = panelW - barX - 2;
  int fillW  = (int)(barW * confidence);
  uint16_t barCol = (confidence > 0.85f) ? TFT_ORANGE : (confidence > 0.6f ? TFT_YELLOW : COL_BAR_FG);
  statusSprite.fillRoundRect(barX, 5, barW, 12, 2, COL_BAR_BG);
  if (fillW > 0) statusSprite.fillRoundRect(barX, 5, fillW, 12, 2, barCol);
  statusSprite.setTextFont(1);
  statusSprite.setTextColor(COL_LABEL, COL_BG);
  statusSprite.setCursor(barX, 20);
  statusSprite.print("sensitivity");

  // --- Stats row ---
  statusSprite.setTextFont(1);

  // Motion count
  statusSprite.setTextColor(COL_LABEL, COL_BG);
  statusSprite.setCursor(0, 28);
  statusSprite.print("Events:");
  statusSprite.setTextColor(COL_TEXT, COL_BG);
  statusSprite.printf(" %lu", motionCount);

  // Last event timestamp
  statusSprite.setTextColor(COL_LABEL, COL_BG);
  statusSprite.setCursor(0, 40);
  statusSprite.print("Last:   ");
  statusSprite.setTextColor(COL_TEXT, COL_BG);
  statusSprite.print(lastMotionStr);

  // Mean / stddev
  statusSprite.setTextColor(COL_LABEL, COL_BG);
  statusSprite.setCursor(0, 52);
  if (bufCount >= 2) {
    statusSprite.setTextColor(COL_LABEL, COL_BG);
    statusSprite.printf("mean %.1f  sd %.2f", rollingMean, rollingStddev);
  } else {
    statusSprite.setTextColor(COL_LABEL, COL_BG);
    statusSprite.print("Warming up...");
  }

  statusSprite.pushSprite(0, STATUS_Y);
}

// ============================================================
//  Setup
// ============================================================
void setup() {
  Serial.begin(115200);

  pinMode(BL_PIN, OUTPUT);
  digitalWrite(BL_PIN, HIGH);

  tft.init();
  tft.setRotation(1);  // landscape, USB on left
  tft.fillScreen(COL_BG);

  // Sprites
  graphSprite.createSprite(GRAPH_W, GRAPH_H);
  graphSprite.setTextFont(1);

  int statusH = H - STATUS_Y;
  statusSprite.createSprite(GRAPH_W, statusH);
  statusSprite.setTextFont(1);

  connectWiFi();

  tft.fillScreen(COL_BG);
  tft.drawFastHLine(0, DIVIDER_Y, W, COL_DIVIDER);

  Serial.printf("[BOOT] WiFi Motion Radar running. SSID=%s  Sensitivity=%.1f\n",
                WIFI_SSID, SENSITIVITY);
  Serial.println("[BOOT] Serial format: [MOTION] Event #N at HH:MM:SS  RSSI=X  mean=M  stddev=S  dev=D");
}

// ============================================================
//  Loop
// ============================================================
void loop() {
  uint32_t now = millis();

  // Sample RSSI on interval
  if (now - lastSampleMs >= SAMPLE_INTERVAL_MS) {
    lastSampleMs = now;
    if (WiFi.status() == WL_CONNECTED) {
      takeSample();
    } else {
      // Reconnect if dropped
      Serial.println("[WARN] WiFi disconnected, reconnecting...");
      WiFi.reconnect();
    }
  }

  // Redraw display every ~100ms (synced to sample rate)
  if (now - lastDrawMs >= 100) {
    lastDrawMs = now;
    tft.drawFastHLine(0, DIVIDER_Y, W, COL_DIVIDER);
    drawGraph();
    drawStatus();
  }
}
