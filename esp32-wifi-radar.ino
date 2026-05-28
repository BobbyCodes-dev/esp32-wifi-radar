// esp32-wifi-radar — WiFi Radar for LilyGo T-Display (ESP32, 135x240)
// Libraries: TFT_eSPI by Bodmer
// In TFT_eSPI/User_Setup_Select.h: uncomment Setup25_TTGO_T_Display.h
//
// Features:
//   - Scans nearby WiFi (SSID, RSSI, channel, encryption)
//   - Radar sweep animation with blips plotted by signal strength
//   - Color-coded blips: green=strong, yellow=medium, red=weak
//   - Sidebar cycles through detected networks with SSID + RSSI
//   - Scan refreshes every 5 seconds

#include <TFT_eSPI.h>
#include <WiFi.h>
#include <math.h>

// --- Display dimensions (T-Display landscape) ---
#define W   240
#define H   135

// --- Radar area (left square region) ---
#define RADAR_CX   67          // center x of radar circle
#define RADAR_CY   67          // center y of radar circle
#define RADAR_R    62          // radar radius

// --- Sidebar ---
#define SIDEBAR_X  137         // sidebar starts here
#define SIDEBAR_W  (W - SIDEBAR_X - 2)

// --- Colors ---
#define COL_BG        TFT_BLACK
#define COL_GRID      0x0841   // very dark green
#define COL_SWEEP     0x07E0   // bright green
#define COL_SWEEP_DIM 0x0320   // dim trailing green
#define COL_STRONG    TFT_GREEN
#define COL_MED       TFT_YELLOW
#define COL_WEAK      TFT_RED
#define COL_TEXT      TFT_WHITE
#define COL_TITLE     0x07E0
#define COL_DIVIDER   0x2945

// --- Sweep config ---
#define SWEEP_STEP_DEG   3      // degrees per frame
#define SCAN_INTERVAL_MS 5000   // rescan every 5s
#define MAX_NETWORKS     20
#define TRAIL_STEPS      8      // number of trailing sweep lines

// --- Blip fade config ---
#define BLIP_TTL_TICKS   120   // how long a blip stays bright after being hit

// --- Backlight ---
#define BL_PIN  4

TFT_eSPI    tft;
TFT_eSprite radar(&tft);   // sprite for radar area
TFT_eSprite sidebar(&tft); // sprite for sidebar

// --- Network data ---
struct Network {
  char    ssid[33];
  int32_t rssi;
  uint8_t channel;
  wifi_auth_mode_t enc;
  float   angle;    // assigned radar angle (degrees)
  float   dist;     // 0.0–1.0 mapped from RSSI
  int     ttl;      // ticks until blip fades
};

Network nets[MAX_NETWORKS];
int     netCount = 0;

// --- Sweep state ---
float   sweepAngle  = 0;
int     sidebarIdx  = 0;
uint32_t lastScan   = 0;
uint32_t lastFrame  = 0;
uint32_t lastSidebar = 0;

// --- Helpers ---
float rssiToDist(int32_t rssi) {
  // RSSI typically -30 (very strong) to -90 (very weak)
  // Map to 0.15 (near center) to 0.92 (near edge)
  float clamped = constrain((float)rssi, -90.0f, -30.0f);
  return 0.15f + ((clamped + 30.0f) / -60.0f) * 0.77f;
  // -30 → 0.15 (center), -90 → 0.92 (edge)
}

uint16_t rssiColor(int32_t rssi) {
  if (rssi >= -60) return COL_STRONG;
  if (rssi >= -75) return COL_MED;
  return COL_WEAK;
}

const char* encStr(wifi_auth_mode_t enc) {
  switch (enc) {
    case WIFI_AUTH_OPEN:          return "OPEN";
    case WIFI_AUTH_WEP:           return "WEP";
    case WIFI_AUTH_WPA_PSK:       return "WPA";
    case WIFI_AUTH_WPA2_PSK:      return "WPA2";
    case WIFI_AUTH_WPA_WPA2_PSK:  return "WPA/2";
    case WIFI_AUTH_WPA2_ENTERPRISE: return "ENT";
    case WIFI_AUTH_WPA3_PSK:      return "WPA3";
    default:                       return "????";
  }
}

// --- Scan WiFi ---
void doScan() {
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  int found = WiFi.scanNetworks(false, true); // blocking, show hidden
  if (found < 0) found = 0;
  if (found > MAX_NETWORKS) found = MAX_NETWORKS;

  // Preserve blip TTL if same BSSID re-appears (match by SSID for simplicity)
  // Build new list
  Network newNets[MAX_NETWORKS];
  int newCount = 0;

  for (int i = 0; i < found; i++) {
    Network& n = newNets[newCount++];
    strncpy(n.ssid, WiFi.SSID(i).c_str(), 32);
    n.ssid[32] = '\0';
    n.rssi    = WiFi.RSSI(i);
    n.channel = WiFi.channel(i);
    n.enc     = WiFi.encryptionType(i);
    n.dist    = rssiToDist(n.rssi);

    // Try to keep angle from previous scan so blips don't jump
    bool found_prev = false;
    for (int j = 0; j < netCount; j++) {
      if (strcmp(nets[j].ssid, n.ssid) == 0) {
        n.angle = nets[j].angle;
        n.ttl   = nets[j].ttl;
        found_prev = true;
        break;
      }
    }
    if (!found_prev) {
      // Assign deterministic angle based on SSID hash so it doesn't jump on rescan
      uint32_t h = 5381;
      for (int c = 0; n.ssid[c]; c++) h = ((h << 5) + h) + n.ssid[c];
      n.angle = (float)(h % 360);
      n.ttl   = 0;
    }
  }

  memcpy(nets, newNets, sizeof(Network) * newCount);
  netCount = newCount;

  WiFi.scanDelete();
  if (sidebarIdx >= netCount && netCount > 0) sidebarIdx = 0;
}

// --- Draw radar background into sprite ---
void drawRadarBg() {
  radar.fillSprite(COL_BG);

  // Concentric circles
  for (int r = RADAR_R / 4; r <= RADAR_R; r += RADAR_R / 4) {
    radar.drawCircle(RADAR_CX, RADAR_CY, r, COL_GRID);
  }
  // Cross-hairs
  radar.drawFastHLine(RADAR_CX - RADAR_R, RADAR_CY, RADAR_R * 2, COL_GRID);
  radar.drawFastVLine(RADAR_CX, RADAR_CY - RADAR_R, RADAR_R * 2, COL_GRID);
  // Outer ring
  radar.drawCircle(RADAR_CX, RADAR_CY, RADAR_R, COL_SWEEP_DIM);

  // Title above radar (in main tft, not sprite — draw once during init)
}

// --- Draw a single sweep line into the radar sprite ---
void drawSweepLine(float angleDeg, uint16_t color) {
  float rad = angleDeg * DEG_TO_RAD;
  int x2 = RADAR_CX + (int)(RADAR_R * cos(rad));
  int y2 = RADAR_CY + (int)(RADAR_R * sin(rad));
  radar.drawLine(RADAR_CX, RADAR_CY, x2, y2, color);
}

// --- Draw blips ---
void drawBlips() {
  for (int i = 0; i < netCount; i++) {
    Network& n = nets[i];
    if (n.ttl <= 0) continue;

    float rad  = n.angle * DEG_TO_RAD;
    int   bx   = RADAR_CX + (int)(RADAR_R * n.dist * cos(rad));
    int   by   = RADAR_CY + (int)(RADAR_R * n.dist * sin(rad));
    uint16_t col = rssiColor(n.rssi);

    // Fade based on TTL
    uint8_t alpha = (n.ttl > BLIP_TTL_TICKS / 2) ? 255 : 128;
    (void)alpha; // TFT_eSPI doesn't have alpha blend; use full color

    radar.fillCircle(bx, by, 3, col);
    radar.drawCircle(bx, by, 4, col);
  }
}

// --- Draw sidebar ---
void drawSidebar() {
  sidebar.fillSprite(COL_BG);

  // Header
  sidebar.setTextColor(COL_TITLE, COL_BG);
  sidebar.setTextSize(1);
  sidebar.setCursor(2, 2);
  sidebar.print("WiFi Radar");

  sidebar.drawFastHLine(0, 12, SIDEBAR_W, COL_DIVIDER);

  // Network count
  sidebar.setTextColor(COL_TEXT, COL_BG);
  sidebar.setCursor(2, 15);
  sidebar.printf("%d nets found", netCount);
  sidebar.drawFastHLine(0, 25, SIDEBAR_W, COL_DIVIDER);

  if (netCount == 0) {
    sidebar.setCursor(2, 30);
    sidebar.setTextColor(COL_WEAK, COL_BG);
    sidebar.print("Scanning...");
    sidebar.pushSprite(SIDEBAR_X, 0);
    return;
  }

  // Cycle through networks, show 4 at a time starting from sidebarIdx
  int y = 30;
  for (int slot = 0; slot < 4 && slot < netCount; slot++) {
    int idx = (sidebarIdx + slot) % netCount;
    Network& n = nets[idx];

    // Highlight current
    uint16_t bg = (slot == 0) ? COL_GRID : COL_BG;
    sidebar.fillRect(0, y - 1, SIDEBAR_W, 26, bg);

    // SSID (truncate if too long)
    char trunc[14];
    strncpy(trunc, n.ssid, 13);
    trunc[13] = '\0';
    if (strlen(n.ssid) > 13) {
      trunc[11] = '.'; trunc[12] = '.'; trunc[13] = '\0';
    }

    sidebar.setTextColor(rssiColor(n.rssi), bg);
    sidebar.setTextSize(1);
    sidebar.setCursor(2, y);
    sidebar.print(trunc);

    // RSSI bar
    int barW = map(constrain(n.rssi, -90, -30), -90, -30, 1, SIDEBAR_W - 4);
    sidebar.fillRect(2, y + 10, barW, 4, rssiColor(n.rssi));
    sidebar.fillRect(2 + barW, y + 10, SIDEBAR_W - 4 - barW, 4, COL_DIVIDER);

    // RSSI + enc
    sidebar.setTextColor(COL_TEXT, bg);
    sidebar.setCursor(2, y + 16);
    sidebar.printf("%ddBm %s", n.rssi, encStr(n.enc));

    y += 26;
    if (y + 26 > H) break;
  }

  sidebar.pushSprite(SIDEBAR_X, 0);
}

// --- Vertical divider ---
void drawDivider() {
  tft.drawFastVLine(SIDEBAR_X - 1, 0, H, COL_DIVIDER);
}

// --- Setup ---
void setup() {
  Serial.begin(115200);

  pinMode(BL_PIN, OUTPUT);
  digitalWrite(BL_PIN, HIGH);

  tft.init();
  tft.setRotation(1);  // landscape, USB on left
  tft.fillScreen(COL_BG);

  // Init sprites
  radar.createSprite(RADAR_CX * 2 + 4, H);  // ~138 x 135
  radar.setTextFont(1);

  sidebar.createSprite(SIDEBAR_W, H);
  sidebar.setTextFont(1);

  drawDivider();

  // Initial scan
  doScan();
  lastScan = millis();
}

// --- Loop ---
void loop() {
  uint32_t now = millis();

  // --- Rescan every SCAN_INTERVAL_MS ---
  if (now - lastScan >= SCAN_INTERVAL_MS) {
    doScan();
    lastScan = now;
  }

  // --- Advance sweep angle ---
  sweepAngle += SWEEP_STEP_DEG;
  if (sweepAngle >= 360.0f) sweepAngle -= 360.0f;

  // --- Check blip hits (sweep passing over a blip) ---
  for (int i = 0; i < netCount; i++) {
    float diff = fabs(sweepAngle - nets[i].angle);
    if (diff > 180.0f) diff = 360.0f - diff;
    if (diff < (float)SWEEP_STEP_DEG * 2) {
      nets[i].ttl = BLIP_TTL_TICKS;
    }
    if (nets[i].ttl > 0) nets[i].ttl--;
  }

  // --- Draw radar frame ---
  drawRadarBg();

  // Trailing sweep (dim lines behind main sweep)
  for (int t = TRAIL_STEPS; t >= 1; t--) {
    float trailAngle = sweepAngle - t * SWEEP_STEP_DEG;
    if (trailAngle < 0) trailAngle += 360.0f;
    // Fade: darker for older trail
    uint8_t g = (uint8_t)(64 * (TRAIL_STEPS - t + 1) / TRAIL_STEPS);
    uint16_t trailCol = tft.color565(0, g, 0);
    drawSweepLine(trailAngle, trailCol);
  }
  // Main sweep line
  drawSweepLine(sweepAngle, COL_SWEEP);

  drawBlips();
  radar.pushSprite(0, 0);

  // --- Sidebar: cycle displayed network every 2s ---
  if (now - lastSidebar >= 2000 && netCount > 0) {
    sidebarIdx = (sidebarIdx + 1) % netCount;
    lastSidebar = now;
  }
  drawSidebar();

  // Small delay to control frame rate (~30fps target)
  delay(33);
}
