#include "ISISCommon.h"

// Own transport: sendButton()/sendEncoder() emit UDP uplink frames instead of
// MobiFlight connector commands (ISIS_FIRMWARE_SPEC.md section 5).
#include "net.h"
#include "proto.h"
#include "displaycfg.h"   // displaycfgApplyBrightness (Dimmen)

// The real Guition panel ...
LGFX lcdPanel;

// ... and the 480x480 composition canvas the whole drawing code calls "lcd".
// displaycfgComposite() blits it to the panel (1:1 or scaled).
IsisCanvas lcd(&lcdPanel);

// NVS handle for the CC_ISIS settings (version 7, units).
Preferences isisPrefs;

BrightnessMenu brightnessMenu;


// Set a new global power state. Returns false if it was already set, true if there was a change.

bool powerStateSet(PowerState ps)
{
    static PowerState lastPs = PowerState::INVALID;

    // Serial.printf("Set state: %d Control state: %d lastPs %d\n", (int)ps, (int)isisSettings.powerControl, lastPs);

    // No change? Short circuit.
    if (ps == lastPs) return false;

    if (isisSettings.powerControl == PowerControl::ALWAYS_ON) {
        ps = PowerState::POWER_ON;
    }

    // // if the lastPs was shutting down, we need to redraw the HSI.
    // if (lastPs == PowerState::SHUTTING_DOWN) {
    //     Serial.printf("refresh screen\n");
    //     lcd.fillScreen(TFT_BLACK);
    //     isisState.forceRedraw = true;
    // }

    if (ps == PowerState::POWER_OFF && isisSettings.powerControl != PowerControl::ALWAYS_ON) {
        // turn off the display.
        isisSetBacklightPwm(0);
    }

    if (ps == PowerState::POWER_ON) {
        // turn the display back on
        isisSetBacklightPwm(brightnessGamma(isisState.lcdBrightness));

//        Serial.printf("refresh screen 2\n");
        lcd.fillScreen(TFT_BLACK);
        isisState.forceRedraw = true;
    }

    // Start the shutdown timer if we're starting a shtudown.
    if (ps == PowerState::SHUTTING_DOWN) {
        isisState.shutdownStartMs = millis();
    }

    if (ps == PowerState::BATTERY_POWERED) {
        isisState.batteryStartMs = millis();
    }

    // default is just to set the new global state and move on.
    isisState.powerState = ps;
    lastPs             = ps;
    return true;
}

// This function is an exponential decay that allows changing values with a smooth motion
// Desired response time	α (approx)	Feel (at 20hz)
// 0.2 s (snappy pointer)	0.75	very responsive
// 0.5 s (analog needle)	0.3	smooth, analog feel
// 1.0 s (heavy needle)	0.15	sluggish but realistic
// 2.0 s (slow gauge)	0.075	big thermal or pressure gauge feel
// The snapThreshold is the same unit/magnitude as the input. At a minimum it should be the smallest changable value.
int smoothInput(int input, int current, float alpha, int threshold = 1)
{
    int diff = input - current;
    if (abs(diff) <= threshold) return input;

    int update = (int)(alpha * diff);
    if (update == 0 && abs(diff) > threshold) update = (diff > 0) ? 1 : -1;
    return current + update;
}

float smoothInput(float input, float current, float alpha, float snapThreshold = 0.5f)
{
    float diff   = input - current;
    float update = alpha * diff;

    // When update becomes smaller than snapThreshold * diff, snap to target
    // if (abs(update) < abs(diff) * snapThreshold) {
    if (fabs(update) < snapThreshold) {
        return input;
    }
    return current + update;
}

// Smooth a floating point directional value
float smoothDirection(float input, float current, float alpha, float threshold = 0.5f)
{
    // Handle angle wrapping (crossing 0°/360°)
    float diff = input - current;
    if (diff > 180.0f) diff -= 360.0f;
    if (diff < -180.0f) diff += 360.0f;

    // Snap to target when close enough
    if (abs(diff) < threshold) {
        return input;
    }

    float result = current + alpha * diff;

    // Normalize to 0-360
    if (result < 0.0f) result += 360.0f;
    if (result >= 360.0f) result -= 360.0f;

    return result;
}

float smoothAngle(float input, float current, float alpha, float threshold = 0.5f)
{
    // Handle angle wrapping for -180 to +180 range (for bank angle, etc.)
    float diff = input - current;
    if (diff > 180.0f) diff -= 360.0f;
    if (diff < -180.0f) diff += 360.0f;

    // Snap to target when close enough
    if (abs(diff) < threshold) {
        return input;
    }

    float result = current + alpha * diff;

    // Normalize to -180 to +180
    while (result > 180.0f)
        result -= 360.0f;
    while (result < -180.0f)
        result += 360.0f;

    return result;
}

CC_ISIS_Settings isisSettings;
ISISState      isisState;

LGFX_Sprite batterySprite;

void drawBattery(LGFX_Sprite *targetSprite, int x, int y)
{

    if (isisState.powerState != PowerState::BATTERY_POWERED ) return;
    // const long int batLifeSec = 3 * 60 * 60;  // 3 hours
    const long int batLifeSec = 3 * 60 * 60; // 3 hours
    long int       secOnBat   = (int)(millis() - isisState.batteryStartMs) / 1000;

    long int batPct = (int)(100 * (batLifeSec - secOnBat)) / batLifeSec;

    if (batterySprite.bufferLength() == 0) {
        batterySprite.setColorDepth(8);
        batterySprite.createSprite(100, 40);
        batterySprite.loadFont(PrimaSans32);
        batterySprite.setTextSize(0.5);
        batterySprite.setTextDatum(CL_DATUM);
        batterySprite.setTextColor(TFT_WHITE);
    }

    batterySprite.fillSprite(TFT_BLACK);

    if (batPct <= 0) return;

    batterySprite.drawRect(0, 0, batterySprite.width(), batterySprite.height(), TFT_LIGHTGRAY);
    batterySprite.drawRect(1, 1, batterySprite.width() - 2, batterySprite.height() - 2, TFT_LIGHTGRAY);


    char buf[7];
    sprintf(buf, "%d%%", batPct);
    batterySprite.drawString(buf, 5, 20);

    int bX = 55, bY = 10;
    batterySprite.drawBitmap(bX, bY, BATTERY_IMG_DATA, BATTERY_IMG_WIDTH, BATTERY_IMG_HEIGHT, TFT_LIGHTGRAY);

    if (batPct > 85) {
        batterySprite.fillRect(bX + 3, bY + 3, 6, 10, TFT_GREEN);
        batterySprite.fillRect(bX + 10, bY + 3, 6, 10, TFT_GREEN);
        batterySprite.fillRect(bX + 18, bY + 3, 6, 10, TFT_GREEN);
        batterySprite.fillRect(bX + 26, bY + 3, 6, 10, TFT_GREEN);
    } else if (batPct > 70) {
        batterySprite.fillRect(bX + 3, bY + 3, 6, 10, TFT_GREEN);
        batterySprite.fillRect(bX + 10, bY + 3, 6, 10, TFT_GREEN);
        batterySprite.fillRect(bX + 18, bY + 3, 6, 10, TFT_GREEN);
    } else if (batPct > 45) {
        batterySprite.fillRect(bX + 3, bY + 3, 6, 10, TFT_GREEN);
        batterySprite.fillRect(bX + 10, bY + 3, 6, 10, TFT_GREEN);
    } else if (batPct > 25) {
        batterySprite.fillRect(bX + 3, bY + 3, 6, 10, TFT_YELLOW);
        batterySprite.fillRect(bX + 10, bY + 3, 6, 10, TFT_YELLOW);
    } else {

        batterySprite.fillRect(bX + 3, bY + 3, 6, 10, TFT_RED);
    }

    batterySprite.pushSprite(targetSprite, x, y);
    if (batPct <= 0) powerStateSet(PowerState::POWER_OFF);
}

void drawShutdown(LGFX_Sprite *targetSprite)
{
    // This code only fires when we are in the process of shutting down.
    if (isisState.powerState != PowerState::SHUTTING_DOWN || isisSettings.powerControl == PowerControl::ALWAYS_ON) return;

    const int timeOut   = 45;
    int       secRemain = (int)(timeOut - (millis() - isisState.shutdownStartMs) / 1000);
    if (secRemain < 0) {
        isisState.powerState = PowerState::POWER_OFF;
        isisSetBacklightPwm(0);
        return;
    }
    int tw = targetSprite->width();
    int th = targetSprite->height();
    int ww = 380;
    int wh = 260;

    int topY = (th - wh) / 2;

    targetSprite->fillRect((tw - ww) / 2, topY, ww, wh, TFT_BLACK);
    targetSprite->drawRect((tw - ww) / 2, topY, ww, wh, TFT_WHITE);
    targetSprite->drawRect((tw - ww) / 2 + 1, topY + 1, ww - 2, wh - 2, TFT_WHITE);
    targetSprite->setTextDatum(TC_DATUM);
    targetSprite->setTextColor(TFT_WHITE);

    targetSprite->setTextSize(0.8);
    targetSprite->drawString("External Power Lost", tw / 2, topY + 6);
    targetSprite->drawFastHLine((tw - ww) / 2, topY + 34, ww, TFT_WHITE);
    targetSprite->setTextColor(TFT_YELLOW);
    targetSprite->drawString("Shutting down in:", tw / 2, topY + 40);

    targetSprite->setTextSize(2.0);
    char buf[6];
    sprintf(buf, "%d", secRemain);
    targetSprite->drawString(buf, tw / 2, topY + 100);

    targetSprite->setTextSize(0.8);
    targetSprite->setTextColor(TFT_WHITE);
    targetSprite->drawString("Press any key to continue", tw / 2, topY + 200);
    targetSprite->drawString("on battery power", tw / 2, topY + 226);
}

bool loadSettings()
{
    // Was: the MF EEPROM read_block(CC_ISIS_SETTINGS_OFFSET, isisSettings)
    // NVS needs no offset and no migration: on any mismatch the defaults are
    // loaded, exactly like the original handled an EEPROM read failure.
    if (!isisPrefs.begin("isis", true)) {
        // Namespace missing (virgin device): keep the defaults and store them
        // once, so that the first boot already leaves a valid NVS entry.
        saveSettings();
        ESP_LOGV("PREF", "Settings defaults stored (fresh NVS)\n");
        return false;
    }

    const size_t len = isisPrefs.getBytesLength("settings");
    if (len != sizeof(isisSettings)) {
        isisPrefs.end();
        isisSettings = CC_ISIS_Settings(); // Reset to defaults
        saveSettings();
        ESP_LOGV("PREF", "Settings defaults stored (no/short NVS entry)\n");
        return false;
    }

    isisPrefs.getBytes("settings", &isisSettings, sizeof(isisSettings));
    isisPrefs.end();

    if (isisSettings.version != SETTINGS_VERSION) {
        isisSettings = CC_ISIS_Settings(); // Reset to defaults
        saveSettings();
    }

    ESP_LOGV("PREF", "Settings loaded. Version: %d\n", isisSettings.version);
    return true;
}

bool saveSettings()
{
    // Was: the MF EEPROM write_block(CC_ISIS_SETTINGS_OFFSET, isisSettings) + commit()
    if (!isisPrefs.begin("isis", false)) return false;
    const size_t written = isisPrefs.putBytes("settings", &isisSettings, sizeof(isisSettings));
    isisPrefs.end();
    return written == sizeof(isisSettings);
}

// Bildschirmschoner: 100 = normal, darunter wird die Grundhelligkeit
// multiplikativ abgesenkt. Liegt bewusst in brightnessGamma(), weil alle
// Helligkeitsquellen (FDC-DISP_BL, Einstellungen, Tasten) hier durchlaufen.
static int g_dimPercent = 100;

void isisSetDim(int percent)
{
    if (percent < 1)   percent = 1;
    if (percent > 100) percent = 100;
    if (percent == g_dimPercent) return;
    g_dimPercent = percent;
    displaycfgApplyBrightness(displaycfgCurrent());
    Serial.printf("[isis] Helligkeit %d %% (Grundwert %d %%)\n",
                  g_dimPercent, (int)isisState.lcdBrightness);
}

int isisDimPercent() { return g_dimPercent; }

static int g_dimStufe    = 20;    // Helligkeit im gedimmten Zustand (Prozent)
static int g_dimSekunden = 300;   // Ruhezeit bis zum Dimmen

void isisSetDimPolicy(int prozent, int sekunden)
{
    if (prozent  >= 1 && prozent  <= 100)  g_dimStufe    = prozent;
    if (sekunden >= 5 && sekunden <= 3600) g_dimSekunden = sekunden;
    Serial.printf("[isis] Dimmen: nach %d s auf %d %%\n", g_dimSekunden, g_dimStufe);
}

int isisDimTarget()  { return g_dimStufe; }
int isisDimSeconds() { return g_dimSekunden; }

// Backlight direkt auf den LEDC-Kanal schreiben.
// Warum nicht lcd.setBrightness()? Der Weg ueber LovyanGFX zeigte auf diesem
// Panel keine Wirkung (roher ledcWrite dagegen schon). 9 Bit Aufloesung wie
// LovyanGFX, damit sich beide nicht in die Quere kommen.
void isisSetBacklightPwm(uint8_t pwm255)
{
    static bool eingerichtet = false;
    if (!eingerichtet) {
        ledcSetup(7, 1200, 9);
        ledcAttachPin(GPIO_NUM_38, 7);
        eingerichtet = true;
    }
    ledcWrite(7, ((uint32_t)pwm255 * 511) / 255);
}

uint8_t brightnessGamma(int percent)
{
    if (percent < 1) percent = 1;
    if (percent > 100) percent = 100;

    if (g_dimPercent < 100) {
        percent = (percent * g_dimPercent) / 100;
        if (percent < 3) percent = 3;
    }

    float normalized = percent / 100.0f;
    float corrected  = normalized * normalized * 0.75f + normalized * 0.25f;

    return (uint8_t)(40 + corrected * 215.0f);
}

// Saves the common isisState fields to NVS in preparation for a device-type switch.
// Subclasses override this to call CC_ISIS_Base::saveState() first, then write
// their own device-specific fields in a second Preferences session.
void CC_ISIS_Base::saveState()
{
    Preferences prefs;
    prefs.begin("isisState", false);
    prefs.putBool("switching", true);
    prefs.putInt("version", STATE_VERSION);
    // Common fields (message IDs 0-10)
    prefs.putInt("hdgBug", isisState.headingBugAngle);
    prefs.putInt("appType", isisState.gpsApproachType);
    prefs.putFloat("cdiOff", isisState.rawCdiOffset);
    prefs.putInt("cdiVal", isisState.cdiNeedleValid);
    prefs.putInt("toFrom", isisState.cdiToFrom);
    prefs.putFloat("gsiNdl", isisState.rawGsiNeedle);
    prefs.putInt("gsiVal", isisState.gsiNeedleValid);
    prefs.putInt("gndSpd", isisState.groundSpeed);
    prefs.putFloat("gndTrk", isisState.groundTrack);  // float (was putInt in HSI — bug fixed)
    prefs.putFloat("hdgAng", isisState.rawHeadingAngle);
    prefs.putInt("navSrc", isisState.navSource);
    prefs.end();
}

// Reads common isisState fields from NVS after a device-type switch.
// Subclasses call this first; if it returns true, they open a second session
// to restore their own device-specific fields.
bool CC_ISIS_Base::restoreState()
{
    Preferences prefs;
    prefs.begin("isisState", false);

    bool wasSwitching = prefs.getBool("switching", false);
    int  version      = prefs.getInt("version", 0);

    // Clear the flag immediately so a crash mid-restore doesn't loop.
    prefs.putBool("switching", false);

    if (!wasSwitching || version != STATE_VERSION) {
        prefs.end();
        return false;
    }

    // Common fields (message IDs 0-10)
    isisState.headingBugAngle = prefs.getInt("hdgBug", 0);
    isisState.gpsApproachType = prefs.getInt("appType", 0);
    isisState.rawCdiOffset    = prefs.getFloat("cdiOff", 0);
    isisState.cdiNeedleValid  = prefs.getInt("cdiVal", 1);
    isisState.cdiToFrom       = prefs.getInt("toFrom", 0);
    isisState.rawGsiNeedle    = prefs.getFloat("gsiNdl", 0);
    isisState.gsiNeedleValid  = prefs.getInt("gsiVal", 1);
    isisState.groundSpeed     = prefs.getInt("gndSpd", 0);
    isisState.groundTrack     = prefs.getFloat("gndTrk", 0);
    isisState.rawHeadingAngle = prefs.getFloat("hdgAng", 0);
    isisState.navSource       = prefs.getInt("navSrc", 1);

    prefs.end();
    return true;
}

void CC_ISIS_Base::sendEncoder(String name, int count, bool increase)
{
    // Was: MobiFlight cmdMessenger.sendCmdStart(kEncoderChange) ...
    // Now: one UDP uplink frame per detent, direction 0 = increase, 2 = decrease
    // (identical encoding to the original).
    for (int i = 0; i < count; i++) {
        char frame[48];
        if (protoBuildEncoder(frame, sizeof(frame), name.c_str(), increase ? 0 : 2) > 0) {
            netSendUplink(frame);
        }
    }
}

void CC_ISIS_Base::sendButton(String name, int pushType)
{
    // Was: MobiFlight cmdMessenger.sendCmdStart(kButtonChange) ...
    // Now: "BTN:<name>:<state>;" over UDP.
    char frame[48];
    if (protoBuildBtn(frame, sizeof(frame), name.c_str(), pushType) > 0) {
        netSendUplink(frame);
    }
}

// CC_ISIS_Base::setCommon() handles the message IDs shared by all device types (HSI, PFD, ISIS).
void CC_ISIS_Base::setCommon(int16_t messageID, char *setPoint)
{
    // Wake the display when data arrives, but only if MF is managing power.
    // (PowerControl::ALWAYS_ON is handled inside powerStateSet itself.)
    if (messageID > 0 && isisSettings.powerControl == PowerControl::DEVICE_MANAGED)
        powerStateSet(PowerState::POWER_ON);

    switch (messageID) {

    case -2: // PowerSavingMode: 1 = enter power saving, 0 = wake up
        if (atoi(setPoint) == 1)
            powerStateSet(PowerState::SHUTTING_DOWN);
        else
            powerStateSet(PowerState::POWER_ON);
        break;

    case -1: // Stop message from MF — device execution stops
        powerStateSet(PowerState::SHUTTING_DOWN);
        break;

    case 0: // AP Heading Bug
        isisState.headingBugAngle = atoi(setPoint);
        break;
    case 1: // Approach Type
        isisState.gpsApproachType = atoi(setPoint);
        break;
    case 2: // CDI Lateral Deviation
        isisState.rawCdiOffset = atof(setPoint);
        break;
    case 3: // CDI Needle Valid
        isisState.cdiNeedleValid = atoi(setPoint);
        break;
    case 4: // CDI To/From Flag
        isisState.cdiToFrom = atoi(setPoint);
        break;
    case 5: // Glide Slope Deviation
        isisState.rawGsiNeedle = atof(setPoint);
        break;
    case 6: // Glide Slope Needle Valid
        isisState.gsiNeedleValid = atoi(setPoint);
        break;
    case 7: // Ground Speed
        isisState.groundSpeed = atoi(setPoint);
        break;
    case 8: // Ground Track (Magnetic)
        isisState.groundTrack = atof(setPoint);
        break;
    case 9: // Heading (Magnetic)
        isisState.rawHeadingAngle = atof(setPoint);
        break;
    case 10: // Nav Source (1=GPS, 0=NAV)
        isisState.navSource = atoi(setPoint);
        break;
    case 12: // Brightness (0-255)
        isisState.lcdBrightness = max(0, min(atoi(setPoint), 255));
        isisSetBacklightPwm(isisState.lcdBrightness);
        break;
    case 13: // Power State: 0=off, 1=on
        switch (atoi(setPoint)) {
        case 0:
            powerStateSet(PowerState::SHUTTING_DOWN);
            break;
        case 1:
            powerStateSet(PowerState::POWER_ON);
            break;
        }
        break;
    case 14: // Power Control mode: 0=Manual, 1=DeviceManaged, 2=AlwaysOn
        switch (atoi(setPoint)) {
        case 0:
            isisSettings.powerControl = PowerControl::MANUAL;
            break;
        case 1:
            isisSettings.powerControl = PowerControl::DEVICE_MANAGED;
            break;
        case 2:
            isisSettings.powerControl = PowerControl::ALWAYS_ON;
            break;
        }
        saveSettings();
        break;
    }
}