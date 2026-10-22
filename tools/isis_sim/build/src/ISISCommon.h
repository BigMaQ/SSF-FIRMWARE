#pragma once
#include <Arduino.h>
#include <vector>
#include <memory>
#include <functional>
#include <Preferences.h>
#include "MFEEPROM.h"
#include "MFCustomDeviceTypes.h"
#include "commandmessenger.h"

#include "Sprites/battery.h"
// #include "Images\PrimaSans32.h" // ORIGINAL
#include "Images/PrimaSansMid32.h" // Medium weight

#define USE_GUITION_SCREEN

// Screen configuration selection
#ifdef USE_GUITION_SCREEN
#include "4inchLCDConfig_Guition.h"
#else
#include "4inchLCDConfig.h"
#endif

#define PIf                  3.14159f

#define CC_ISIS_SETTINGS_OFFSET 2048 // Well past MF config end (59 + 1496 = 1555)
#define SETTINGS_VERSION        7    // Bumped: removed HSI/PFD fields from CC_ISIS_Settings
#define STATE_VERSION         1

#define TFT_MAIN_TRANSPARENT TFT_PINK // Just pick a color not used in either display

enum class PowerState { INVALID,
                        POWER_OFF,
                        POWER_ON,
                        SHUTTING_DOWN,
                        BATTERY_POWERED,
                        HARD_POWER_OFF };
enum class PowerControl { MANUAL=0, DEVICE_MANAGED=1, ALWAYS_ON=2 };

struct CC_ISIS_Settings {
    int          version       = SETTINGS_VERSION;
    uint8_t      baroUnit      = 0; // 0: inHg, 1: kPa, 3: mmHg
    uint8_t      speedUnits    = 0; // 0:knot 1:mph 2:kph
    uint8_t      distanceUnits = 0; // 0: nm, 1: miles, 2:km
    uint8_t      tempUnits     = 0; // 0: F, 1: C
    uint8_t      lcdBrightness = 100;
    PowerControl powerControl  = PowerControl::ALWAYS_ON;
};

extern CC_ISIS_Settings isisSettings;

// Shared ISIS flight state
struct ISISState {

    int           lcdBrightness   = 100; // We will go 0-100 here.
    PowerState    powerState      = PowerState::SHUTTING_DOWN;
    //PowerControl  powerControl    = PowerControl::DEVICE_MANAGED; // 0-Manual, 1-Device, 2-Always On
    bool          forceRedraw     = false;
    unsigned long shutdownStartMs = 0;
    unsigned long batteryStartMs  = 0;

    // Heading and orientation
    float rawHeadingAngle = 0.0f;
    float headingAngle    = 0.0f;
    int   headingBugAngle = 0;
    float groundTrack     = 0.0f;

    // Attitude
    float rawBankAngle  = 0.0f;
    float bankAngle     = 30.0f;
    float rawPitchAngle = 0.0f;
    float pitchAngle    = 8.0f;
    float rawBallPos    = 0.0f;
    float ballPos       = 0.0f;

    // Speeds
    float rawAirspeed  = 0.0f;
    float airspeed     = 0.0f;
    float trueAirspeed = 0.0f;
    int   groundSpeed  = 0;
    float machSpeed    = 0.0f;

    // Altitude
    float rawAltitude      = 0.0f;
    float altitude         = 100.0f;
    int   rawVerticalSpeed = 0;
    int   verticalSpeed    = 0;
    float kohlsman         = 29.92f;
    int   mbPressure       = 1013;
    int   isStdPressure    = 0; // 1: STD pressure mode active

    // Navigation source and mode
    int  navSource          = 1; // 1=GPS, 0=NAV
    int  gpsApproachType    = 0; // Approach type enum
    int  navCDILabelIndex   = 0; // GPS:0, LOC1:1, VOR1:2, etc.
    int  cdiScaleLabel      = 0;
    bool terminalModeActive = true;

    // CDI (Course Deviation Indicator)
    int   cdiDirection   = 0; // HSI needle direction
    int   rawCdiDirection = 0;
    float rawCdiOffset   = 0.0f;
    float cdiOffset      = 0.0f;
    int   cdiNeedleValid = 1;
    int   cdiToFrom      = 0; // 0=off, 1=to, 2=from

    // Glide slope
    float rawGsiNeedle   = 0.0f;
    float gsiNeedle      = 0.0f;
    int   gsiNeedleValid = 1;

    // Desired track / Course to steer (shared between HSI and PFD)
    float desiredTrack      = 0.0f; // GPS course to steer / DTK (used by both HSI and PFD)
    int   desiredTrackValid = 0;
    float navCourse         = 0.0f; // NAV OBS course

    // Distance / waypoint
    float distNextWaypoint = 0.0f;
    int   gpsEteWp         = 0;

    // Other
    int oat = 15; // Outside air temp

    
};

extern ISISState isisState;

extern LGFX lcd;

float smoothDirection(float inputDir, float currentDir, float alpha, float threshold);
float smoothAngle(float input, float current, float alpha, float threshold);
int   smoothInput(int input, int current, float alpha, int threshold);
float smoothInput(float input, float current, float alpha, float snapThreshold);
void  drawShutdown(LGFX_Sprite *targetSprite);
void  drawBattery(LGFX_Sprite *targetSprite, int x, int y);

bool powerStateSet(PowerState ps);

uint8_t brightnessGamma(int percent);

extern MFEEPROM MFeeprom;
bool            loadSettings();
bool            saveSettings();

// Selection option for popup menus
struct SelectionOption {
    const char *label;
    int         value;
};

// BASE DEVICE CLASS

// Shared base class for all ISIS device types.
// Provides setCommon() for message IDs -2 through 14.
class CC_ISIS_Base {
public:
    void setCommon(int16_t messageID, char *setPoint);
    virtual void saveState();    // saves common isisState fields to NVS
    bool restoreState();         // reads common isisState fields from NVS; returns false if no saved state
    void sendEncoder(String name, int count, bool increase);
    void sendButton(String name, int pushType = 0);
};

// MENU SYSTEM

// Brightness menu manager - standalone, not part of main menu
class BrightnessMenu
{
private:
    bool isActive = false;

public:
    void show() { isActive = true; }
    void hide() { isActive = false; }
    bool active() const { return isActive; }

    void adjustBrightness(int delta)
    {
        isisState.lcdBrightness += delta;
        if (isisState.lcdBrightness < 1) isisState.lcdBrightness = 1;
        if (isisState.lcdBrightness > 100) isisState.lcdBrightness = 100;
        lcd.setBrightness(brightnessGamma(isisState.lcdBrightness));
    }

    void draw(LGFX_Sprite *targetSprite)
    {
        if (!isActive || !targetSprite) return;

        int popupWidth = 200, popupHeight = 120;
        int centerX = (targetSprite->width() - popupWidth) / 2; // Center X
        int centerY = (targetSprite->height() - popupHeight);   // Bottom Y

        // Draw popup
        targetSprite->fillRoundRect(centerX, centerY, popupWidth, popupHeight, 4, 0x7BEF);
        targetSprite->fillRoundRect(centerX + 3, centerY + 3, popupWidth - 6, popupHeight - 6, 3, TFT_BLACK);

        // Title
        targetSprite->setTextColor(TFT_WHITE);
        targetSprite->setTextDatum(lgfx::v1::textdatum::textdatum_t::top_center);
        targetSprite->setTextSize(0.7);
        targetSprite->drawString("Backlight", centerX + popupWidth / 2, centerY + 10);

        // Percentage
        targetSprite->setTextColor(TFT_CYAN);
        targetSprite->setTextSize(1.2);
        targetSprite->setTextDatum(lgfx::v1::textdatum::textdatum_t::middle_center);
        String valueStr = String(isisState.lcdBrightness) + "%";
        targetSprite->drawString(valueStr, centerX + popupWidth / 2, centerY + popupHeight / 2 + 5);

        // Bar
        int barWidth  = popupWidth - 40;
        int barHeight = 12;
        int barX      = centerX + 20;
        int barY      = centerY + popupHeight - 30;
        targetSprite->drawRoundRect(barX, barY, barWidth, barHeight, 2, TFT_WHITE);
        int fillWidth = (barWidth - 4) * isisState.lcdBrightness / 100;
        targetSprite->fillRoundRect(barX + 2, barY + 2, fillWidth, barHeight - 4, 1, TFT_CYAN);
    }
};

extern BrightnessMenu brightnessMenu;

template <typename ParentDevice>
class ISISMenuBase
{
public:
    enum class MenuState {
        BROWSING,
        ADJUSTING,
        SELECTING,
        SETTINGS_BROWSING
    };

    class MenuItemBase
    {
    public:
        virtual ~MenuItemBase()                 = default;
        virtual String getTitle()               = 0;
        virtual String getDisplayValue()        = 0;
        virtual int    getDisplayValueColor()   = 0;
        virtual void   onEncoderTurn(int delta) = 0;
        virtual void   onEncoderPress()         = 0;

        virtual bool isVisible() const { return true; }

        // Icon support - return nullptr if no icon
        virtual const uint16_t *getIcon() { return nullptr; }
        virtual int             getIconWidth() { return 0; }
        virtual int             getIconHeight() { return 0; }
    };

protected:
    ParentDevice                              *parent;
    std::vector<std::unique_ptr<MenuItemBase>> menuItems;
    int                                        currentHighlight = 0;
    MenuItemBase                              *adjustingItem    = nullptr;

    // Selection popup state
    std::vector<SelectionOption> selectionOptions;
    int                         *selectionValuePtr  = nullptr;
    int                          selectionHighlight = 0; // Index in options array

public:
    bool      menuActive   = false;
    MenuState currentState = MenuState::BROWSING;
    int       menuXpos     = 0;
    int       menuYpos     = 0;
    int       menuHeight   = 0;
    int       menuWidth    = 0;

    ISISMenuBase(ParentDevice *p) : parent(p) {}

    // Pure virtual - each device defines its menu items
    virtual void createMenuItems() = 0;

    // Pure virtual - each device provides its target sprite
    virtual LGFX_Sprite *getTargetSprite() = 0;

    void initializeMenu()
    {
        menuItems.clear();
        createMenuItems();
        currentState     = MenuState::BROWSING;
        currentHighlight = 0;
        while (currentHighlight < menuItems.size() &&
               !menuItems[currentHighlight]->isVisible()) {
            currentHighlight++;
        }
        if (currentHighlight >= menuItems.size()) currentHighlight = 0;
    }

    bool setActive(bool isActive)
    {
        menuActive = isActive;
        if (!isActive) currentState = MenuState::BROWSING;
        return menuActive;
    }

    virtual void handleEncoder(int delta)
    { // Make virtual to support the Settings in the PFD.
        if (currentState == MenuState::BROWSING) {
            scrollHighlight(delta);
        } else if (currentState == MenuState::SELECTING) {
            // Scroll through selection options
            selectionHighlight += (delta > 0) ? 1 : -1;
            if (selectionHighlight < 0) selectionHighlight = 0;
            if (selectionHighlight >= selectionOptions.size()) selectionHighlight = selectionOptions.size() - 1;
        } else if (currentState == MenuState::ADJUSTING && adjustingItem) {
            adjustingItem->onEncoderTurn(delta);
        }
    }

    virtual void handleEncoderButton(bool pressed)
    {
        if (!pressed) return;

        if (currentState == MenuState::BROWSING) {
            // Make sure current item is visible
            if (currentHighlight >= menuItems.size() || !menuItems[currentHighlight]->isVisible()) {
                return; // Shouldn't happen...
            }
            menuItems[currentHighlight]->onEncoderPress();

        } else if (currentState == MenuState::SELECTING) {
            // Save selected value and return to menu
            *selectionValuePtr = selectionOptions[selectionHighlight].value;
            saveSettings();
            adjustingItem = nullptr;
            currentState  = MenuState::BROWSING;
            setActive(false); // Not sure if menu should stay open or close. This will close it.
        } else if (currentState == MenuState::ADJUSTING) {
            adjustingItem = nullptr;
            currentState  = MenuState::BROWSING;
            setActive(false);
        }
    }

    void enterSelectionMode(MenuItemBase *item, std::vector<SelectionOption> options, int *valuePtr)
    {
        adjustingItem     = item;
        selectionOptions  = options;
        selectionValuePtr = valuePtr;

        // Find current value in options and set highlight
        selectionHighlight = 0;
        for (int i = 0; i < options.size(); i++) {
            if (options[i].value == *valuePtr) {
                selectionHighlight = i;
                break;
            }
        }

        currentState = MenuState::SELECTING;
    }

    void scrollHighlight(int delta)
    {
        int newHighlight = getNextVisibleItem(currentHighlight, delta);

        // Boundary check
        if (newHighlight < 0) newHighlight = 0;
        if (newHighlight >= menuItems.size()) newHighlight = menuItems.size() - 1;

        // Make sure we landed on a visible item
        if (newHighlight < menuItems.size() && menuItems[newHighlight]->isVisible()) {
            currentHighlight = newHighlight;
        }
    }

    int getVisibleItemCount() const
    {
        int count = 0;
        for (const auto &item : menuItems) {
            if (item->isVisible()) count++;
        }
        return count;
    }

    int getNextVisibleItem(int currentIndex, int delta) const
    {
        int step = (delta > 0) ? 1 : -1;
        int idx  = currentIndex + step;

        while (idx >= 0 && idx < menuItems.size()) {
            if (menuItems[idx]->isVisible()) {
                return idx;
            }
            idx += step;
        }

        // No more visible items.
        return currentIndex;
    }

    // Get the visible index of an absolute index (for display purposes)
    // Returns which "nth visible item" this is
    int getVisibleIndex(int absoluteIndex) const
    {
        int visibleIdx = 0;
        for (int i = 0; i < absoluteIndex && i < menuItems.size(); i++) {
            if (menuItems[i]->isVisible()) visibleIdx++;
        }
        return visibleIdx;
    }

    void enterAdjustmentMode(MenuItemBase *item)
    {
        adjustingItem = item;
        currentState  = MenuState::ADJUSTING;
    }

    void drawMenu()
    {

        static int currentStartPos = 0;
        if (!menuActive || currentState == MenuState::ADJUSTING || currentState == MenuState::SELECTING) return;

        auto targetSprite = getTargetSprite();
        if (!targetSprite) return;

        int itemWidth = 105, itemHeight = 80, itemSpacing = 9, outlineWidth = 4;
        int curX   = itemSpacing;
        int yPos   = targetSprite->height() - 120;
        menuYpos   = yPos;
        menuXpos   = 0;
        menuWidth  = targetSprite->width();
        menuHeight = 108;

        int menuSize = (targetSprite->width() - outlineWidth * 2 - itemSpacing * 3) / itemWidth;

        // Visible items
        int visibleItemCount    = getVisibleItemCount();
        int currentVisibleIndex = getVisibleIndex(currentHighlight);

        // Draw menu background directly on target sprite
        targetSprite->fillRoundRect(0, yPos, targetSprite->width(), 108, outlineWidth, 0x7BEF);                                                                     // Light gray. Include space for the scroll bar at bottom.
        targetSprite->fillRoundRect(outlineWidth, yPos + outlineWidth, targetSprite->width() - outlineWidth * 2, 108 - outlineWidth * 2, outlineWidth / 2, 0x0000); // Black

        yPos += itemSpacing;

        // Figure scroll position based on visible items.
        if (currentVisibleIndex < currentStartPos) currentStartPos = currentVisibleIndex;
        if (currentVisibleIndex > currentStartPos + (menuSize - 1)) currentStartPos = currentVisibleIndex - (menuSize - 1);
        //      Serial.printf("cHi: %d, cSPo: %d\n", currentHighlight, currentStartPos);

        // Draw visible menu items (up to menuSize + 1)
        int visibleDrawn   = 0;
        int visibleSkipped = 0;

        for (int i = 0; i < menuItems.size() && visibleDrawn < menuSize; i++) {
            if (!menuItems[i]->isVisible()) continue;

            // Skip items before scroll
            if (visibleSkipped < currentStartPos) {
                visibleSkipped++;
                continue;
            }

            bool isHighlighted = (i == currentHighlight);

            // Draw item rectangle
            targetSprite->fillRoundRect(curX, yPos, itemWidth, itemHeight, outlineWidth, isHighlighted ? 0xFFFF : 0x7BEF); // White if selected, gray otherwise
            targetSprite->fillRoundRect(curX + outlineWidth, yPos + outlineWidth, itemWidth - outlineWidth * 2, itemHeight - outlineWidth * 2, outlineWidth / 2, 0x0000);

            // Draw item title
            targetSprite->setTextColor(0xFFFF, 0x0000); // White on black
            targetSprite->setTextDatum(lgfx::v1::textdatum::textdatum_t::top_center);
            targetSprite->setTextSize(0.6);
            targetSprite->drawString(menuItems[i]->getTitle().c_str(), curX + itemWidth / 2, yPos + 15);

            // Draw item icon if it has one
            const uint16_t *iconData = menuItems[i]->getIcon();
            if (iconData != nullptr) {
                int iconWidth  = menuItems[i]->getIconWidth();
                int iconHeight = menuItems[i]->getIconHeight();
                int iconX      = curX + ((itemWidth - iconWidth) / 2);
                int iconY      = yPos + ((itemHeight - iconHeight) / 2) + 10; // Offset down from title
                targetSprite->pushImage(iconX, iconY, iconWidth, iconHeight, iconData);
            }

            // Draw item value if it has one
            String displayValue = menuItems[i]->getDisplayValue();
            if (displayValue.length() > 0) {
                targetSprite->setTextColor(menuItems[i]->getDisplayValueColor(), 0x0000);
                targetSprite->setTextDatum(lgfx::v1::textdatum::textdatum_t::bottom_center);
                targetSprite->setTextSize(0.8);
                targetSprite->drawString(displayValue.c_str(), curX + itemWidth / 2, yPos + itemHeight - 3);
            }

            visibleDrawn++;
            curX += (itemWidth + itemSpacing);
        }
        // Draw the scroll bar based on visible count

        yPos               = targetSprite->height() - 120 + 100 - outlineWidth;
        int totalLineWidth = (targetSprite->width() - (outlineWidth * 4) - (itemSpacing * 2));
        int lineIncrement  = 0;
        int xPos           = outlineWidth * 2 + itemSpacing;
        int lineWidth      = totalLineWidth;

        if (visibleItemCount > menuSize) {
            lineIncrement = totalLineWidth / visibleItemCount;
            // draw margin line
            targetSprite->drawFastHLine(xPos, yPos + 1, totalLineWidth, TFT_DARKGRAY);

            // adjust scroll bar width to fractional size
            lineWidth = (menuSize * lineIncrement);
            xPos += currentStartPos * lineIncrement;
        }

        // draw scroll bar
        targetSprite->drawFastHLine(xPos, yPos, lineWidth, TFT_WHITE);
        targetSprite->drawFastHLine(xPos, yPos + 1, lineWidth, TFT_WHITE);
        targetSprite->drawFastHLine(xPos, yPos + 2, lineWidth, TFT_WHITE);

        // The target sprite will get pushed to screen in the main loop.
    }

    void drawAdjustmentPopup()
    {
        if (!adjustingItem) return;

        //  LGFX_Sprite *sprite = getTargetSprite();
        //      sprite->fillSprite(TFT_MAIN_TRANSPARENT);  // clear the sprite
        // clear the previous menu

        auto targetSprite = getTargetSprite();
        if (!targetSprite) return;

        // Draw popup in center of target sprite
        int popupWidth = 160, popupHeight = 100;
        int centerX = (targetSprite->width() - popupWidth) / 2;
        int centerY = (targetSprite->height() - popupHeight - 15); // 15 off the bottom

        // Draw popup background
        targetSprite->fillRoundRect(centerX, centerY, popupWidth, popupHeight, 3, TFT_BLACK);
        targetSprite->fillGradientRect(centerX + 2, centerY + 2, popupWidth - 4, 40, 0x7BEF, 0x0000, lgfx::v1::gradient_fill_styles::vertical_linear);
        targetSprite->drawRoundRect(centerX, centerY, popupWidth, popupHeight, 3, 0xFFFF);
        targetSprite->drawRoundRect(centerX + 1, centerY + 1, popupWidth - 2, popupHeight - 2, 2, 0xFFFF);

        // Draw title
        targetSprite->setTextColor(0xFFFF);
        targetSprite->setTextDatum(lgfx::v1::textdatum::textdatum_t::top_center);
        targetSprite->setTextSize(0.7);
        targetSprite->drawString(adjustingItem->getTitle(), centerX + popupWidth / 2, centerY + 10);

        // Draw current value with color
        targetSprite->setTextColor(adjustingItem->getDisplayValueColor());
        targetSprite->setTextSize(1.0);
        targetSprite->setTextDatum(lgfx::v1::textdatum::textdatum_t::middle_center);
        String valueStr = adjustingItem->getDisplayValue();
        targetSprite->drawString(valueStr, centerX + popupWidth / 2, centerY + popupHeight / 2 + 15);
    }

    // Draw Selection, used to select bearing pointer source
    void drawSelectionPopup()
    {
        if (!adjustingItem || selectionOptions.empty()) return;

        auto targetSprite = getTargetSprite();
        if (!targetSprite) return;

        // Popup dimensions
        int popupWidth   = 200;
        int itemHeight   = 50;
        int spacing      = 8;
        int maxVisible   = 4;
        int visibleItems = min(maxVisible, (int)selectionOptions.size());
        int popupHeight  = (visibleItems * itemHeight) + ((visibleItems + 1) * spacing) + 40; // +40 for title

        int centerX = (targetSprite->width() - popupWidth) / 2;
        int centerY = (targetSprite->height() - popupHeight) / 2;

        // Draw popup background
        targetSprite->fillRoundRect(centerX, centerY, popupWidth, popupHeight, 4, 0x7BEF); // Light gray
        targetSprite->fillRoundRect(centerX + 3, centerY + 3, popupWidth - 6, popupHeight - 6, 3, TFT_BLACK);

        // Draw title
        targetSprite->setTextColor(TFT_WHITE);
        targetSprite->setTextDatum(lgfx::v1::textdatum::textdatum_t::top_center);
        targetSprite->setTextSize(0.7);
        targetSprite->drawString(adjustingItem->getTitle(), centerX + popupWidth / 2, centerY + 10);

        // Calculate scroll offset to keep selection visible
        int startIdx = 0;
        if (selectionHighlight >= maxVisible) {
            startIdx = selectionHighlight - maxVisible + 1;
        }

        // Draw option buttons
        int yPos = centerY + 40;
        for (int i = startIdx; i < min(startIdx + maxVisible, (int)selectionOptions.size()); i++) {
            bool isSelected = (i == selectionHighlight);
            bool isCurrent  = (selectionOptions[i].value == *selectionValuePtr);

            // Button colors
            uint16_t borderColor = isSelected ? TFT_WHITE : 0x7BEF;
            uint16_t bgColor     = TFT_BLACK;
            uint16_t textColor   = isCurrent ? TFT_CYAN : TFT_WHITE;

            int btnX     = centerX + spacing;
            int btnWidth = popupWidth - (spacing * 2);

            // Draw button
            targetSprite->fillRoundRect(btnX, yPos, btnWidth, itemHeight, 3, borderColor);
            targetSprite->fillRoundRect(btnX + 2, yPos + 2, btnWidth - 4, itemHeight - 4, 2, bgColor);

            // Draw label
            targetSprite->setTextColor(textColor, bgColor);
            targetSprite->setTextDatum(lgfx::v1::textdatum::textdatum_t::middle_center);
            targetSprite->setTextSize(0.9);
            targetSprite->drawString(selectionOptions[i].label, btnX + btnWidth / 2, yPos + itemHeight / 2);

            yPos += itemHeight + spacing;
        }
    }

    void sendEncoder(String name, int count, bool increase)
    {
        parent->sendEncoder(name, count, increase);
    }

    void sendButton(String name, int pushType = 0)
    {
        parent->sendButton(name, pushType);
    }
};
