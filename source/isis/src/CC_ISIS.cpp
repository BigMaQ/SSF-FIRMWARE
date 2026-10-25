#include "CC_ISIS.h"
#include "Images/isisFont.h"
#include "Sprites/isisBg.h"
#include "Sprites/attBackground.h"
#include "Sprites/blackoutArc.h"
#include "Sprites/altBg.h"
#include "Sprites/rollArc.h"
#include "Sprites/rollPointer.h"
#include "Sprites/rollSlip.h"
#include "Sprites/gsDeviation.h"
#include "Sprites/gsDeviationH.h"
#include "Sprites/deviationDiamond.h"   // magenta Ablage-Diamant (PFD-Variante, wie auf dem echten ISIS)

#include <cmath>

#define PITCH_LINE_NARROW 17 // 2.5° and 7.5° tick marks
#define PITCH_LINE_MEDIUM 27 // 5° tick marks
#define PITCH_LINE_WIDE   54 // 10° tick marks (labeled)

#define ATT_LEFT_EDGE 76
#define ATT_TOP_EDGE  56
#define ATT_HORIZON   194
#define ALT_LEFT_EDGE 398
#define ATT_WIDTH     320
#define ATT_HEIGHT    350

LGFX_Sprite attSprite(&lcd);   // Main attitude display

LGFX_Sprite slipSprite(&attSprite);  // Roll pointer and slip skid indicator

LGFX_Sprite ladderValSprite(&attSprite); // Used to hold the scale numbers on the pitch ladder
LGFX_Sprite blackoutArcSprite(&attSprite);  // Arc to punch-out the top and bottom of the attSprite
LGFX_Sprite speedSprite(&lcd); // Holds the speed tape

LGFX_Sprite altSprite(&lcd);         // Holds the alt tape
LGFX_Sprite alt100Sprite(&altSprite);   

LGFX_Sprite kohlsSprite(&lcd);



CC_ISIS::CC_ISIS()
{
}

void CC_ISIS::setupSprites()
{
    // bgSprite.setColorDepth(8);
    // bgSprite.createSprite(ISISBG_IMG_WIDTH, ISISBG_IMG_HEIGHT);
    // bgSprite.pushImage(0, 0, ISISBG_IMG_WIDTH, ISISBG_IMG_HEIGHT, ISISBG_IMG_DATA);

    attSprite.setPsram(true);      // interner RAM ist zu klein (siehe main.cpp)
    attSprite.setColorDepth(8);
    attSprite.createSprite(320, 350);
    attSprite.loadFont(A320ISIS24);
    attSprite.setTextColor(TFT_WHITE);
    attSprite.setTextSize(1.0f);
    attSprite.setTextDatum(CC_DATUM);

    slipSprite.setPsram(true);      // interner RAM ist zu klein (siehe main.cpp)
    slipSprite.setColorDepth(8);
    slipSprite.createSprite(ROLLSLIP_IMG_WIDTH*2+ROLLPOINTER_IMG_WIDTH, ROLLSLIP_IMG_HEIGHT + ROLLPOINTER_IMG_HEIGHT);
    slipSprite.fillSprite(TFT_MAGENTA);
    slipSprite.pushImage(slipSprite.width()/2 - ROLLPOINTER_IMG_WIDTH/2, 0, ROLLPOINTER_IMG_WIDTH, ROLLPOINTER_IMG_HEIGHT, ROLLPOINTER_IMG_DATA, 8184);
    slipSprite.setPivot(ROLLSLIP_IMG_WIDTH, 159);  // 159 from ref image. dist from tip to center.

    ladderValSprite.setPsram(true);      // interner RAM ist zu klein (siehe main.cpp)
    ladderValSprite.setColorDepth(8);
    ladderValSprite.createSprite(45, 28); // Hold two digits. Digits are 24 high
    ladderValSprite.setPivot(25, 14);
    ladderValSprite.setTextColor(TFT_WHITE, TFT_BLACK);
    ladderValSprite.setTextDatum(CL_DATUM);
    ladderValSprite.loadFont(A320ISIS24);

    blackoutArcSprite.setPsram(true);      // interner RAM ist zu klein (siehe main.cpp)
    blackoutArcSprite.setColorDepth(8);
    blackoutArcSprite.createSprite(BLACKOUTARC_IMG_WIDTH + 2, BLACKOUTARC_IMG_HEIGHT); // make it a little wider to fix any rotation integer math
    blackoutArcSprite.fillSprite(TFT_BLACK);
    blackoutArcSprite.pushImage(1, 0, BLACKOUTARC_IMG_WIDTH, BLACKOUTARC_IMG_HEIGHT, BLACKOUTARC_IMG_DATA);
    blackoutArcSprite.setPivot(BLACKOUTARC_IMG_WIDTH / 2, BLACKOUTARC_IMG_HEIGHT / 2);

    speedSprite.setPsram(true);      // interner RAM ist zu klein (siehe main.cpp)
    speedSprite.setColorDepth(8);
    speedSprite.createSprite(ATT_LEFT_EDGE - 1, attSprite.height());
    speedSprite.loadFont(A320ISIS24);
    speedSprite.setColor(TFT_LIGHTGRAY);
    speedSprite.setTextColor(TFT_LIGHTGRAY, TFT_BLACK);
    speedSprite.setTextDatum(CR_DATUM);

    altSprite.setPsram(true);      // interner RAM ist zu klein (siehe main.cpp)
    altSprite.setColorDepth(8);
    altSprite.createSprite(ALT_LEFT_EDGE, attSprite.height());
    altSprite.loadFont(A320ISIS24);
    altSprite.setColor(TFT_LIGHTGRAY);
    altSprite.setTextColor(TFT_LIGHTGRAY, TFT_BLACK);
    altSprite.setTextDatum(CL_DATUM);


    alt100Sprite.setPsram(true);      // interner RAM ist zu klein (siehe main.cpp)
    alt100Sprite.setColorDepth(8);
    alt100Sprite.createSprite(ALTBG_IMG_WIDTH, ALTBG_IMG_HEIGHT);
    alt100Sprite.pushImage(0, 0, ALTBG_IMG_WIDTH, ALTBG_IMG_HEIGHT, ALTBG_IMG_DATA, 8184);
    alt100Sprite.loadFont(A320ISIS24);
    alt100Sprite.setTextColor(TFT_GREEN);
    alt100Sprite.setTextDatum(CR_DATUM);

    kohlsSprite.setPsram(true);      // interner RAM ist zu klein (siehe main.cpp)
    kohlsSprite.setColorDepth(8);
    kohlsSprite.createSprite(120, 33);
    kohlsSprite.loadFont(A320ISIS24);
    kohlsSprite.setTextDatum(TL_DATUM);
    kohlsSprite.setTextColor(TFT_BLUE);

    // Sichtbar machen, wenn eine Allokation scheitert: ein leerer Sprite
    // zeichnet nichts und das Display bleibt schwarz - ohne diese Zeile
    // sucht man den Fehler im Zeichencode statt in der Speicherverwaltung.
    Serial.printf("[isis] Sprites: att=%u slip=%u ladder=%u arc=%u speed=%u alt=%u alt100=%u kohls=%u\n",
                  (unsigned)attSprite.bufferLength(), (unsigned)slipSprite.bufferLength(),
                  (unsigned)ladderValSprite.bufferLength(), (unsigned)blackoutArcSprite.bufferLength(),
                  (unsigned)speedSprite.bufferLength(), (unsigned)altSprite.bufferLength(),
                  (unsigned)alt100Sprite.bufferLength(), (unsigned)kohlsSprite.bufferLength());
}

void CC_ISIS::begin()
{
    loadSettings();

    lcd.setColorDepth(8);
    lcd.init();

    setupSprites();

    isisSetBacklightPwm(brightnessGamma(isisSettings.lcdBrightness));
    isisState.lcdBrightness = isisSettings.lcdBrightness;

#ifdef USE_GUITION_SCREEN
    lcd.setRotation(3); // Puts the USB jack at the bottom on Guition screen.
#else
    lcd.setRotation(0); // Orients the Waveshare screen with FPCB connector at bottom.
#endif

    lcd.fillScreen(TFT_BLACK);
    lcd.setTextColor(TFT_WHITE, TFT_BLACK);
    // lcd.loadFont(A320ISIS24);
    // lcd.setTextDatum(CC_DATUM);
    // lcd.drawString("A320 STARTUP", 240, 240);
}

void CC_ISIS::attach()
{
}

void CC_ISIS::detach()
{
}

void CC_ISIS::set(int16_t messageID, char *setPoint)
{
    // Common messages are handled by the base class.
    if (messageID < MSG_HSI_MIN)
        setCommon(messageID, setPoint);
    // ISIS-specific messages (IDs >= MSG_ISIS_MIN) go here when implemented.
    switch (messageID) {
    case 80: // Pitch
        isisState.rawPitchAngle = atof(setPoint);
        break;
    case 72: // Bank
        isisState.rawBankAngle = atof(setPoint);
        break;
    case 60: // Airspeed
        isisState.rawAirspeed = atof(setPoint);
        break;
    case 71: // Ball slip/skid
        isisState.ballPos = atof(setPoint);
        break;
    case 77: // Indicated Altitude
        isisState.rawAltitude = atof(setPoint);
        break;
    case 100: // Pressure in mb
        isisState.mbPressure = atoi(setPoint);
        break;
    case 101: // are we in std pressure mode
        isisState.isStdPressure = atoi(setPoint);
        break;
    case 103:  // LS-Anzeige ein/aus (eigene ID, Runde 7)
        isisState.lsActive = atoi(setPoint) ? 1 : 0;
        break;
    case 104:  // Localizer-Ablage -1..1 (+ rechts)
        isisState.lsLocDev = atof(setPoint);
        break;
    case 105:  // Localizer-Signal gueltig
        isisState.lsLocValid = atoi(setPoint) ? 1 : 0;
        break;
    case 106:  // Gleitpfad-Ablage -1..1 (+ oben)
        isisState.lsGsDev = atof(setPoint);
        break;
    case 107:  // Gleitpfad-Signal gueltig
        isisState.lsGsValid = atoi(setPoint) ? 1 : 0;
        break;
    case 102:  // Mach number
        isisState.machSpeed = atof(setPoint);
        break;
    }
}

void CC_ISIS::drawAttitude()
{

    const int16_t CENTER_X = attSprite.width() / 2;
    const int16_t CENTER_Y = ATT_HORIZON-2;

    const uint16_t HORIZON_COLOR = 0xFFFF;
    const uint16_t SKY_COLOR     = TFT_BLUE;
    const uint16_t GND_COLOR     = 37316; // = TFT_BROWN;

    float bankRad = isisState.bankAngle * PIf / 180.0f;
    // Pitch scaling factor (pixels per degree)
    const float PITCH_SCALE = 8.0;

    bool inverted = (isisState.bankAngle > 90.0 || isisState.bankAngle < -90.0);

    // Calculate vertical offset of the horizon due to pitch.
    // When inverted, flip the pitch offset to match what pilot sees from inverted perspective
    // A negative isisState.pitchAngle (nose up) moves the horizon down (positive pixel offset).
    float horizonPixelOffset = inverted ? (isisState.pitchAngle * PITCH_SCALE) : (-isisState.pitchAngle * PITCH_SCALE);

    attSprite.fillSprite(SKY_COLOR);

    // Pre-calculate trig for the loop.
    // tanBank drives the slope of the horizon line across columns.
    // When pitch is non-zero, rotating that offset line by bankAngle shifts the Y-intercept
    // at CENTER_X by horizonPixelOffset/cosBank (not just horizonPixelOffset), so we need cosBank too.
    float tanBank = tan(bankRad);
    float cosBank = cos(bankRad);
    // Guard: at ~90° bank cosBank→0 and the formula would blow up. Clamp to ±0.01 (≈89.4°).
    float safeCosBank    = (fabsf(cosBank) < 0.01f) ? (cosBank < 0.0f ? -0.01f : 0.01f) : cosBank;
    float horizonCenterY = CENTER_Y + horizonPixelOffset / safeCosBank;

    for (int16_t x = 0; x < attSprite.width(); x++) {
        // Distance from center
        int16_t dx = x - CENTER_X;

        // Correct horizon Y for this column: the rotated-line equation gives
        //   y = (CENTER_Y + horizonPixelOffset/cosBank) + dx*tanBank
        // which matches the rotation-matrix method used for the pitch ladder and horizon line.
        float horizonY = horizonCenterY + (dx * tanBank);

        int16_t horizonPixel = round(horizonY);

        // if (inverted) {
        //     // When inverted, ground is ABOVE the horizon line
        //     if (horizonPixel > 0) {
        //         attSprite.drawFastVLine(x, 0, min((int16_t)ATT_HEIGHT, horizonPixel),  GND_COLOR);
        //         if (x < SPEED_COL_WIDTH || x > (ATTITUDE_WIDTH - ALTITUDE_COL_WIDTH)) attitude.drawFastVLine(x, max((int16_t)0, horizonPixel), ATTITUDE_HEIGHT - max((int16_t)0, horizonPixel), (x < SPEED_COL_WIDTH || x > ATTITUDE_WIDTH - ALTITUDE_COL_WIDTH) ? DARK_SKY_COLOR : SKY_COLOR);
        //         // attitude.drawFastVLine(x, 0, min((int16_t)attitude.height(), horizonPixel), GND_COLOR);
        //     }
        // } else {
        // When upright, ground is BELOW the horizon line
        if (horizonPixel < attSprite.height()) {
            attSprite.drawFastVLine(x, max((int16_t)0, horizonPixel), ATT_HEIGHT - max((int16_t)0, horizonPixel), GND_COLOR);
            // attSprite.drawLine(x, max((int16_t)0, horizonPixel), x, ATT_HEIGHT, GND_COLOR);
        }
    }

    // --- 2. Draw Pitch Ladder (with correct math) ---
    // cosBank was already computed above for the fill loop; compute sinBank here.
    float sinBank = sin(bankRad);

    auto drawPitchLine = [&](float pitchDegrees, int lineWidth, bool showNumber, uint16_t color) {
        // Calculate the line's vertical distance from the screen center in an un-rotated frame.
        // A positive value moves the line DOWN the screen.
        // (pitchDegrees - isisState.pitchAngle) gives the correct relative position.
        float verticalOffset = (pitchDegrees - isisState.pitchAngle) * PITCH_SCALE;

        // Define the line's endpoints relative to the screen center before rotation
        float halfWidth = lineWidth / 2.0;
        float p1x_unrot = -halfWidth;
        float p1y_unrot = verticalOffset;
        float p2x_unrot = +halfWidth;
        float p2y_unrot = verticalOffset;

        // Apply the bank rotation to the endpoints
        int16_t x1 = CENTER_X + p1x_unrot * cosBank - p1y_unrot * sinBank;
        int16_t y1 = CENTER_Y + p1x_unrot * sinBank + p1y_unrot * cosBank;
        int16_t x2 = CENTER_X + p2x_unrot * cosBank - p2y_unrot * sinBank;
        int16_t y2 = CENTER_Y + p2x_unrot * sinBank + p2y_unrot * cosBank;

        attSprite.drawLine(x1, y1, x2, y2, color);
        //        attSprite.drawWideLine(x1, y1, x2, y2, 2, color);

        if (showNumber && abs(pitchDegrees) >= 10) {
            char pitchText[4];
            sprintf(pitchText, "%d", (int)abs(pitchDegrees));

            float textOffset   = halfWidth + 15;
            float text1x_unrot = -textOffset;
            float texty_unrot  = verticalOffset;

            int16_t textX1 = CENTER_X + text1x_unrot * cosBank - texty_unrot * sinBank;
            int16_t textY1 = CENTER_Y + text1x_unrot * sinBank + texty_unrot * cosBank;

            ladderValSprite.fillSprite(TFT_BLACK);
            ladderValSprite.drawString(pitchText, 2, ladderValSprite.height() / 2);
            attSprite.setPivot(textX1, textY1);
            ladderValSprite.pushRotated(inverted ? isisState.bankAngle + 180.0 : isisState.bankAngle, TFT_BLACK);
            //            ladderValSprite.pushRotated(inverted ? isisState.bankAngle + 180.0 : isisState.bankAngle);
        }
    };

    // Define and draw all the pitch lines.
    // 2.5° increments, 90 to -90 (0° is the horizon, drawn separately).
    // Wide (80) + number at every 10°; medium (60) at every 5°; small (40) at 2.5° / 7.5° offsets.
    const struct {
        float deg;
        int   width;
        bool  num;
    } pitch_lines[] = {
        // clang-format off
        { 90.0, PITCH_LINE_WIDE, true },  { 87.5, PITCH_LINE_NARROW, false}, { 85.0, PITCH_LINE_MEDIUM, false}, { 82.5, PITCH_LINE_NARROW, false},
        { 80.0, PITCH_LINE_WIDE, true },  { 77.5, PITCH_LINE_NARROW, false}, { 75.0, PITCH_LINE_MEDIUM, false}, { 72.5, PITCH_LINE_NARROW, false},
        { 70.0, PITCH_LINE_WIDE, true },  { 67.5, PITCH_LINE_NARROW, false}, { 65.0, PITCH_LINE_MEDIUM, false}, { 62.5, PITCH_LINE_NARROW, false},
        { 60.0, PITCH_LINE_WIDE, true },  { 57.5, PITCH_LINE_NARROW, false}, { 55.0, PITCH_LINE_MEDIUM, false}, { 52.5, PITCH_LINE_NARROW, false},
        { 50.0, PITCH_LINE_WIDE, true },  { 47.5, PITCH_LINE_NARROW, false}, { 45.0, PITCH_LINE_MEDIUM, false}, { 42.5, PITCH_LINE_NARROW, false},
        { 40.0, PITCH_LINE_WIDE, true },  { 37.5, PITCH_LINE_NARROW, false}, { 35.0, PITCH_LINE_MEDIUM, false}, { 32.5, PITCH_LINE_NARROW, false},
        { 30.0, PITCH_LINE_WIDE, true },  { 27.5, PITCH_LINE_NARROW, false}, { 25.0, PITCH_LINE_MEDIUM, false}, { 22.5, PITCH_LINE_NARROW, false},
        { 20.0, PITCH_LINE_WIDE, true },  { 17.5, PITCH_LINE_NARROW, false}, { 15.0, PITCH_LINE_MEDIUM, false}, { 12.5, PITCH_LINE_NARROW, false},
        { 10.0, PITCH_LINE_WIDE, true },  {  7.5, PITCH_LINE_NARROW, false}, {  5.0, PITCH_LINE_MEDIUM, false}, {  2.5, PITCH_LINE_NARROW, false},
        { -2.5, PITCH_LINE_NARROW, false},  { -5.0, PITCH_LINE_MEDIUM, false}, { -7.5, PITCH_LINE_NARROW, false}, {-10.0, PITCH_LINE_WIDE, true },
        {-12.5, PITCH_LINE_NARROW, false},  {-15.0, PITCH_LINE_MEDIUM, false}, {-17.5, PITCH_LINE_NARROW, false}, {-20.0, PITCH_LINE_WIDE, true },
        {-22.5, PITCH_LINE_NARROW, false},  {-25.0, PITCH_LINE_MEDIUM, false}, {-27.5, PITCH_LINE_NARROW, false}, {-30.0, PITCH_LINE_WIDE, true },
        {-32.5, PITCH_LINE_NARROW, false},  {-35.0, PITCH_LINE_MEDIUM, false}, {-37.5, PITCH_LINE_NARROW, false}, {-40.0, PITCH_LINE_WIDE, true },
        {-42.5, PITCH_LINE_NARROW, false},  {-45.0, PITCH_LINE_MEDIUM, false}, {-47.5, PITCH_LINE_NARROW, false}, {-50.0, PITCH_LINE_WIDE, true },
        {-52.5, PITCH_LINE_NARROW, false},  {-55.0, PITCH_LINE_MEDIUM, false}, {-57.5, PITCH_LINE_NARROW, false}, {-60.0, PITCH_LINE_WIDE, true },
        {-62.5, PITCH_LINE_NARROW, false},  {-65.0, PITCH_LINE_MEDIUM, false}, {-67.5, PITCH_LINE_NARROW, false}, {-70.0, PITCH_LINE_WIDE, true },
        {-72.5, PITCH_LINE_NARROW, false},  {-75.0, PITCH_LINE_MEDIUM, false}, {-77.5, PITCH_LINE_NARROW, false}, {-80.0, PITCH_LINE_WIDE, true },
        {-82.5, PITCH_LINE_NARROW, false},  {-85.0, PITCH_LINE_MEDIUM, false}, {-87.5, PITCH_LINE_NARROW, false}, {-90.0, PITCH_LINE_WIDE, true },
        // clang-format on
    };

    uint16_t color = TFT_RED;
    for (const auto &line : pitch_lines) {
        if (line.deg > isisState.pitchAngle + 15.0f) continue;
        if (line.deg < isisState.pitchAngle - 17.5f) continue;
        float degFromCenter = fabsf(line.deg - isisState.pitchAngle);

        float verticalPos = degFromCenter * PITCH_SCALE;
        color             = TFT_WHITE;
        if (verticalPos > 0 && verticalPos < attSprite.height()) {
            drawPitchLine(line.deg, line.width, line.num, color);
        }
    }

    // -- Draw slip/skid and poitner. 
    slipSprite.fillSprite(TFT_MAGENTA);
    slipSprite.pushImage(slipSprite.width()/2 - ROLLPOINTER_IMG_WIDTH/2, 0, ROLLPOINTER_IMG_WIDTH, ROLLPOINTER_IMG_HEIGHT, ROLLPOINTER_IMG_DATA, 8184);
    slipSprite.pushImage(min((int)(slipSprite.width() - ROLLSLIP_IMG_WIDTH),  max(0, (int)(slipSprite.width()/2 - ROLLSLIP_IMG_WIDTH/2 + (0.7 * isisState.ballPos)*ROLLSLIP_IMG_WIDTH))), ROLLPOINTER_IMG_HEIGHT, ROLLSLIP_IMG_WIDTH, ROLLSLIP_IMG_HEIGHT, ROLLSLIP_IMG_DATA, 8184);
    attSprite.setPivot(attSprite.width()/2 - 12, ATT_HORIZON);
    slipSprite.pushRotated(isisState.bankAngle, TFT_MAGENTA);



    // --- 3. Draw Horizon Line ---
    // The horizon is just a pitch line at 0 degrees.
    // We draw it extra long to ensure it always crosses the screen.
    float horiz_unrot_y = (0 - isisState.pitchAngle) * PITCH_SCALE;
    float lineLength    = attSprite.width() * 1.5;

    int16_t hx1 = CENTER_X + (-lineLength / 2.0) * cosBank - horiz_unrot_y * sinBank;
    int16_t hy1 = CENTER_Y + (-lineLength / 2.0) * sinBank + horiz_unrot_y * cosBank;
    int16_t hx2 = CENTER_X + (lineLength / 2.0) * cosBank - horiz_unrot_y * sinBank;
    int16_t hy2 = CENTER_Y + (lineLength / 2.0) * sinBank + horiz_unrot_y * cosBank;

    attSprite.drawLine(hx1, hy1, hx2, hy2, HORIZON_COLOR);
    attSprite.drawLine(hx1, hy1 + 1, hx2, hy2 + 1, HORIZON_COLOR); // Thicker line
}

void CC_ISIS::drawSpeedTape()
{
    // Speed tape is drastically simpler than the core G5. Only a scrolling speed tape with no digit readout or rolling numbers
    // Speed shown in 10's with tiny hash marks at 5knots below 250, small 10kt hash marks and tiny marks with number at the 20s

    // I measure 153px for 40 kt = 3.8px/kt Let's call it 4.
    // The screen can show about 85kt at one time.

    const float pixPerKt = 3.8f;

    const float curSpeed = max(isisState.airspeed, 30.0f);
    const int   yOffset  = (int)(pixPerKt * fmodf(curSpeed, 20.0f)) - 35; // offset factor for non-centered arrow.
    const int   first20  = (int)(curSpeed / 20.0f) * 20;

    const int speedMarkWidth = 6;

    speedSprite.fillSprite(TFT_BLACK);
    speedSprite.setColor(TFT_WHITE);

    int curY = +yOffset;

    //    Serial.printf("CurY: %d  first20: %d yOffset: %d\n", curY, first20, yOffset);

    for (int i = first20 + 60; i > first20 - 60; i -= 5) {

        if (i < 30) continue;

        if (i % 20 == 0) {
            speedSprite.drawNumber(i, speedSprite.width() - 6, curY);
            speedSprite.drawWideLine(70, curY, 75, curY, 1, TFT_WHITE);
            // speedSprite.drawFastHLine(speedSprite.width() - 12, curY, 5);
            // speedSprite.drawFastHLine(speedSprite.width() - 12, curY+1, 5);
        } else if (i % 10 == 0) {
            speedSprite.drawWideLine(65, curY, 75, curY, 1, TFT_WHITE);
            // speedSprite.drawFastHLine(speedSprite.width() - 16, curY, 10);
            // speedSprite.drawFastHLine(speedSprite.width() - 16, curY+1, 10);
        } else if (i <= 245) {
            speedSprite.drawWideLine(70, curY, 75, curY, 1, TFT_WHITE);
            // speedSprite.drawFastHLine(speedSprite.width() - 12, curY, 5);
            // speedSprite.drawFastHLine(speedSprite.width() - 12, curY+1, 5);
        }

        curY += (int)(pixPerKt * 5); // pixPerKt * 5
    }

    speedSprite.pushSprite(0, ATT_TOP_EDGE);
}

void CC_ISIS::drawAltTape()
{

    // Value display 20' increments but scrolls to single digit. Shows 5 digigs of alt (i.e. up to 99,980').
    // Vertical Tape shows FL (100's of feet) at 500' increments (200, 205, 210 etc)
    //  Which is also in the left (wide) part of the digit box
    // The tall (right) part of the digit box shows 0-99 feet
    //
    // Hash marks every 100' on the tape. Labels every 500' Values as three digits with leading 0s

    bool  isNeg  = (isisState.altitude < 0.0f);
    float curAlt = fabsf(isisState.altitude); // always non-negative; display NEG indicator for sub-sea-level
    int   fl     = (int)(curAlt / 100);     // hundreds and above (e.g. 1234ft → fl=12)

    char buf[8];

    // Clear Sprites
    altSprite.fillSprite(TFT_BLACK);
    alt100Sprite.fillSprite(TFT_BLACK);
    alt100Sprite.pushImage(0, 0, ALTBG_IMG_WIDTH, ALTBG_IMG_HEIGHT, ALTBG_IMG_DATA, 8184); // MagentaRGB

    // Compute the scroll state once; everything below uses these values.
    // dispUnit: the current 20-ft band label (changes only at band boundaries).
    // sub20:    position within that band (0..20, continuous — never jumps).
    int   dispUnit = (int)(curAlt / 20) * 20;
    float sub20    = fmodf(curAlt, 20.0f);

    // nearRoll: true during the last 20-ft band of each 100-ft block (the 80-99 range).
    // rollFraction: 0.0→1.0 progress through that roll-over window.
    bool  nearRoll     = (dispUnit % 100 >= 80);
    float rollFraction = nearRoll ? (sub20 / 20.0f) : 0.0f;

    // --- Thousands-and-above digits (drawn directly on attSprite with clip rect) ---
    // Mirrors old flSprite: clip area (260, 178, 60, 33) in attSprite coords.
    // Each digit column animates independently — ten-thousands only rolls when
    // thousands is 9→0, fixing the "1 scrolls unnecessarily" bug.
    // offset spans the full clip height so digits fully exit before rollFraction=1.
    {
        const int clipX = attSprite.width() - 60; // 260
        const int clipY = ATT_HORIZON - 20;       // 178
        const int clipW = 60;
        const int clipH = 37;
        const int centY = clipY + clipH / 2 + 2; // 194 = ATT_HORIZON

        bool nearRollK  = nearRoll && ((fl % 10) == 9);
        bool nearRollTK = nearRollK && ((fl / 10) % 10 == 9);

        int offsetK  = nearRollK ? (int)(rollFraction * clipH) : 0 + 2;
        int offsetTK = nearRollTK ? (int)(rollFraction * clipH) : 0 + 2;

        attSprite.fillRect(clipX, clipY, clipW, clipH, TFT_BLACK);
        attSprite.setClipRect(clipX, clipY, clipW, clipH);
        attSprite.setTextSize(1.2f);
        attSprite.setTextColor(TFT_GREEN);
        attSprite.setTextDatum(CR_DATUM);

        // Ten-thousands digit (left column, ~x=300). Only shown at >= 10,000 ft.
        if (nearRollTK) {
            attSprite.drawNumber(fl / 100 + 1, clipX + 28, centY + offsetTK - clipH);
        }
        if (fl / 100 > 0) {
            attSprite.drawNumber(fl / 100, clipX + 28, centY + offsetTK);
        }

        // Thousands digit (right column, x=318). Only shown at >= 1,000 ft.
        if (nearRollK) {
            attSprite.drawNumber(((fl / 10) % 10 + 1) % 10, clipX + clipW - 2, centY + offsetK - clipH);
        }
        if (fl / 10 > 0) {
            attSprite.drawNumber((fl / 10) % 10, clipX + clipW - 2, centY + offsetK);
        }

        attSprite.clearClipRect();
        attSprite.setTextSize(1.0f);
        attSprite.setTextColor(TFT_WHITE);
        attSprite.setTextDatum(CC_DATUM);
    }

    // --- Hundreds digit (alt100Sprite, left column at x=24) ---
    // Rolls during the last 20-ft band of each 100-ft block.
    // As altitude increases: current digit descends (exits bottom), next enters from top.
    {
        const int centY  = alt100Sprite.height() / 2 + 3; // 34
        int       offset = (int)(rollFraction * centY);
        alt100Sprite.setTextSize(1.2f);
        alt100Sprite.setClipRect(0, 13, ALTBG_IMG_WIDTH, 38);
        if (nearRoll) {
            // Next hundreds digit descends from above
            alt100Sprite.drawNumber((fl % 10 + 1) % 10, 24, offset);
        }
        alt100Sprite.drawNumber(fl % 10, 24, centY + offset);
        alt100Sprite.clearClipRect();
    }

    // --- 20-ft scroll (alt100Sprite, right column at x=70) ---
    // curY: Y centre of the current label; starts at sprite centre (32) and descends
    // (increases) as altitude increases, so higher values enter from above.
    const float pxPerFt = 50.0f / 20.0f;               // 3 px per foot
    int         curY    = (int)(34 + sub20 * pxPerFt); // 32..92 as sub20 goes 0→19

    alt100Sprite.setTextSize(1.0f);
    sprintf(buf, "%02d", dispUnit % 100); // current label, descends off bottom
    alt100Sprite.drawString(buf, 70, curY);
    sprintf(buf, "%02d", (dispUnit + 20) % 100); // next-higher, enters from above
    alt100Sprite.drawString(buf, 70, curY - (pxPerFt * 20.0f));

    // Show the NEG indicator on the screen if altitude < 0
    if (isNeg) {
        attSprite.drawString("N", 269, 208 - 56);
        attSprite.drawString("G", 269, 292 - 56);
        // "E" sits where the old flSprite was: CR_DATUM at (278, ATT_HORIZON)
        attSprite.setTextDatum(CR_DATUM);
        attSprite.drawString("E", 278, ATT_HORIZON);
        attSprite.setTextDatum(CC_DATUM);
    }

    // Draw the tape.
    // Each tick's y is computed directly from its altitude relative to curAlt,
    // so scrolling is pixel-smooth with no fmodf rollover artifacts.
    //   y = referenceY + (curAlt - tickAlt) * pixPerFt
    // Higher altitudes (tickAlt > curAlt) produce smaller y (higher on screen). ✓
    const float pixPerFt    = 0.245f;
    const float referenceY  = 179.0f; // y in altSprite where the current-alt tick sits. Found with trial and error
    const int   markerWidth = 15;
    altSprite.setTextDatum(CL_DATUM);

    int baseTick = (int)floorf(curAlt / 100.0f) * 100; // nearest 100-ft band below curAlt
    for (int alt = baseTick + 900; alt >= baseTick - 900; alt -= 100) {
        curY = (int)(referenceY + (curAlt - alt) * pixPerFt);
        if (curY < 0 || curY >= altSprite.height()) continue;

        altSprite.drawWideLine(2, curY, 2 + markerWidth, curY, 1, TFT_WHITE);
        if (alt % 500 == 0) {
            sprintf(buf, "%03d", abs(alt) / 100); // absolute value — NEG indicator handles sign
            altSprite.drawString(buf, 10, curY + 2);
        }
    }

    alt100Sprite.pushSprite(0, ATT_HORIZON - alt100Sprite.height() / 2 - 1);
    altSprite.pushSprite(ALT_LEFT_EDGE - 2, ATT_TOP_EDGE);

    return;
}

void CC_ISIS::drawBackground()
{
    // Draw the yellow background markers.
    // bgSprite.pushSprite(0, ATT_TOP_EDGE + ATT_HORIZON + 33, TFT_MAGENTA);  // 33 is tip of pointer from top of bgSprite

    attSprite.pushImage(2, ATT_HORIZON - 23, ATTBACKGROUND_IMG_WIDTH, ATTBACKGROUND_IMG_HEIGHT, ATTBACKGROUND_IMG_DATA, 8184);
    attSprite.pushImage(6, 4, ROLLARC_IMG_WIDTH, ROLLARC_IMG_HEIGHT, ROLLARC_IMG_DATA, 8184);

    // Draw the curves top and bottom of the gauge.
    blackoutArcSprite.pushSprite(0, 0, TFT_MAGENTA);
    attSprite.setPivot(attSprite.width() / 2, attSprite.height() - blackoutArcSprite.height() / 2);
    blackoutArcSprite.pushRotated(180.0f, TFT_MAGENTA);
}

void CC_ISIS::drawPressure()
{
    kohlsSprite.fillSprite(TFT_BLACK);
    kohlsSprite.setTextColor(TFT_BLUE);
    if (isisState.isStdPressure) {
        kohlsSprite.setTextSize(1.2);
        kohlsSprite.drawString("STD", 1, 1);
    } else {
        kohlsSprite.setTextSize(1.0);
        kohlsSprite.drawNumber(isisState.mbPressure, 1, 1);
    }
    kohlsSprite.pushSprite(140, 420);
}

void CC_ISIS::drawMach() {
    if(isisState.machSpeed < 0.45) {
        kohlsSprite.fillSprite(TFT_BLACK);
        kohlsSprite.pushSprite(20, 420); 
        return;
    }

    char buf[5];

    kohlsSprite.fillSprite(TFT_BLACK);
    kohlsSprite.setTextColor(TFT_GREEN);
    kohlsSprite.setTextSize(1.0);
    sprintf(buf, "%.2f", isisState.machSpeed);
    char *s = buf;
    // 2. Remove leading 0 if present (handles 0.49 -> .49)
    if (s[0] == '0' && s[1] == '.') {
        s++;
    }
    kohlsSprite.drawString(s, 1, 1);
    kohlsSprite.pushSprite(20, 420); 
}
// ---------------------------------------------------------------------------
// Landing System (Runde 7)
//
// Aufbau nach dem echten ISIS: zwei schwarze Skalenbaender mit weissen Ringen,
// je ein magenta Diamant als Ablagezeiger. Die Gleitpfad-Skala steht senkrecht
// rechts, die Localizer-Skala waagerecht unten, beide mittig zur Anzeige.
// Die Bilder stammen aus CC_ISIS/Sprites (gsDeviation, 90-Grad-Variante daraus).
// ---------------------------------------------------------------------------
#define LS_GRAU            0x18C3   // Hintergrundfarbe der Skalen-Assets
#define LS_GS_X            348      // linke Kante der senkrechten Skala (84 % Breite,
                                    // direkt links neben der ersten Ziffer der Hoehenanzeige;
                                    // die Ziffern beginnen bei x=374)
#define LS_LOC_TOP         360      // Oberkante der waagerechten Skala (75 % Hoehe,
                                    // noch im braunen Boden, Braun reicht bis 83 %)
#define LS_DEV_FULL        53       // Pixel pro Dot (Skala deckt +-2 Dots ab:
                                    // 106 px halbe Skalenlaenge / 2)

// Zur Laufzeit veraenderbar (Konsolenbefehl LSPOS), damit die Lage am echten
// Geraet einjustiert werden kann. Startwerte = abgeleitete Geometrie.
static int g_lsGsX      = LS_GS_X;
static int g_lsLocTop   = LS_LOC_TOP;
static int g_lsDevFull  = LS_DEV_FULL;
static int g_lsLocX     = 0;      // 0 = automatisch mittig zum Attitude-Fenster

void isisSetLsLayout(int gsX, int locTop, int devFull, int locX)
{
    if (gsX  >= 0 && gsX  <= 478) g_lsGsX     = gsX;
    if (locTop >= 0 && locTop <= 478) g_lsLocTop = locTop;
    if (devFull >= 20 && devFull <= 140) g_lsDevFull = devFull;
    if (locX >= 0 && locX <= 478) g_lsLocX = locX;
    Serial.printf("[isis] LSPOS gsX=%d locTop=%d full=%d locX=%d\n",
                  g_lsGsX, g_lsLocTop, g_lsDevFull, g_lsLocX);
}

void CC_ISIS::drawLS()
{
    if (!isisState.lsActive) return;

    const int16_t centerY = ATT_TOP_EDGE + ATT_HORIZON;   // Horizontlinie

    // Airbus-Verhalten: mit LS-Taste erscheinen BEIDE Skalen sofort (grau, ohne
    // Signal). Der Diamant kommt erst mit gueltigem Empfang dazu. Vorher haengen
    // Skala UND Diamant an lsGsValid/lsLocValid — ohne getuntes ILS blieb der
    // Schirm also leer, obwohl LS sauber ankam ("ISIS ignoriert LS", 23.10.2026).

    // ---- Gleitpfad: senkrechte Skala rechts, Diamant laeuft hoch/runter
    lcd.pushImage(g_lsGsX, centerY - GSDEVIATION_IMG_HEIGHT / 2,
                  GSDEVIATION_IMG_WIDTH, GSDEVIATION_IMG_HEIGHT,
                  GSDEVIATION_IMG_DATA, LS_GRAU);

    if (isisState.lsGsValid) {
        // Anschlag ist die halbe Skalenlaenge, NICHT g_lsDevFull: der Wert ist
        // Pixel pro Dot, der Weg bis zum Skalenende ist halbe Laenge minus
        // halber Diamant. Sonst klebt der Diamant schon bei einem Dot.
        const int16_t grenzeGs = (GSDEVIATION_IMG_HEIGHT - DEVIATIONDIAMOND_IMG_HEIGHT) / 2;
        int16_t versatz = (int16_t)lroundf(isisState.lsGsDev * g_lsDevFull);
        if (versatz >  grenzeGs) versatz =  grenzeGs;
        if (versatz < -grenzeGs) versatz = -grenzeGs;
        lcd.pushImage(g_lsGsX + (GSDEVIATION_IMG_WIDTH - DEVIATIONDIAMOND_IMG_WIDTH) / 2,
                      centerY + versatz - DEVIATIONDIAMOND_IMG_HEIGHT / 2,
                      DEVIATIONDIAMOND_IMG_WIDTH, DEVIATIONDIAMOND_IMG_HEIGHT,
                      DEVIATIONDIAMOND_IMG_DATA, LS_GRAU);
    }

    // ---- Localizer: waagerechte Skala unten, Diamant laeuft links/rechts
    const int16_t locMitte = g_lsLocX ? (int16_t)g_lsLocX
                                      : (int16_t)(ATT_LEFT_EDGE + ATT_WIDTH / 2);
    lcd.pushImage(locMitte - GSDEVIATIONH_IMG_WIDTH / 2, g_lsLocTop,
                  GSDEVIATIONH_IMG_WIDTH, GSDEVIATIONH_IMG_HEIGHT,
                  GSDEVIATIONH_IMG_DATA, LS_GRAU);

    if (isisState.lsLocValid) {
        const int16_t grenzeLoc = (GSDEVIATIONH_IMG_WIDTH - DEVIATIONDIAMOND_IMG_WIDTH) / 2;
        int16_t versatz = (int16_t)lroundf(isisState.lsLocDev * g_lsDevFull);
        if (versatz >  grenzeLoc) versatz =  grenzeLoc;
        if (versatz < -grenzeLoc) versatz = -grenzeLoc;
        lcd.pushImage(locMitte + versatz - DEVIATIONDIAMOND_IMG_WIDTH / 2,
                      g_lsLocTop + (GSDEVIATIONH_IMG_HEIGHT - DEVIATIONDIAMOND_IMG_HEIGHT) / 2,
                      DEVIATIONDIAMOND_IMG_WIDTH, DEVIATIONDIAMOND_IMG_HEIGHT,
                      DEVIATIONDIAMOND_IMG_DATA, LS_GRAU);
    }
}

void CC_ISIS::draw()
{
    drawAttitude();
    drawBackground();
    drawSpeedTape();
    drawAltTape(); // NOTE: Alt tape writes on the attSprite.
    attSprite.pushSprite(ATT_LEFT_EDGE, ATT_TOP_EDGE);
    drawPressure();
    drawMach();
    drawLS();      // Overlay zuletzt - die bestehende Grafik bleibt unberuehrt.
}

void CC_ISIS::updateInputValues()
{
    // Smooth raw input values toward current display values each frame,
    // giving fluid motion instead of stepping directly to the new value.
    // Alpha controls smoothing speed (higher = faster); threshold snaps to
    // the target once the gap is small enough to avoid endless micro-steps.
    isisState.pitchAngle = smoothInput(isisState.rawPitchAngle, isisState.pitchAngle, 0.3f, 0.05f);
    isisState.bankAngle  = smoothAngle(isisState.rawBankAngle, isisState.bankAngle, 0.3f, 0.05f);
    isisState.airspeed   = smoothInput(isisState.rawAirspeed, isisState.airspeed, 0.1f, 0.005f);
    isisState.altitude   = smoothInput(isisState.rawAltitude, isisState.altitude, 0.1f, 0.05f);
}

void CC_ISIS::update()
{
    // static unsigned long lastDelta = 0;

    updateInputValues();
    draw();

    /*
    char buf[80];
    lcd.fillRect(20, 0, 480, 25);
    sprintf(buf, "p:%.1f b:%.1f a:%.1f a:%.0f", isisState.pitchAngle, isisState.bankAngle, isisState.airspeed, isisState.altitude);
    lcd.setTextSize(2.0f);
    lcd.drawString(buf, 20, 20);
    if (millis() > lastDelta + 500) {
        isisState.rawBankAngle += 0.5;
        if (isisState.rawBankAngle > 30) isisState.rawBankAngle = -30.0f;
        lastDelta = millis();

        isisState.rawAirspeed += 1.0f;
        if (isisState.rawAirspeed > 500.0f) isisState.rawAirspeed = 0.0f;

        //        isisState.rawAltitude += 10.0f;
        if (isisState.rawAltitude > 15000.0f) isisState.rawAltitude = 0.0f;
    }
        */
}
