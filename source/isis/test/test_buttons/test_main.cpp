// Host unit tests: button bit mapping, 20 ms debounce, RST hold and the
// boot combo (ISIS_FIRMWARE_SPEC.md section 5).
#include <unity.h>

#include "buttons.h"

void setUp(void) {}
void tearDown(void) {}

// PCF8574 inputs are pulled up: a pressed key reads 0.
static uint8_t rawWithPressed(uint8_t pressedMask)
{
    return (uint8_t)(~pressedMask & 0xFFu);
}

// ---------------------------------------------------------------------------
static void test_bit_mapping_matches_the_ini_section(void)
{
    // b1..b8 -> bits 0..7
    TEST_ASSERT_EQUAL_UINT8(0x01, buttonsBit(BTN_BARO_MINUS));
    TEST_ASSERT_EQUAL_UINT8(0x02, buttonsBit(BTN_BARO_PLUS));
    TEST_ASSERT_EQUAL_UINT8(0x04, buttonsBit(BTN_BARO_PUSH));
    TEST_ASSERT_EQUAL_UINT8(0x08, buttonsBit(BTN_RST));
    TEST_ASSERT_EQUAL_UINT8(0x10, buttonsBit(BTN_BUGS));
    TEST_ASSERT_EQUAL_UINT8(0x20, buttonsBit(BTN_LS));
    TEST_ASSERT_EQUAL_UINT8(0x40, buttonsBit(BTN_BRT_MINUS));
    TEST_ASSERT_EQUAL_UINT8(0x80, buttonsBit(BTN_BRT_PLUS));
    TEST_ASSERT_EQUAL_UINT8(8, BTN_COUNT);
}

static void test_each_key_maps_to_exactly_one_button(void)
{
    for (uint8_t i = 0; i < BTN_COUNT; ++i) {
        const uint8_t raw  = rawWithPressed((uint8_t)(1u << i));
        const uint8_t mask = buttonsDownMask(raw);
        TEST_ASSERT_EQUAL_UINT8((uint8_t)(1u << i), mask);
        TEST_ASSERT_TRUE(buttonsIsDown(raw, (IsisButton)i));
        // no other key must be reported as pressed
        for (uint8_t j = 0; j < BTN_COUNT; ++j) {
            if (j == i) continue;
            TEST_ASSERT_FALSE(buttonsIsDown(raw, (IsisButton)j));
        }
    }
}

static void test_idle_port_reports_nothing_pressed(void)
{
    TEST_ASSERT_EQUAL_UINT8(0x00, buttonsDownMask(0xFF));
    for (uint8_t i = 0; i < BTN_COUNT; ++i) {
        TEST_ASSERT_FALSE(buttonsIsDown(0xFF, (IsisButton)i));
    }
}

static void test_all_keys_at_once(void)
{
    TEST_ASSERT_EQUAL_UINT8(0xFF, buttonsDownMask(0x00));
}

static void test_button_names_for_uplink_frames(void)
{
    TEST_ASSERT_EQUAL_STRING("BARO-", buttonsName(BTN_BARO_MINUS));
    TEST_ASSERT_EQUAL_STRING("BARO+", buttonsName(BTN_BARO_PLUS));
    TEST_ASSERT_EQUAL_STRING("BAROPUSH", buttonsName(BTN_BARO_PUSH));
    TEST_ASSERT_EQUAL_STRING("RST", buttonsName(BTN_RST));
    TEST_ASSERT_EQUAL_STRING("BUGS", buttonsName(BTN_BUGS));
    TEST_ASSERT_EQUAL_STRING("LS", buttonsName(BTN_LS));
    TEST_ASSERT_EQUAL_STRING("BRT-", buttonsName(BTN_BRT_MINUS));
    TEST_ASSERT_EQUAL_STRING("BRT+", buttonsName(BTN_BRT_PLUS));
    TEST_ASSERT_EQUAL_STRING("UNKNOWN", buttonsName((IsisButton)9));
}

// ---------------------------------------------------------------------------
static void test_boot_combo_needs_both_brightness_keys(void)
{
    // b7 + b8 pressed -> setup mode escape hatch
    TEST_ASSERT_TRUE(buttonsBootCombo(rawWithPressed(buttonsBit(BTN_BRT_MINUS) |
                                                     buttonsBit(BTN_BRT_PLUS))));
    // only one of them -> no combo
    TEST_ASSERT_FALSE(buttonsBootCombo(rawWithPressed(buttonsBit(BTN_BRT_PLUS))));
    TEST_ASSERT_FALSE(buttonsBootCombo(rawWithPressed(buttonsBit(BTN_BRT_MINUS))));
    // both plus others still counts
    TEST_ASSERT_TRUE(buttonsBootCombo(rawWithPressed(0xFF)));
    TEST_ASSERT_FALSE(buttonsBootCombo(0xFF));
}

// ---------------------------------------------------------------------------
static void test_debouncer_reports_one_edge_after_20ms(void)
{
    ButtonDebouncer db;
    TEST_ASSERT_EQUAL_UINT32(ISIS_BTN_DEBOUNCE_MS, db.debounceMs());

    TEST_ASSERT_EQUAL_UINT8(0x00, db.update(0xFF, 0));    // first sample is adopted
    TEST_ASSERT_FALSE(db.pressed(BTN_BARO_MINUS));

    TEST_ASSERT_EQUAL_UINT8(0x00, db.update(0xFE, 5));    // still bouncing
    TEST_ASSERT_EQUAL_UINT8(0x01, db.update(0xFE, 25));   // stable now
    TEST_ASSERT_TRUE(db.pressed(BTN_BARO_MINUS));

    TEST_ASSERT_EQUAL_UINT8(0x00, db.update(0xFE, 30));   // held -> no new edge

    TEST_ASSERT_EQUAL_UINT8(0x00, db.update(0xFF, 40));   // release: candidate
    TEST_ASSERT_EQUAL_UINT8(0x01, db.update(0xFF, 61));   // release: committed
    TEST_ASSERT_FALSE(db.pressed(BTN_BARO_MINUS));
}

static void test_debouncer_restarts_the_window_on_bounce(void)
{
    ButtonDebouncer db;
    db.update(0xFF, 0);

    TEST_ASSERT_EQUAL_UINT8(0x00, db.update(0xFE, 5));    // candidate starts
    TEST_ASSERT_EQUAL_UINT8(0x00, db.update(0xFF, 10));   // bounced back -> cleared
    TEST_ASSERT_EQUAL_UINT8(0x00, db.update(0xFE, 15));   // candidate restarts
    TEST_ASSERT_EQUAL_UINT8(0x00, db.update(0xFE, 30));   // 15 ms only
    TEST_ASSERT_EQUAL_UINT8(0x01, db.update(0xFE, 36));   // 21 ms -> stable
}

static void test_debouncer_prime_adopts_a_held_key(void)
{
    ButtonDebouncer db;
    db.prime(0xFE, 0);
    TEST_ASSERT_TRUE(db.primed());
    TEST_ASSERT_TRUE(db.pressed(BTN_BARO_MINUS));
    // no spurious edge for a key that was already held at boot
    TEST_ASSERT_EQUAL_UINT8(0x00, db.update(0xFE, 1000));
}

static void test_debouncer_multiple_keys(void)
{
    ButtonDebouncer db;
    db.update(0xFF, 0);

    const uint8_t raw = rawWithPressed(buttonsBit(BTN_BUGS) | buttonsBit(BTN_LS));
    TEST_ASSERT_EQUAL_UINT8(0x00, db.update(raw, 5));
    TEST_ASSERT_EQUAL_UINT8((uint8_t)(buttonsBit(BTN_BUGS) | buttonsBit(BTN_LS)),
                            db.update(raw, 26));
    TEST_ASSERT_TRUE(db.pressed(BTN_BUGS));
    TEST_ASSERT_TRUE(db.pressed(BTN_LS));
}

// ---------------------------------------------------------------------------
static void test_rst_needs_two_seconds(void)
{
    ButtonHold hold;
    TEST_ASSERT_EQUAL_UINT32(ISIS_BTN_RST_HOLD_MS, hold.holdMs());

    TEST_ASSERT_FALSE(hold.fired(true, 0));
    TEST_ASSERT_TRUE(hold.holding());
    TEST_ASSERT_FALSE(hold.fired(true, 1999));
    TEST_ASSERT_TRUE(hold.fired(true, 2000));   // fires ...
    TEST_ASSERT_FALSE(hold.fired(true, 5000));  // ... only once

    TEST_ASSERT_FALSE(hold.fired(false, 5010)); // release resets
    TEST_ASSERT_FALSE(hold.holding());
    TEST_ASSERT_FALSE(hold.fired(true, 5020));  // new press
    TEST_ASSERT_FALSE(hold.fired(true, 6000));  // 980 ms
    TEST_ASSERT_TRUE(hold.fired(true, 7020));   // 2 s reached
}

static void test_hold_shorter_than_two_seconds_never_fires(void)
{
    ButtonHold hold;
    hold.fired(true, 0);
    TEST_ASSERT_FALSE(hold.fired(false, 1500));
    TEST_ASSERT_FALSE(hold.fired(true, 1600));
}

int main(int, char **)
{
    UNITY_BEGIN();
    RUN_TEST(test_bit_mapping_matches_the_ini_section);
    RUN_TEST(test_each_key_maps_to_exactly_one_button);
    RUN_TEST(test_idle_port_reports_nothing_pressed);
    RUN_TEST(test_all_keys_at_once);
    RUN_TEST(test_button_names_for_uplink_frames);
    RUN_TEST(test_boot_combo_needs_both_brightness_keys);
    RUN_TEST(test_debouncer_reports_one_edge_after_20ms);
    RUN_TEST(test_debouncer_restarts_the_window_on_bounce);
    RUN_TEST(test_debouncer_prime_adopts_a_held_key);
    RUN_TEST(test_debouncer_multiple_keys);
    RUN_TEST(test_rst_needs_two_seconds);
    RUN_TEST(test_hold_shorter_than_two_seconds_never_fires);
    return UNITY_END();
}
