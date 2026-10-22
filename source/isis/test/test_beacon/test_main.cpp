// Host unit tests: uplink/beacon frame construction
// (ISIS_FIRMWARE_SPEC.md section 4, IP_TRANSPORT_SPEC.md section 4).
#include <unity.h>
#include <string.h>

#include "proto.h"

void setUp(void) {}
void tearDown(void) {}

// ---------------------------------------------------------------------------
static void test_beacon_frame_matches_the_spec(void)
{
    char buf[128];
    const int n = protoBuildBeacon(buf, sizeof(buf), "AA:BB:CC:DD:EE:FF",
                                   "192.168.20.71", "1.0.3");

    TEST_ASSERT_EQUAL_STRING("IDENT:ISIS;MAC:AA:BB:CC:DD:EE:FF;IP:192.168.20.71;"
                             "FW:1.0.3;HW:ESP32S3;",
                             buf);
    TEST_ASSERT_EQUAL_INT((int)strlen(buf), n);
    // the FDC parses the firmware version out of the beacon
    TEST_ASSERT_NOT_NULL(strstr(buf, "FW:1.0.3;"));
    TEST_ASSERT_NOT_NULL(strstr(buf, "HW:ESP32S3;"));
}

static void test_beacon_null_safe(void)
{
    char buf[128];
    protoBuildBeacon(buf, sizeof(buf), nullptr, nullptr, nullptr);
    TEST_ASSERT_NOT_NULL(strstr(buf, "IDENT:ISIS;"));
    TEST_ASSERT_NOT_NULL(strstr(buf, "HW:ESP32S3;"));
}

static void test_beacon_detects_short_buffer(void)
{
    char buf[8];
    TEST_ASSERT_EQUAL_INT(-1, protoBuildBeacon(buf, sizeof(buf), "AA:BB:CC:DD:EE:FF",
                                              "10.0.0.5", "1.0.0"));
    TEST_ASSERT_EQUAL_STRING("", buf);
}

// ---------------------------------------------------------------------------
static void test_qnhset_frame(void)
{
    char buf[32];
    TEST_ASSERT_EQUAL_STRING("QNHSET:1013;", (protoBuildQnhSet(buf, sizeof(buf), 1013), buf));

    protoBuildQnhSet(buf, sizeof(buf), 9999);
    TEST_ASSERT_EQUAL_STRING("QNHSET:9999;", buf);

    // always in mb (spec section 5), even if the display shows inHg
    protoBuildQnhSet(buf, sizeof(buf), 1013);
    TEST_ASSERT_EQUAL_STRING("QNHSET:1013;", buf);
}

static void test_button_frame(void)
{
    char buf[32];
    protoBuildBtn(buf, sizeof(buf), "BUGS", 1);
    TEST_ASSERT_EQUAL_STRING("BTN:BUGS:1;", buf);

    protoBuildBtn(buf, sizeof(buf), "BUGS", 0);
    TEST_ASSERT_EQUAL_STRING("BTN:BUGS:0;", buf);

    protoBuildBtn(buf, sizeof(buf), "LS", 2);
    TEST_ASSERT_EQUAL_STRING("BTN:LS:1;", buf); // normalised to 0/1
}

static void test_encoder_frame_keeps_original_direction_encoding(void)
{
    char buf[32];
    // 0 = increase, 2 = decrease - identical to the MobiFlight encoding
    protoBuildEncoder(buf, sizeof(buf), "BARO", 0);
    TEST_ASSERT_EQUAL_STRING("ENC:BARO:0;", buf);
    protoBuildEncoder(buf, sizeof(buf), "BARO", 2);
    TEST_ASSERT_EQUAL_STRING("ENC:BARO:2;", buf);
}

static void test_ota_frames(void)
{
    char buf[32];
    protoBuildOtaProgress(buf, sizeof(buf), 42);
    TEST_ASSERT_EQUAL_STRING("OTA:progress=42;", buf);

    protoBuildOtaProgress(buf, sizeof(buf), 500);
    TEST_ASSERT_EQUAL_STRING("OTA:progress=100;", buf);

    protoBuildOtaDone(buf, sizeof(buf));
    TEST_ASSERT_EQUAL_STRING("OTA:done;", buf);
}

static void test_builders_signal_short_buffers(void)
{
    char small[6];
    TEST_ASSERT_EQUAL_INT(-1, protoBuildQnhSet(small, sizeof(small), 1013));
    TEST_ASSERT_EQUAL_INT(-1, protoBuildBtn(small, sizeof(small), "BAROPUSH", 1));
    TEST_ASSERT_EQUAL_INT(-1, protoBuildOtaDone(small, sizeof(small)));
    // NULL name must not crash
    TEST_ASSERT_EQUAL_INT(-1, protoBuildBtn(small, sizeof(small), nullptr, 1));
}

int main(int, char **)
{
    UNITY_BEGIN();
    RUN_TEST(test_beacon_frame_matches_the_spec);
    RUN_TEST(test_beacon_null_safe);
    RUN_TEST(test_beacon_detects_short_buffer);
    RUN_TEST(test_qnhset_frame);
    RUN_TEST(test_button_frame);
    RUN_TEST(test_encoder_frame_keeps_original_direction_encoding);
    RUN_TEST(test_ota_frames);
    RUN_TEST(test_builders_signal_short_buffers);
    return UNITY_END();
}
