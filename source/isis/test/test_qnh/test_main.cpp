// Host unit tests: QNH settle logic (control-loop trap), BARO acceleration and
// the >5 s keepalive watchdog (ISIS_FIRMWARE_SPEC.md sections 4 and 5).
#include <unity.h>

#include "proto.h"

void setUp(void) {}
void tearDown(void) {}

// ---------------------------------------------------------------------------
// QnhSettle: while the firmware owns the value, incoming dsp4 must be ignored.
// ---------------------------------------------------------------------------
static void test_remote_accepted_without_local_change(void)
{
    QnhSettle settle;
    TEST_ASSERT_TRUE(settle.remoteAccepted(0));
    TEST_ASSERT_TRUE(settle.remoteAccepted(123456));
    TEST_ASSERT_FALSE(settle.localActive());
    TEST_ASSERT_EQUAL_UINT32(1000, settle.settleMs());
}

static void test_local_change_blocks_remote_for_one_second(void)
{
    QnhSettle settle;

    settle.onLocalChange(5000);

    TEST_ASSERT_TRUE(settle.localActive());
    TEST_ASSERT_FALSE(settle.remoteAccepted(5000));   // same instant
    TEST_ASSERT_FALSE(settle.remoteAccepted(5500));   // 500 ms later
    TEST_ASSERT_FALSE(settle.remoteAccepted(5999));   // just inside the window
    TEST_ASSERT_TRUE(settle.remoteAccepted(6000));    // exactly 1 s -> take over
    TEST_ASSERT_TRUE(settle.remoteAccepted(9000));

    // still "localActive" but no longer blocking
    TEST_ASSERT_TRUE(settle.remoteAccepted(6001));
}

static void test_settle_window_survives_millis_overflow(void)
{
    QnhSettle settle;
    // local change shortly before the 32 bit millis() rollover
    const uint32_t nearMax = 0xFFFFFF00u;
    settle.onLocalChange(nearMax);
    TEST_ASSERT_FALSE(settle.remoteAccepted(nearMax + 100u)); // still inside
    TEST_ASSERT_TRUE(settle.remoteAccepted(nearMax + 1000u)); // window over
}

static void test_settle_window_is_configurable(void)
{
    QnhSettle settle;
    settle.setSettleMs(250);
    TEST_ASSERT_EQUAL_UINT32(250, settle.settleMs());
    settle.onLocalChange(1000);
    TEST_ASSERT_FALSE(settle.remoteAccepted(1249));
    TEST_ASSERT_TRUE(settle.remoteAccepted(1250));
}

// ---------------------------------------------------------------------------
// BaroRotation: slow turn = 1 mb per detent, fast turn = 10 mb.
// ---------------------------------------------------------------------------
static void test_baro_acceleration(void)
{
    BaroRotation rot;

    // first detent always 1 mb
    TEST_ASSERT_EQUAL_INT(1, rot.step(+1, 0));
    // a fast follow-up (inside 150 ms) jumps by 10 mb
    TEST_ASSERT_EQUAL_INT(10, rot.step(+1, 100));
    TEST_ASSERT_EQUAL_INT(10, rot.step(+1, 200));
    // a slow follow-up falls back to 1 mb
    TEST_ASSERT_EQUAL_INT(1, rot.step(+1, 1000));
}

static void test_baro_rotation_direction(void)
{
    BaroRotation rot;
    TEST_ASSERT_EQUAL_INT(1, rot.step(+1, 0));
    TEST_ASSERT_EQUAL_INT(-1, rot.step(-1, 900));
    TEST_ASSERT_EQUAL_INT(-10, rot.step(-1, 950));
}

static void test_baro_rotation_reset(void)
{
    BaroRotation rot;
    rot.step(+1, 0);
    rot.reset();
    TEST_ASSERT_EQUAL_INT(1, rot.step(+1, 50)); // no acceleration after reset
}

// ---------------------------------------------------------------------------
// DataWatchdog: > 5 s without data -> defined state (offline edge fires once).
// ---------------------------------------------------------------------------
static void test_watchdog_offline_behaviour(void)
{
    DataWatchdog wd;

    TEST_ASSERT_FALSE(wd.online(0));
    TEST_ASSERT_FALSE(wd.everHadData());

    wd.onData(1000);
    TEST_ASSERT_TRUE(wd.online(1000));
    TEST_ASSERT_TRUE(wd.online(5999));
    TEST_ASSERT_FALSE(wd.online(6000)); // 5 s threshold reached

    TEST_ASSERT_FALSE(wd.offlineEdge(5999));
    TEST_ASSERT_TRUE(wd.offlineEdge(6000));  // fires exactly once ...
    TEST_ASSERT_FALSE(wd.offlineEdge(6001)); // ... and not again

    // data comes back -> online again, edge re-armed
    wd.onData(7000);
    TEST_ASSERT_TRUE(wd.online(7000));
    TEST_ASSERT_FALSE(wd.offlineEdge(8000));
    TEST_ASSERT_TRUE(wd.offlineEdge(12000));
}

static void test_watchdog_threshold_is_configurable(void)
{
    DataWatchdog wd;
    wd.setOfflineMs(2000);
    TEST_ASSERT_EQUAL_UINT32(2000, wd.offlineMs());
    wd.onData(0);
    TEST_ASSERT_TRUE(wd.online(1999));
    TEST_ASSERT_FALSE(wd.online(2000));
}

int main(int, char **)
{
    UNITY_BEGIN();
    RUN_TEST(test_remote_accepted_without_local_change);
    RUN_TEST(test_local_change_blocks_remote_for_one_second);
    RUN_TEST(test_settle_window_survives_millis_overflow);
    RUN_TEST(test_settle_window_is_configurable);
    RUN_TEST(test_baro_acceleration);
    RUN_TEST(test_baro_rotation_direction);
    RUN_TEST(test_baro_rotation_reset);
    RUN_TEST(test_watchdog_offline_behaviour);
    RUN_TEST(test_watchdog_threshold_is_configurable);
    return UNITY_END();
}
