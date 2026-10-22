// Host unit tests: downlink frame parser (ISIS_FIRMWARE_SPEC.md section 3).
#include <unity.h>
#include <string.h>

#include "proto.h"

void setUp(void) {}
void tearDown(void) {}

// ---------------------------------------------------------------------------
static void test_dsp_index_mapping(void)
{
    TEST_ASSERT_EQUAL_INT(0, protoDspIndex("DSP1"));
    TEST_ASSERT_EQUAL_INT(1, protoDspIndex("DSP2"));
    TEST_ASSERT_EQUAL_INT(7, protoDspIndex("DSP8"));
    // phase-2 channels and unknown keys must not be mapped
    TEST_ASSERT_EQUAL_INT(-1, protoDspIndex("DSP9"));
    TEST_ASSERT_EQUAL_INT(-1, protoDspIndex("DSP10"));
    TEST_ASSERT_EQUAL_INT(-1, protoDspIndex("DSP0"));
    TEST_ASSERT_EQUAL_INT(-1, protoDspIndex("DISP_BL"));
    TEST_ASSERT_EQUAL_INT(-1, protoDspIndex("DSP"));
    TEST_ASSERT_EQUAL_INT(-1, protoDspIndex(""));
}

// The DSP -> message-ID table must be identical to CC_ISIS::set() (spec section 3).
static void test_dsp_message_ids_match_original(void)
{
    TEST_ASSERT_EQUAL_INT(60,  protoDspMessageId(ISIS_DSP_AIRSPEED));
    TEST_ASSERT_EQUAL_INT(102, protoDspMessageId(ISIS_DSP_MACH));
    TEST_ASSERT_EQUAL_INT(77,  protoDspMessageId(ISIS_DSP_ALTITUDE));
    TEST_ASSERT_EQUAL_INT(100, protoDspMessageId(ISIS_DSP_QNH));
    TEST_ASSERT_EQUAL_INT(101, protoDspMessageId(ISIS_DSP_STD));
    TEST_ASSERT_EQUAL_INT(80,  protoDspMessageId(ISIS_DSP_PITCH));
    TEST_ASSERT_EQUAL_INT(72,  protoDspMessageId(ISIS_DSP_BANK));
    TEST_ASSERT_EQUAL_INT(71,  protoDspMessageId(ISIS_DSP_BALL));
    TEST_ASSERT_EQUAL_INT(-1,  protoDspMessageId(8));

    // atoi() channels: QNH + STD, everything else atof() (like the original)
    TEST_ASSERT_TRUE(protoDspIsInt(ISIS_DSP_QNH));
    TEST_ASSERT_TRUE(protoDspIsInt(ISIS_DSP_STD));
    TEST_ASSERT_FALSE(protoDspIsInt(ISIS_DSP_AIRSPEED));
    TEST_ASSERT_FALSE(protoDspIsInt(ISIS_DSP_ALTITUDE));
    TEST_ASSERT_FALSE(protoDspIsInt(ISIS_DSP_BALL));
}

// ---------------------------------------------------------------------------
static void test_full_spec_frame(void)
{
    IsisDisplayValues v;
    IsisCfg            cfg;
    ProtoContext       ctx;

    const ProtoResult r = protoParseFrame(
        "SEQ42|DSP1:280;DSP2:0.78;DSP3:35000;DSP4:1013;DSP5:0;"
        "DSP6:-3.2;DSP7:12.5;DSP8:0.1;DISP_BL:180;",
        v, cfg, ctx);

    TEST_ASSERT_EQUAL_INT((int)ProtoStatus::OK, (int)r.status);
    TEST_ASSERT_TRUE(r.hasSeq);
    TEST_ASSERT_EQUAL_UINT32(42, r.seq);
    TEST_ASSERT_EQUAL_UINT16(0x00FF, r.dspMask);

    TEST_ASSERT_EQUAL_FLOAT(280.0f, v.airspeed);
    TEST_ASSERT_EQUAL_FLOAT(0.78f, v.mach);
    TEST_ASSERT_EQUAL_FLOAT(35000.0f, v.altitude);
    TEST_ASSERT_EQUAL_INT(1013, v.qnhMb);
    TEST_ASSERT_EQUAL_INT(0, v.std);
    TEST_ASSERT_EQUAL_FLOAT(-3.2f, v.pitch);
    TEST_ASSERT_EQUAL_FLOAT(12.5f, v.bank);
    TEST_ASSERT_EQUAL_FLOAT(0.1f, v.ball);

    TEST_ASSERT_TRUE(r.dispBl);
    TEST_ASSERT_EQUAL_INT(180, v.dispBl);
}

static void test_partial_frame_only_sets_present_channels(void)
{
    IsisDisplayValues v;
    IsisCfg            cfg;
    ProtoContext       ctx;

    const ProtoResult r = protoParseFrame("SEQ1|DSP3:12000;DSP5:1;", v, cfg, ctx);

    TEST_ASSERT_EQUAL_INT((int)ProtoStatus::OK, (int)r.status);
    TEST_ASSERT_EQUAL_UINT16((uint16_t)((1u << ISIS_DSP_ALTITUDE) | (1u << ISIS_DSP_STD)), r.dspMask);

    TEST_ASSERT_EQUAL_FLOAT(12000.0f, v.altitude);
    TEST_ASSERT_EQUAL_INT(1, v.std);
    // untouched channels keep their defaults
    TEST_ASSERT_EQUAL_FLOAT(0.0f, v.airspeed);
    TEST_ASSERT_EQUAL_INT(1013, v.qnhMb);
}

static void test_unknown_keys_are_ignored(void)
{
    IsisDisplayValues v;
    IsisCfg            cfg;
    ProtoContext       ctx;

    const ProtoResult r = protoParseFrame("SEQ5|FOO:1;BAR:baz;DSP1:200;DSP9:77;", v, cfg, ctx);

    TEST_ASSERT_EQUAL_INT((int)ProtoStatus::OK, (int)r.status);
    TEST_ASSERT_EQUAL_UINT16((uint16_t)(1u << ISIS_DSP_AIRSPEED), r.dspMask);
    TEST_ASSERT_EQUAL_FLOAT(200.0f, v.airspeed);
}

static void test_value_without_terminator_is_discarded(void)
{
    IsisDisplayValues v;
    IsisCfg            cfg;
    ProtoContext       ctx;

    const ProtoResult r = protoParseFrame("SEQ9|DSP1:100;DSP2:0.5", v, cfg, ctx);

    TEST_ASSERT_EQUAL_INT((int)ProtoStatus::OK, (int)r.status);
    TEST_ASSERT_EQUAL_UINT16((uint16_t)(1u << ISIS_DSP_AIRSPEED), r.dspMask);
    TEST_ASSERT_EQUAL_FLOAT(100.0f, v.airspeed);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, v.mach); // dropped - not terminated by ';'
}

static void test_empty_and_garbage_frames(void)
{
    IsisDisplayValues v;
    IsisCfg            cfg;
    ProtoContext       ctx;

    TEST_ASSERT_EQUAL_INT((int)ProtoStatus::EMPTY, (int)protoParseFrame("", v, cfg, ctx).status);
    TEST_ASSERT_EQUAL_INT((int)ProtoStatus::EMPTY, (int)protoParseFrame(nullptr, v, cfg, ctx).status);
    TEST_ASSERT_EQUAL_INT((int)ProtoStatus::EMPTY, (int)protoParseFrame(";;;", v, cfg, ctx).status);
    TEST_ASSERT_EQUAL_INT((int)ProtoStatus::EMPTY, (int)protoParseFrame("SEQ1|", v, cfg, ctx).status);
    // a key without ':' is not a key/value pair (new seq, otherwise it is
    // correctly dropped as a duplicate)
    TEST_ASSERT_EQUAL_INT((int)ProtoStatus::EMPTY, (int)protoParseFrame("SEQ2|JUSTTEXT;", v, cfg, ctx).status);
}

static void test_duplicate_sequence_is_dropped(void)
{
    IsisDisplayValues v;
    IsisCfg            cfg;
    ProtoContext       ctx;

    const ProtoResult a = protoParseFrame("SEQ7|DSP1:100;", v, cfg, ctx);
    TEST_ASSERT_EQUAL_INT((int)ProtoStatus::OK, (int)a.status);
    TEST_ASSERT_EQUAL_FLOAT(100.0f, v.airspeed);

    // identical sequence number -> UDP duplicate -> whole frame ignored
    const ProtoResult b = protoParseFrame("SEQ7|DSP1:250;", v, cfg, ctx);
    TEST_ASSERT_EQUAL_INT((int)ProtoStatus::DUPLICATE, (int)b.status);
    TEST_ASSERT_EQUAL_FLOAT(100.0f, v.airspeed);

    // a new sequence number is accepted again
    const ProtoResult c = protoParseFrame("SEQ8|DSP1:250;", v, cfg, ctx);
    TEST_ASSERT_EQUAL_INT((int)ProtoStatus::OK, (int)c.status);
    TEST_ASSERT_EQUAL_FLOAT(250.0f, v.airspeed);
}

static void test_frame_without_header_is_tolerated(void)
{
    IsisDisplayValues v;
    IsisCfg            cfg;
    ProtoContext       ctx;

    const ProtoResult r = protoParseFrame("DSP1:150;", v, cfg, ctx);
    TEST_ASSERT_EQUAL_INT((int)ProtoStatus::OK, (int)r.status);
    TEST_ASSERT_FALSE(r.hasSeq);
    TEST_ASSERT_EQUAL_FLOAT(150.0f, v.airspeed);
}

static void test_frame_cap_is_enforced(void)
{
    // Anything beyond ISIS_MAX_FRAME is cut off; the tail must not produce a
    // partial value (spec section 3, robustness rule).  The cap itself was
    // raised from 96 to 512 because the real FDC frames are 155 bytes long.
    char frame[1200];
    strcpy(frame, "SEQ1|DSP1:100;");
    // pad with an ignored key up to just below the cap
    while (strlen(frame) < ISIS_MAX_FRAME - 2) strcat(frame, "PAD:x;");
    // this channel starts past the cap -> must be dropped
    strcat(frame, "DSP2:0.9;");

    IsisDisplayValues v;
    IsisCfg            cfg;
    ProtoContext       ctx;
    const ProtoResult r = protoParseFrame(frame, v, cfg, ctx);

    TEST_ASSERT_TRUE(strlen(frame) > ISIS_MAX_FRAME);
    TEST_ASSERT_EQUAL_FLOAT(100.0f, v.airspeed);
    // the truncated tail must not have produced a partial value
    TEST_ASSERT_EQUAL_FLOAT(0.0f, v.mach);
}

// Regression guard for the real wire format: these are the exact bytes the FDC
// puts on the wire (captured with FDC_IP/tools/isis_fake.py on 13.10.2026).
// The frame is 155 bytes - with the old 96-byte window the cut landed inside
// "CFG", so scaling, offsets and CFG:ota=1 were silently dropped.
static void test_real_fdc_frame_from_capture(void)
{
    const char *frame =
        "SEQ1|DSP1:280;DSP2:0.78;DSP3:35000;DSP4:1013;DSP5:0;DSP6:-3.2;"
        "DSP7:12.5;DSP8:0.1;DISP_BL:180;"
        "CFG:scale_w=480;CFG:scale_h=480;CFG:offset_x=0;CFG:offset_y=0;";

    // guards the fixture itself against silent edits
    TEST_ASSERT_EQUAL_INT(155, (int)strlen(frame));

    IsisDisplayValues v;
    IsisCfg            cfg;
    ProtoContext       ctx;
    const ProtoResult  r = protoParseFrame(frame, v, cfg, ctx);

    TEST_ASSERT_EQUAL_INT((int)ProtoStatus::OK, (int)r.status);
    TEST_ASSERT_TRUE(r.hasSeq);
    TEST_ASSERT_EQUAL_UINT32(1, r.seq);
    TEST_ASSERT_EQUAL_UINT8(0xFF, (uint8_t)r.dspMask); // all 8 channels present
    TEST_ASSERT_TRUE(r.dispBl);
    TEST_ASSERT_TRUE(r.cfg);

    TEST_ASSERT_EQUAL_FLOAT(280.0f, v.airspeed);
    TEST_ASSERT_EQUAL_FLOAT(0.78f, v.mach);
    TEST_ASSERT_EQUAL_FLOAT(35000.0f, v.altitude);
    TEST_ASSERT_EQUAL_INT32(1013, v.qnhMb);
    TEST_ASSERT_EQUAL_INT32(0, v.std);
    TEST_ASSERT_EQUAL_FLOAT(-3.2f, v.pitch);
    TEST_ASSERT_EQUAL_FLOAT(12.5f, v.bank);
    TEST_ASSERT_EQUAL_FLOAT(0.1f, v.ball);
    TEST_ASSERT_EQUAL_INT32(180, v.dispBl);

    // the whole CFG block has to arrive - this is the actual bug guard
    TEST_ASSERT_EQUAL_INT32(480, cfg.scaleW);
    TEST_ASSERT_EQUAL_INT32(480, cfg.scaleH);
    TEST_ASSERT_EQUAL_INT32(0, cfg.offX);
    TEST_ASSERT_EQUAL_INT32(0, cfg.offY);
}

static void test_dsp_bl_alias_and_clamping(void)
{
    IsisDisplayValues v;
    IsisCfg            cfg;
    ProtoContext       ctx;

    protoParseFrame("SEQ1|DSP_BL:120;", v, cfg, ctx); // tolerated alias
    TEST_ASSERT_EQUAL_INT(120, v.dispBl);

    protoParseFrame("SEQ2|DISP_BL:999;", v, cfg, ctx);
    TEST_ASSERT_EQUAL_INT(255, v.dispBl);

    protoParseFrame("SEQ3|DISP_BL:-5;", v, cfg, ctx);
    TEST_ASSERT_EQUAL_INT(0, v.dispBl);
}

int main(int, char **)
{
    UNITY_BEGIN();
    RUN_TEST(test_dsp_index_mapping);
    RUN_TEST(test_dsp_message_ids_match_original);
    RUN_TEST(test_full_spec_frame);
    RUN_TEST(test_partial_frame_only_sets_present_channels);
    RUN_TEST(test_unknown_keys_are_ignored);
    RUN_TEST(test_value_without_terminator_is_discarded);
    RUN_TEST(test_empty_and_garbage_frames);
    RUN_TEST(test_duplicate_sequence_is_dropped);
    RUN_TEST(test_frame_without_header_is_tolerated);
    RUN_TEST(test_frame_cap_is_enforced);
    RUN_TEST(test_real_fdc_frame_from_capture);
    RUN_TEST(test_dsp_bl_alias_and_clamping);
    return UNITY_END();
}
