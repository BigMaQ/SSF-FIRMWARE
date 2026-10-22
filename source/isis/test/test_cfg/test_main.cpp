// Host unit tests: CFG handling, scaling ranges and the
// FDC > NVS > default priority (ISIS_FIRMWARE_SPEC.md sections 4.1 and 7).
#include <unity.h>
#include <cstring>

#include "displaycfg.h"
#include "proto.h"

void setUp(void) {}
void tearDown(void) {}

// ---------------------------------------------------------------------------
static void test_defaults_are_identity(void)
{
    IsisCfg cfg;
    TEST_ASSERT_EQUAL_INT(480, cfg.scaleW);
    TEST_ASSERT_EQUAL_INT(480, cfg.scaleH);
    TEST_ASSERT_EQUAL_INT(0, cfg.offX);
    TEST_ASSERT_EQUAL_INT(0, cfg.offY);
    TEST_ASSERT_TRUE(displaycfgIdentity(cfg));
    TEST_ASSERT_EQUAL_FLOAT(1.0f, displaycfgZoomX(cfg));
    TEST_ASSERT_EQUAL_FLOAT(1.0f, displaycfgZoomY(cfg));
}

static void test_apply_key_short_and_long_spelling(void)
{
    IsisCfg cfg;

    TEST_ASSERT_EQUAL_UINT32(CFG_SCALE_W, displaycfgApplyKey(cfg, "scale_w", "400"));
    TEST_ASSERT_EQUAL_INT(400, cfg.scaleW);

    TEST_ASSERT_EQUAL_UINT32(CFG_OFF_X, displaycfgApplyKey(cfg, "off_x", "40"));
    TEST_ASSERT_EQUAL_INT(40, cfg.offX);

    TEST_ASSERT_EQUAL_UINT32(CFG_OFF_Y, displaycfgApplyKey(cfg, "offset_y", "30"));
    TEST_ASSERT_EQUAL_INT(30, cfg.offY);

    TEST_ASSERT_EQUAL_UINT32(CFG_OFF_Y, displaycfgApplyKey(cfg, "off_y", "31"));
    TEST_ASSERT_EQUAL_INT(31, cfg.offY);

    TEST_ASSERT_FALSE(displaycfgIdentity(cfg));
}

static void test_unknown_keys_are_ignored(void)
{
    IsisCfg cfg;
    const IsisCfg before = cfg;

    TEST_ASSERT_EQUAL_UINT32(0, displaycfgApplyKey(cfg, "nonsense", "5"));
    TEST_ASSERT_EQUAL_UINT32(0, displaycfgApplyKey(cfg, "", "5"));
    TEST_ASSERT_EQUAL_UINT32(0, displaycfgApplyKey(cfg, "net", "carrier_pigeon"));
    TEST_ASSERT_EQUAL_UINT32(0, displaycfgApplyKv(cfg, "noseparator"));

    TEST_ASSERT_EQUAL_INT(before.scaleW, cfg.scaleW);
    TEST_ASSERT_EQUAL_INT(before.offX, cfg.offX);
    TEST_ASSERT_EQUAL_INT((int)before.netMode, (int)cfg.netMode);
}

static void test_scale_is_clamped_to_240_480(void)
{
    IsisCfg cfg;
    displaycfgApplyKey(cfg, "scale_w", "100");
    TEST_ASSERT_EQUAL_INT(ISIS_SCALE_MIN, cfg.scaleW);

    displaycfgApplyKey(cfg, "scale_h", "9999");
    TEST_ASSERT_EQUAL_INT(ISIS_SCALE_MAX, cfg.scaleH);
}

static void test_backlight_range(void)
{
    IsisCfg cfg;
    displaycfgApplyKey(cfg, "disp_bl", "255");
    TEST_ASSERT_EQUAL_INT(255, cfg.blBase);
    displaycfgApplyKey(cfg, "disp_bl", "300");
    TEST_ASSERT_EQUAL_INT(255, cfg.blBase);

    // local BRT +/- steps (5 % of the range) and stays inside 0..255
    const int32_t up = displaycfgBlStep(200, +1);
    TEST_ASSERT_EQUAL_INT(200 + ISIS_BL_LOCAL_STEP, up);
    TEST_ASSERT_EQUAL_INT(255, displaycfgBlStep(250, +1));
    TEST_ASSERT_EQUAL_INT(0, displaycfgBlStep(5, -1));
    TEST_ASSERT_EQUAL_INT(200, displaycfgBlStep(200, 0));
}

static void test_net_mode_key(void)
{
    IsisCfg cfg;
    TEST_ASSERT_EQUAL_UINT32(CFG_NET, displaycfgApplyKey(cfg, "net", "static_nogw"));
    TEST_ASSERT_EQUAL_INT((int)ISIS_NET_STATIC_NOGW, (int)cfg.netMode);

    displaycfgApplyKey(cfg, "net", "dhcp");
    TEST_ASSERT_EQUAL_INT((int)ISIS_NET_DHCP, (int)cfg.netMode);

    displaycfgApplyKey(cfg, "net", "static");
    TEST_ASSERT_EQUAL_INT((int)ISIS_NET_STATIC, (int)cfg.netMode);
}

// ---------------------------------------------------------------------------
static void test_priority_fdc_beats_nvs_beats_default(void)
{
    IsisCfgLayers layers;
    layers.defaults = IsisCfg();          // 480/480/0/0
    layers.nvs      = IsisCfg();
    layers.fdc      = IsisCfg();

    layers.nvs.scaleW = 400;
    layers.nvsMask    = CFG_SCALE_W;

    layers.fdc.scaleW = 360;
    layers.fdc.offX   = 20;
    layers.fdcMask    = CFG_SCALE_W | CFG_OFF_X;

    IsisCfg out;
    const uint32_t used = displaycfgResolve(out, layers);

    // FDC wins for scale_w, NVS value is overruled
    TEST_ASSERT_EQUAL_INT(360, out.scaleW);
    // FDC offset applies
    TEST_ASSERT_EQUAL_INT(20, out.offX);
    // nothing provided scale_h -> firmware default stays
    TEST_ASSERT_EQUAL_INT(480, out.scaleH);
    TEST_ASSERT_EQUAL_UINT32(CFG_SCALE_W | CFG_OFF_X, used);
}

static void test_nvs_applies_when_fdc_sends_nothing(void)
{
    IsisCfgLayers layers;
    layers.nvs.scaleW  = 300;
    layers.nvs.scaleH  = 300;
    layers.nvs.blBase  = 90;
    layers.nvsMask     = CFG_SCALE_W | CFG_SCALE_H | CFG_BL_BASE;

    IsisCfg out;
    displaycfgResolve(out, layers);

    TEST_ASSERT_EQUAL_INT(300, out.scaleW);
    TEST_ASSERT_EQUAL_INT(300, out.scaleH);
    TEST_ASSERT_EQUAL_INT(90, out.blBase);
    TEST_ASSERT_EQUAL_INT(0, out.offX);
    TEST_ASSERT_FALSE(displaycfgIdentity(out));
}

// ---------------------------------------------------------------------------
static void test_cfg_token_is_parsed_inside_a_frame(void)
{
    IsisDisplayValues v;
    IsisCfg            cfg;
    ProtoContext       ctx;

    const ProtoResult r = protoParseFrame(
        "SEQ3|DSP1:250;CFG:scale_w=400;CFG:scale_h=400;CFG:off_x=40;CFG:off_y=30;",
        v, cfg, ctx);

    TEST_ASSERT_EQUAL_INT((int)ProtoStatus::OK, (int)r.status);
    TEST_ASSERT_TRUE(r.cfg);
    TEST_ASSERT_TRUE(r.cfgChanged);
    TEST_ASSERT_EQUAL_UINT32(CFG_SCALE_W | CFG_SCALE_H | CFG_OFF_X | CFG_OFF_Y, r.cfgMask);

    TEST_ASSERT_EQUAL_INT(400, cfg.scaleW);
    TEST_ASSERT_EQUAL_INT(400, cfg.scaleH);
    TEST_ASSERT_EQUAL_INT(40, cfg.offX);
    TEST_ASSERT_EQUAL_INT(30, cfg.offY);
    TEST_ASSERT_EQUAL_FLOAT(250.0f, v.airspeed);
}

static void test_cfg_ota_request_is_flagged(void)
{
    IsisDisplayValues v;
    IsisCfg            cfg;
    ProtoContext       ctx;

    const ProtoResult r = protoParseFrame("SEQ4|CFG:ota=1;", v, cfg, ctx);
    TEST_ASSERT_TRUE(r.cfg);
    TEST_ASSERT_TRUE(r.otaRequest);
    TEST_ASSERT_TRUE(cfg.ota);
    TEST_ASSERT_EQUAL_UINT32(CFG_OTA, r.cfgMask);

    // CFG:ota=0 must not request anything
    IsisDisplayValues v2;
    IsisCfg            cfg2;
    ProtoContext       ctx2;
    const ProtoResult r2 = protoParseFrame("SEQ5|CFG:ota=0;", v2, cfg2, ctx2);
    TEST_ASSERT_FALSE(r2.otaRequest);
    TEST_ASSERT_FALSE(cfg2.ota);
}

static void test_bad_cfg_value_does_not_break_the_frame(void)
{
    IsisDisplayValues v;
    IsisCfg            cfg;
    ProtoContext       ctx;

    const ProtoResult r = protoParseFrame("SEQ6|CFG:scale_w;CFG:;;DSP1:180;", v, cfg, ctx);
    TEST_ASSERT_EQUAL_INT((int)ProtoStatus::OK, (int)r.status);
    TEST_ASSERT_FALSE(r.cfg);
    TEST_ASSERT_EQUAL_FLOAT(180.0f, v.airspeed);
    TEST_ASSERT_EQUAL_INT(480, cfg.scaleW);
}

// ---------------------------------------------------------------------------
// Zuordnungs-Modus (CFG:assign=<zone>) — Wunsch aus dem Cockpit: beim Zuordnen
// blinkt der betroffene Wert, und im Display steht "Assignment in progress".
// Bewusst KEIN neuer Textbefehl: der Modus reist als Flag in der CFG-Schicht mit.
// ---------------------------------------------------------------------------
static void test_assign_zone_is_parsed_from_a_frame(void)
{
    IsisDisplayValues v;
    IsisCfg           cfg;
    ProtoContext      ctx;

    const ProtoResult r = protoParseFrame("SEQ7|DSP4:1013;CFG:assign=baro;", v, cfg, ctx);
    TEST_ASSERT_TRUE(r.cfg);
    TEST_ASSERT_EQUAL_UINT32(CFG_ASSIGN, r.cfgMask);
    TEST_ASSERT_EQUAL_INT((int)ISIS_ASSIGN_BARO, (int)cfg.assignZone);
    TEST_ASSERT_EQUAL_INT(1013, v.qnhMb);          // der Wert kommt weiterhin mit
}

static void test_assign_off_ends_the_mode(void)
{
    IsisCfg cfg;
    cfg.assignZone = ISIS_ASSIGN_ALT;

    TEST_ASSERT_EQUAL_UINT32(CFG_ASSIGN, displaycfgApplyKey(cfg, "assign", "off"));
    TEST_ASSERT_EQUAL_INT((int)ISIS_ASSIGN_NONE, (int)cfg.assignZone);

    cfg.assignZone = ISIS_ASSIGN_ALT;
    TEST_ASSERT_EQUAL_UINT32(CFG_ASSIGN, displaycfgApplyKey(cfg, "assign", "none"));
    TEST_ASSERT_EQUAL_INT((int)ISIS_ASSIGN_NONE, (int)cfg.assignZone);
}

static void test_unknown_assign_zone_is_ignored(void)
{
    IsisCfg cfg;
    cfg.assignZone = ISIS_ASSIGN_SPD;

    // Tippfehler darf den laufenden Modus nicht abschalten
    TEST_ASSERT_EQUAL_UINT32(0, displaycfgApplyKey(cfg, "assign", "barro"));
    TEST_ASSERT_EQUAL_UINT32(0, displaycfgApplyKey(cfg, "assign", ""));
    TEST_ASSERT_EQUAL_INT((int)ISIS_ASSIGN_SPD, (int)cfg.assignZone);

    // ... und "off"/"0"/"none" schalten ihn wirklich ab
    TEST_ASSERT_EQUAL_UINT32(CFG_ASSIGN, displaycfgApplyKey(cfg, "assign", "0"));
    TEST_ASSERT_EQUAL_INT((int)ISIS_ASSIGN_NONE, (int)cfg.assignZone);
}

static void test_assign_names_round_trip(void)
{
    const IsisAssignZone zonen[] = {
        ISIS_ASSIGN_NONE, ISIS_ASSIGN_BARO, ISIS_ASSIGN_ALT, ISIS_ASSIGN_SPD,
        ISIS_ASSIGN_MACH, ISIS_ASSIGN_ATT,  ISIS_ASSIGN_BANK, ISIS_ASSIGN_SLIP,
        ISIS_ASSIGN_ALL,
    };
    for (IsisAssignZone z : zonen) {
        const char *name = isisAssignZoneName(z);
        TEST_ASSERT_NOT_NULL(name);
        IsisCfg cfg;
        TEST_ASSERT_EQUAL_UINT32(CFG_ASSIGN, displaycfgApplyKey(cfg, "assign", name));
        TEST_ASSERT_EQUAL_INT((int)z, (int)cfg.assignZone);
    }
    TEST_ASSERT_EQUAL_STRING("none", isisAssignZoneName((IsisAssignZone)99));
}

static void test_assign_mode_wins_over_the_nvs_layer(void)
{
    IsisCfgLayers layers;
    layers.nvs.assignZone = ISIS_ASSIGN_SPD;
    layers.nvsMask        = CFG_ASSIGN;
    layers.fdc.assignZone = ISIS_ASSIGN_BARO;
    layers.fdcMask        = CFG_ASSIGN;

    IsisCfg out;
    const uint32_t mask = displaycfgResolve(out, layers);
    TEST_ASSERT_EQUAL_INT((int)ISIS_ASSIGN_BARO, (int)out.assignZone);
    TEST_ASSERT_EQUAL_UINT32(CFG_ASSIGN, mask & CFG_ASSIGN);
}

static void test_assign_mode_is_never_persisted(void)
{
    // Assign ist ein Modus, kein Zustand: sonst startet das Panel nach dem Neustart
    // im Blink-Zustand. Alles andere darf weiter in den NVS.
    const uint32_t alles = CFG_SCALE_W | CFG_SCALE_H | CFG_OFF_X | CFG_OFF_Y
                         | CFG_BL_BASE | CFG_OTA | CFG_NET | CFG_ASSIGN;
    const uint32_t persist = displaycfgPersistMask(alles);

    TEST_ASSERT_EQUAL_UINT32(0, persist & CFG_ASSIGN);
    TEST_ASSERT_EQUAL_UINT32(0, persist & CFG_OTA);
    TEST_ASSERT_TRUE(persist & CFG_SCALE_W);
    TEST_ASSERT_TRUE(persist & CFG_NET);
    TEST_ASSERT_TRUE(persist & CFG_BL_BASE);
}

static void test_assign_label_is_sanitized_and_cut(void)
{
    IsisCfg cfg;
    TEST_ASSERT_EQUAL_UINT32(CFG_ASSIGN_LABEL,
                             displaycfgApplyKey(cfg, "assign_label", "baro-dec"));
    TEST_ASSERT_EQUAL_STRING("BARO-DEC", cfg.assignLabel);

    // unlesbare Zeichen fliegen raus, Leerzeichen und +/-/_/. bleiben
    TEST_ASSERT_EQUAL_UINT32(CFG_ASSIGN_LABEL,
                             displaycfgApplyKey(cfg, "assign_label", "IN1-B5 \u00e4\u00f6#"));
    TEST_ASSERT_EQUAL_STRING("IN1-B5 ", cfg.assignLabel);

    // zu lang: abschneiden statt ueberlaufen (15 sichtbare Zeichen)
    TEST_ASSERT_EQUAL_UINT32(CFG_ASSIGN_LABEL,
                             displaycfgApplyKey(cfg, "assign_label", "ABCDEFGHIJKLMNOPQRSTUVW"));
    TEST_ASSERT_EQUAL_INT(15, (int)strlen(cfg.assignLabel));
    TEST_ASSERT_EQUAL_STRING("ABCDEFGHIJKLMNO", cfg.assignLabel);
}

static void test_assign_label_is_never_persisted(void)
{
    const uint32_t alles = CFG_ASSIGN | CFG_ASSIGN_LABEL | CFG_SCALE_W | CFG_NET;
    const uint32_t persist = displaycfgPersistMask(alles);
    TEST_ASSERT_EQUAL_UINT32(0, persist & CFG_ASSIGN);
    TEST_ASSERT_EQUAL_UINT32(0, persist & CFG_ASSIGN_LABEL);
    TEST_ASSERT_TRUE(persist & CFG_SCALE_W);
}

static void test_assign_label_rides_in_the_same_frame_as_the_zone(void)
{
    IsisDisplayValues v;
    IsisCfg           cfg;
    ProtoContext      ctx;
    const ProtoResult r = protoParseFrame("SEQ9|DSP1:280;CFG:assign=spd;CFG:assign_label=SPD-TAPE;",
                                          v, cfg, ctx);
    TEST_ASSERT_TRUE(r.cfg);
    TEST_ASSERT_EQUAL_INT((int)ISIS_ASSIGN_SPD, (int)cfg.assignZone);
    TEST_ASSERT_EQUAL_STRING("SPD-TAPE", cfg.assignLabel);
    TEST_ASSERT_EQUAL_UINT32(CFG_ASSIGN | CFG_ASSIGN_LABEL, r.cfgMask);
}

int main(int, char **)
{
    UNITY_BEGIN();
    RUN_TEST(test_defaults_are_identity);
    RUN_TEST(test_apply_key_short_and_long_spelling);
    RUN_TEST(test_unknown_keys_are_ignored);
    RUN_TEST(test_scale_is_clamped_to_240_480);
    RUN_TEST(test_backlight_range);
    RUN_TEST(test_net_mode_key);
    RUN_TEST(test_priority_fdc_beats_nvs_beats_default);
    RUN_TEST(test_nvs_applies_when_fdc_sends_nothing);
    RUN_TEST(test_cfg_token_is_parsed_inside_a_frame);
    RUN_TEST(test_cfg_ota_request_is_flagged);
    RUN_TEST(test_bad_cfg_value_does_not_break_the_frame);
    RUN_TEST(test_assign_zone_is_parsed_from_a_frame);
    RUN_TEST(test_assign_off_ends_the_mode);
    RUN_TEST(test_unknown_assign_zone_is_ignored);
    RUN_TEST(test_assign_names_round_trip);
    RUN_TEST(test_assign_mode_wins_over_the_nvs_layer);
    RUN_TEST(test_assign_mode_is_never_persisted);
    RUN_TEST(test_assign_label_is_sanitized_and_cut);
    RUN_TEST(test_assign_label_is_never_persisted);
    RUN_TEST(test_assign_label_rides_in_the_same_frame_as_the_zone);
    return UNITY_END();
}
