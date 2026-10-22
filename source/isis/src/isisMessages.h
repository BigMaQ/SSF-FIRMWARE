#pragma once
//
// isisMessages.h — device type / message-ID routing constants.
//
// Taken over unchanged from the MobiFlight adapter header of CC_ISIS (MIT
// License, Copyright (c) 2024 Chris Crawford / elral,
// https://github.com/ccrawford/CC_ISIS).  The MobiFlight code around it is
// gone; only the plain constants remain, which is explicitly allowed by the
// spec ("device-types header -> remove or take the constants over in blocks").
//
// CC_ISIS.cpp uses MSG_HSI_MIN to route message IDs: everything below it goes
// to CC_ISIS_Base::setCommon(), everything above to the ISIS-specific switch.
//
#pragma once
// Device type constant (ISIS is the only device type in this firmware)
#define CUSTOM_ISIS_DEVICE 2

// Message ID routing boundaries.
#define MSG_HSI_MIN  30
#define MSG_PFD_MIN  60
#define MSG_ISIS_MIN 99
