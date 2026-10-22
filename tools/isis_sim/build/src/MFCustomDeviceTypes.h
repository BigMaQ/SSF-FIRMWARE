#pragma once

// Device type constant (ISIS is the only device type in this firmware)
#define CUSTOM_ISIS_DEVICE 2

// Message ID routing boundaries.
// IDs below MSG_HSI_MIN are common (routed to CC_ISIS::setCommon).
// IDs >= MSG_ISIS_MIN   are ISIS-specific (routed to CC_ISIS::set).
#define MSG_HSI_MIN  30
#define MSG_PFD_MIN  60
#define MSG_ISIS_MIN 99