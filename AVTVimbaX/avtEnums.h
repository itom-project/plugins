#ifndef AVTENUMS_H
#define AVTENUMS_H

#include <VimbaCPP/Include/VimbaCPP.h>


struct BppEnum
{
    BppEnum() : bppMono8(-1), bppMono10(-1), bppMono12(-1), bppMono14(-1) {}
    VmbInt64_t bppMono8;
    VmbInt64_t bppMono10;
    VmbInt64_t bppMono12;
    VmbInt64_t bppMono14;
};

struct TriggerSourceEnum
{
    TriggerSourceEnum() : triggerFreerun(-1), triggerLine1(-1), triggerLine2(-1), triggerLine3(-1), triggerLine4(-1), triggerFixedRate(-1), triggerSoftware(-1), triggerInputLines(-1) {}
    VmbInt64_t triggerFreerun;
    VmbInt64_t triggerLine1;
    VmbInt64_t triggerLine2;
    VmbInt64_t triggerLine3;
    VmbInt64_t triggerLine4;
    VmbInt64_t triggerFixedRate;
    VmbInt64_t triggerSoftware;
    VmbInt64_t triggerInputLines;
};

struct TriggerActivationEnum
{
    TriggerActivationEnum() : taRisingEdge(-1), taFallingEdge(-1), taAnyEdge(-1), taLevelHigh(-1), taLevelLow(-1) {}
    VmbInt64_t taRisingEdge;
    VmbInt64_t taFallingEdge;
    VmbInt64_t taAnyEdge;
    VmbInt64_t taLevelHigh;
    VmbInt64_t taLevelLow;
};

#endif
