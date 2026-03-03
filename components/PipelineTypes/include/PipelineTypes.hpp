#ifndef PIPELINES_TYPES_HPP
#define PIPELINES_TYPES_HPP

#pragma once
#include <stdio.h>
#include <bits/stdc++.h>

enum WifiManagerStatus {
    INIT,
    READY,
    DISCONNECTED,
    CONNECTING,
    CONNECTED,
    SCANNING,
    CONNECTION_FAILED
};


struct RollPitch
{
    float roll = 0.0f;
    float pitch = 0.0f;
    float temperature = 0.0f;
};

struct WifiManagerPipeline
{
    char AvailableNetworks[200];
    WifiManagerStatus WifiStatus;
};


#endif //PIPELINES_TYPES_HPP