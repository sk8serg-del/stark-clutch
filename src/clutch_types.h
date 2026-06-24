#pragma once

struct FlywheelParams {
  float riseLoaded, fallLoaded, riseUnloaded, fallUnloaded;
  float hangDuration, hangThreshold;
  float snapBoost, snapDuration;
  float snapThreshold;
};

struct Preset {
  FlywheelParams fw;
  double coef1[11];
  double coef2[11];
};
