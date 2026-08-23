#include <cmath>
#include <cstdint>
#include <unistd.h>

#include "MLX90640_API.h"

namespace {
paramsMLX90640 parameters;
bool initialized = false;
}

// Defined by the existing Linux I2C driver source.
extern int i2c_fd;

extern "C" int InnoMlx90640_Init(int device_address, int refresh_rate_hz) {
  if (device_address < 0 || device_address > 0x7f) {
    return -100;
  }

  int rate_code = -1;
  switch (refresh_rate_hz) {
    case 1: rate_code = 1; break;
    case 2: rate_code = 2; break;
    case 4: rate_code = 3; break;
    case 8: rate_code = 4; break;
    case 16: rate_code = 5; break;
    case 32: rate_code = 6; break;
    case 64: rate_code = 7; break;
    default: return -101;
  }

  const auto address = static_cast<uint8_t>(device_address);
  int status = MLX90640_SetRefreshRate(address, static_cast<uint8_t>(rate_code));
  if (status != 0) return status;
  status = MLX90640_GetRefreshRate(address);
  if (status != rate_code) return status < 0 ? status : -102;
  status = MLX90640_SetChessMode(address);
  if (status != 0) return status;

  uint16_t eeprom[832];
  status = MLX90640_DumpEE(address, eeprom);
  if (status != 0) return status;
  status = MLX90640_ExtractParameters(eeprom, &parameters);
  if (status != 0) return status;
  initialized = true;
  return 0;
}

extern "C" int InnoMlx90640_ReadFrame(int device_address, float *temperatures) {
  if (!initialized || temperatures == nullptr) return -103;
  const auto address = static_cast<uint8_t>(device_address);
  uint16_t frame[834];
  int status = MLX90640_GetFrameData(address, frame);
  if (status < 0) return status;

  const float ambient = MLX90640_GetTa(frame, &parameters);
  if (!std::isfinite(ambient)) return -104;
  MLX90640_CalculateTo(frame, &parameters, 0.95f, ambient, temperatures);
  MLX90640_BadPixelsCorrection(parameters.brokenPixels, temperatures, 1, &parameters);
  MLX90640_BadPixelsCorrection(parameters.outlierPixels, temperatures, 1, &parameters);
  return 0;
}

extern "C" void InnoMlx90640_Close() {
  initialized = false;
  if (i2c_fd > 0) {
    close(i2c_fd);
    i2c_fd = 0;
  }
}
