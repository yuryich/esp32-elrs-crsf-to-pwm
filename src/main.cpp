#include <Arduino.h>
#include <map>
#include "crsf.h"
#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"

#define RXD2 16
#define TXD2 17
#define VTX_RX_PIN 18
#define VTX_TX_PIN 23

#define SBUS_BUFFER_SIZE 25

uint8_t _rcs_buf[SBUS_BUFFER_SIZE] {};
uint16_t _raw_rc_values[RC_INPUT_MAX_CHANNELS] {};
uint16_t _raw_rc_count{};

HardwareSerial trampSerial(1);

uint32_t lastVTXUpdate = 0;
const uint32_t VTX_UPDATE_INTERVAL = 500;  // Обновлять каждые 500 мс

// ---------------- Таблица "частота -> код" ----------------
struct FreqCode {
  uint16_t freq;
  uint8_t code;
};

// Пример: можно расширить под весь freq_table
std::map<std::uint16_t, std::uint8_t> freqTable = {
    // A
    {5865, 0},
    {5845, 1},
    {5825, 2},
    {5805, 3},
    {5785, 4},
    {5765, 5},
    {5745, 6},
    {5725, 7},

    // B
    {5733, 8},
    {5752, 9},
    {5771, 10},
    {5790, 11},
    {5809, 12},
    {5828, 13},
    {5847, 14},
    {5866, 15},

    // E
    {5705, 16},
    {5685, 17},
    {5665, 18},
    {5645, 19},
    {5885, 20},
    {5905, 21},
    {5925, 22},
    {5945, 23},

    // F
    {5740, 24},
    {5760, 25},
    {5780, 26},
    {5800, 27},
    {5820, 28},
    {5840, 29},
    {5860, 30},
    {5880, 31},

    // R
    {5658, 32},
    {5695, 33},
    {5732, 34},
    {5769, 35},
    {5806, 36},
    {5843, 37},
    // {5880, 0x38},
    {5917, 39},

    // L
    {5362, 40},
    {5399, 41},
    {5436, 42},
    {5473, 43},
    {5510, 44},
    {5547, 45},
    {5584, 46},
    {5621, 47},

    // X
    {4990, 48},
    {5020, 49},
    {5050, 50},
    {5080, 51},
    {5110, 52},
    {5140, 53},
    {5170, 54},
    {5200, 55}
};

const uint8_t FREQ_TABLE_SIZE = sizeof(freqTable) / sizeof(freqTable[0]);


uint16_t findFreqCode(uint16_t freq) {
  uint16_t num, currentDiff;
  uint16_t closest = freqTable.begin()->first;
  uint16_t minDiff = abs(closest - freq);

  for (auto it = freqTable.begin(); it != freqTable.end(); ++it)
  {
      num = it->first;
      currentDiff = abs(num - freq);
      if (currentDiff < minDiff) {
          minDiff = currentDiff;
          closest = num;
      }
  }

  return closest;
}

// ---------------- Отправка команды Steadyview X ----------------

void setModuleFrequencyX(uint16_t frequency) {
  uint16_t closest = findFreqCode(frequency);
  uint8_t freq_value = freqTable[closest];
  uint8_t data[] = {0x02, 0x06, 0x31, freq_value, 0x01, 0x03};
  
  uint8_t crc = 0;
  for (int i = 1; i < 4; i++) {
      crc ^= data[i];
  }
  data[4] = crc;
    
  for (int i = 0; i < 5; i++) {
      trampSerial.write(data, sizeof(data));
      delay(1);
  }
  
  

  Serial.print("Set freq: ");
  Serial.print(frequency);
  Serial.print(" MHz, code: ");
  Serial.println(freq_value);
}



uint16_t currentFreq = 0;

void updateVTXFromAux8(int aux8Value) {
  uint16_t freq;

  if (aux8Value < 1000) {
    freq = 5865;   
  } else if (aux8Value < 1100) {
    freq = 5845;
  } else if (aux8Value < 1210) {
    freq = 5825;
  } else if (aux8Value < 1350) {
    freq = 5805;
  } else if (aux8Value < 1450) {
    freq = 5785;
  } else if (aux8Value < 1560) {
    freq = 5765;
  } else if (aux8Value < 1875) {
    freq = 5745;
  } else {
    freq = 5725;   
  }

  if (freq != currentFreq) {
    currentFreq = freq;
    setModuleFrequencyX(freq);
  }
}


void setup() {
  Serial.begin(115200);

  Serial2.begin(420000, SERIAL_8N1, RXD2, TXD2);

  trampSerial.begin(115200, SERIAL_8N1, VTX_RX_PIN, VTX_TX_PIN);
}

void loop() {
  while (Serial2.available()) {
    size_t numBytesRead = Serial2.readBytes(_rcs_buf, SBUS_BUFFER_SIZE);
    if (numBytesRead > 0) {
      crsf_parse(&_rcs_buf[0], SBUS_BUFFER_SIZE,
                 &_raw_rc_values[0], &_raw_rc_count, RC_INPUT_MAX_CHANNELS);
      int AUX8 = _raw_rc_values[11];  // 12‑й канал (AUX8)
      Serial.print("Channel 12 (AUX8): ");
      Serial.println(AUX8);
      if (millis() - lastVTXUpdate > VTX_UPDATE_INTERVAL) {
        updateVTXFromAux8(AUX8);
        lastVTXUpdate = millis();
      }
    }
  }
}
