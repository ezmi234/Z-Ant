/************ Nicla Vision QSPI XIP (Memory-Mapped) + Serial RPC ************
 * Core: arduino:mbed_nicla (4.4.1)
 * Purpose: Serve simple line-based commands over USB Serial to:
 *   - run inference on demand (INFER or INFER+IMAGE)
 *   - stream a camera frame as PPM (IMAGE)
 *   - basic handshake and info (HELLO, PING, INFO)
 *
 * Protocol (newline-terminated ASCII):
 *   -> HELLO         : Nicla replies with a JSON hello line
 *   -> PING          : Nicla replies {"type":"pong","t_ms":...}
 *   -> INFO          : Nicla replies with JSON of in/out shapes
 *   -> IMAGE         : Nicla replies BEGIN_IMAGE + PPM + END_IMAGE
 *   -> INFER         : Nicla replies {"type":"result", "class":i, "score":f, "latency_us":u}
 *   -> INFER+IMAGE   : Nicla replies result JSON, then BEGIN_IMAGE...END_IMAGE
 *****************************************************************************/

extern "C" {
#ifndef STM32H747xx
#define STM32H747xx
#endif
#ifndef HAL_QSPI_MODULE_ENABLED
#define HAL_QSPI_MODULE_ENABLED
#endif
#include "stm32h7xx_hal.h"
#include "stm32h7xx_hal_qspi.h"
}

// Required by the Zig library:
extern "C" __attribute__((used))
const uint8_t *flash_weights_base = (const uint8_t *)0x90000000u;

#include <Arduino.h>
#include <lib_zant.h> // int predict(float*, uint32_t*, uint32_t, float**)
#include "RPC.h"
#include "camera.h"
#include "gc2145.h"

// ===== QSPI low-level =====
static QSPI_HandleTypeDef hqspi;
static const uint8_t CMD_RDID = 0x9F, CMD_WREN = 0x06;
static const uint8_t CMD_RDSR1 = 0x05, CMD_RDSR2 = 0x35, CMD_WRSR = 0x01;
static const uint8_t CMD_READ_QO = 0x6B;
static volatile bool g_busy = false;
static constexpr float R_MEAN = 123.675f;
static constexpr float G_MEAN = 116.28f;
static constexpr float B_MEAN = 103.53f;
static constexpr float R_STD  = 58.395f;
static constexpr float G_STD  = 57.12f;
static constexpr float B_STD  = 57.375f;

extern "C" void HAL_QSPI_MspInit(QSPI_HandleTypeDef *h) {
  if (h->Instance != QUADSPI) return;
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();
  __HAL_RCC_QSPI_CLK_ENABLE();

  GPIO_InitTypeDef GPIO = {0};
  // CLK PB2 (AF9)
  GPIO.Pin = GPIO_PIN_2; GPIO.Mode = GPIO_MODE_AF_PP; GPIO.Pull = GPIO_NOPULL;
  GPIO.Speed = GPIO_SPEED_FREQ_VERY_HIGH; GPIO.Alternate = GPIO_AF9_QUADSPI;
  HAL_GPIO_Init(GPIOB, &GPIO);
  // CS PG6 (AF10)
  GPIO.Pin = GPIO_PIN_6; GPIO.Alternate = GPIO_AF10_QUADSPI;
  HAL_GPIO_Init(GPIOG, &GPIO);
  // IO0..IO3 PD11..PD14 (AF9)
  GPIO.Pin = GPIO_PIN_11 | GPIO_PIN_12 | GPIO_PIN_13 | GPIO_PIN_14;
  GPIO.Alternate = GPIO_AF9_QUADSPI;
  HAL_GPIO_Init(GPIOD, &GPIO);
}

static HAL_StatusTypeDef qspi_init_16mb(QSPI_HandleTypeDef *h) {
  h->Instance = QUADSPI;
  h->Init.ClockPrescaler = 7;
  h->Init.FifoThreshold = 4;
  h->Init.SampleShifting = QSPI_SAMPLE_SHIFTING_NONE;
  h->Init.FlashSize = 23; // 2^24 = 16MB -> set 23
  h->Init.ChipSelectHighTime = QSPI_CS_HIGH_TIME_2_CYCLE;
  h->Init.ClockMode = QSPI_CLOCK_MODE_0;
  h->Init.FlashID = QSPI_FLASH_ID_1;
  h->Init.DualFlash = QSPI_DUALFLASH_DISABLE;
  return HAL_QSPI_Init(h);
}

static HAL_StatusTypeDef qspi_cmd(QSPI_HandleTypeDef *h, uint8_t inst,
                                  uint32_t addrMode, uint32_t dataMode,
                                  uint32_t addr, uint32_t dummy,
                                  uint8_t *data, size_t len, bool rx) {
  QSPI_CommandTypeDef c = {0};
  c.InstructionMode = QSPI_INSTRUCTION_1_LINE;
  c.Instruction = inst;
  c.AddressMode = addrMode;
  c.Address = addr;
  c.AddressSize = QSPI_ADDRESS_24_BITS;
  c.DataMode = dataMode;
  c.NbData = len;
  c.DummyCycles = dummy;
  if (HAL_QSPI_Command(h, &c, HAL_MAX_DELAY) != HAL_OK) return HAL_ERROR;
  if (len == 0) return HAL_OK;
  return rx ? HAL_QSPI_Receive(h, data, HAL_MAX_DELAY)
            : HAL_QSPI_Transmit(h, data, HAL_MAX_DELAY);
}

static HAL_StatusTypeDef rd_sr(QSPI_HandleTypeDef *h, uint8_t cmd, uint8_t *val) {
  return qspi_cmd(h, cmd, QSPI_ADDRESS_NONE, QSPI_DATA_1_LINE, 0, 0, val, 1, true);
}
static HAL_StatusTypeDef wren(QSPI_HandleTypeDef *h) {
  return qspi_cmd(h, CMD_WREN, QSPI_ADDRESS_NONE, QSPI_DATA_NONE, 0, 0, nullptr, 0, true);
}
static HAL_StatusTypeDef wr_sr12(QSPI_HandleTypeDef *h, uint8_t sr1, uint8_t sr2) {
  uint8_t buf[2] = {sr1, sr2};
  return qspi_cmd(h, CMD_WRSR, QSPI_ADDRESS_NONE, QSPI_DATA_1_LINE, 0, 0, buf, 2, false);
}

static HAL_StatusTypeDef wait_wip_clear(QSPI_HandleTypeDef *h, uint32_t timeout_ms) {
  uint32_t t0 = millis();
  for (;;) {
    uint8_t sr1 = 0;
    if (rd_sr(h, CMD_RDSR1, &sr1) != HAL_OK) return HAL_ERROR;
    if ((sr1 & 0x01) == 0) return HAL_OK;
    if ((millis() - t0) > timeout_ms) return HAL_TIMEOUT;
    delay(1);
  }
}
static HAL_StatusTypeDef enable_quad(QSPI_HandleTypeDef *h) {
  uint8_t sr1 = 0, sr2 = 0;
  if (rd_sr(h, CMD_RDSR1, &sr1) != HAL_OK) return HAL_ERROR;
  if (rd_sr(h, CMD_RDSR2, &sr2) != HAL_OK) return HAL_ERROR;
  if (sr2 & 0x02) return HAL_OK; // QE already 1
  if (wren(h) != HAL_OK) return HAL_ERROR;
  sr2 |= 0x02;
  if (wr_sr12(h, sr1, sr2) != HAL_OK) return HAL_ERROR;
  if (wait_wip_clear(h, 500) != HAL_OK) return HAL_ERROR;
  if (rd_sr(h, CMD_RDSR2, &sr2) != HAL_OK) return HAL_ERROR;
  return (sr2 & 0x02) ? HAL_OK : HAL_ERROR;
}

static HAL_StatusTypeDef qspi_enter_mmap(QSPI_HandleTypeDef *h) {
  QSPI_CommandTypeDef c = {0};
  c.InstructionMode = QSPI_INSTRUCTION_1_LINE;
  c.Instruction = CMD_READ_QO; // 0x6B
  c.AddressMode = QSPI_ADDRESS_1_LINE;
  c.AddressSize = QSPI_ADDRESS_24_BITS;
  c.Address = 0x000000;
  c.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
  c.DataMode = QSPI_DATA_4_LINES;
  c.DummyCycles = 8;
#ifdef QSPI_DDR_MODE_DISABLE
  c.DdrMode = QSPI_DDR_MODE_DISABLE;
  c.DdrHoldHalfCycle = QSPI_DDR_HHC_ANALOG_DELAY;
#endif
#ifdef QSPI_SIOO_INST_EVERY_CMD
  c.SIOOMode = QSPI_SIOO_INST_EVERY_CMD;
#endif
  QSPI_MemoryMappedTypeDef mm = {0};
  mm.TimeOutActivation = QSPI_TIMEOUT_COUNTER_DISABLE;
  mm.TimeOutPeriod = 0;
  return HAL_QSPI_MemoryMapped(h, &c, &mm);
}

// ===== Model I/O =====
#ifndef ZANT_OUTPUT_LEN
#define ZANT_OUTPUT_LEN 80
#endif
static const int OUT_LEN = ZANT_OUTPUT_LEN;
static const uint32_t IN_N = 1, IN_C = 3, IN_H = 96, IN_W = 96;
static const uint32_t IN_SIZE = IN_N * IN_C * IN_H * IN_W;
static float inputData[IN_SIZE];
static uint32_t inputShape[4] = {IN_N, IN_C, IN_H, IN_W};

// ===== Camera =====
#define USE_QQVGA
static const uint32_t CAM_W = 160;
static const uint32_t CAM_H = 120;
GC2145  sensor;
Camera  cam(sensor);
FrameBuffer frame;

// ===== Serial command parser =====
#define BAUDRATE 115200
static char cmdBuf[64];
static size_t cmdLen = 0;

static bool readLine() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\r') continue;
    if (c == '\n') { cmdBuf[cmdLen] = '\0'; cmdLen = 0; return true; }
    if (cmdLen < sizeof(cmdBuf) - 1) cmdBuf[cmdLen++] = c;
    else { cmdLen = 0; } // overflow -> reset
  }
  return false;
}

// ===== Utilities =====
static void captureToInputCenterCrop96() {
  cam.grabFrame(frame);
  const uint16_t *pix = (const uint16_t*)frame.getBuffer();

  // center crop 96x96 from 160x120
  const int offx = (int)(CAM_W - (int)IN_W) / 2;  // 32
  const int offy = (int)(CAM_H - (int)IN_H) / 2;  // 12

  const int planeSize = (int)(IN_H * IN_W);
  int di = 0;

  for (int y = 0; y < (int)IN_H; ++y) {
    const uint16_t* row = pix + (offy + y) * (int)CAM_W + offx;
    for (int x = 0; x < (int)IN_W; ++x, ++di) {
      uint16_t v = row[x];

      v = (uint16_t)((v << 8) | (v >> 8));

      uint8_t r = (uint8_t)(((v >> 11) & 0x1F) << 3);
      uint8_t g = (uint8_t)(((v >> 5)  & 0x3F) << 2);
      uint8_t b = (uint8_t)(( v        & 0x1F) << 3);

      inputData[0 * planeSize + di] = ((float)r - R_MEAN) / R_STD; 
      inputData[1 * planeSize + di] = ((float)g - G_MEAN) / G_STD;
      inputData[2 * planeSize + di] = ((float)b - B_MEAN) / B_STD;
    }
  }
}


static void sendImagePPM() {
  cam.grabFrame(frame);
  uint8_t *buf = frame.getBuffer();

  Serial.println("BEGIN_IMAGE");
  Serial.print("P6\n");
  Serial.print(CAM_W); Serial.print(" "); Serial.print(CAM_H);
  Serial.print("\n255\n");

  uint16_t *pixels = (uint16_t *)buf;
  const size_t N = (size_t)CAM_W * (size_t)CAM_H;
  for (size_t i = 0; i < N; ++i) {
    uint16_t pix = pixels[i];
    // Sensor buffer endianness on H7 can be swapped; ensure correct
    pix = (uint16_t)((pix << 8) | (pix >> 8));

    uint8_t r = ((pix >> 11) & 0x1F) << 3;
    uint8_t g = ((pix >> 5)  & 0x3F) << 2;
    uint8_t b = ( pix        & 0x1F) << 3;
    Serial.write(r); Serial.write(g); Serial.write(b);
  }
  Serial.println("\nEND_IMAGE");
}

static void sendHello() {
  Serial.print("{\"type\":\"hello\",\"proto\":\"nicla-vision-serial/1.0\"");
  Serial.print(",\"baud\":"); Serial.print(BAUDRATE);
  Serial.print(",\"cam\":{\"w\":"); Serial.print(CAM_W); Serial.print(",\"h\":"); Serial.print(CAM_H); Serial.print("}");
  Serial.print(",\"in_shape\":["); Serial.print(IN_N); Serial.print(","); Serial.print(IN_C); Serial.print(","); Serial.print(IN_H); Serial.print(","); Serial.print(IN_W); Serial.print("]");
  Serial.print(",\"out_len\":"); Serial.print(OUT_LEN);
  Serial.println(",\"status\":\"READY\"}");
}

static void sendInfo() {
  Serial.print("{\"type\":\"info\",\"in_shape\":[");
  Serial.print(IN_N); Serial.print(","); Serial.print(IN_C); Serial.print(","); Serial.print(IN_H); Serial.print(","); Serial.print(IN_W); Serial.print("]");
  Serial.print(",\"out_len\":"); Serial.print(OUT_LEN);
  Serial.print(",\"cam\":{\"w\":"); Serial.print(CAM_W); Serial.print(",\"h\":"); Serial.print(CAM_H); Serial.print("}}");
  Serial.println();
}

static void sendPong() {
  Serial.print("{\"type\":\"pong\",\"t_ms\":");
  Serial.print(millis());
  Serial.println("}");
}

static void runInfer(bool alsoImage) {
  if (g_busy) {
    Serial.println("{\"type\":\"busy\"}");
    return;
  }
  g_busy = true;

  captureToInputCenterCrop96();

  float *out = nullptr;
  unsigned long t0 = micros();
  int rc = predict(inputData, inputShape, 4, &out);
  unsigned long dt = micros() - t0;

  if (rc != 0 || !out) {
    Serial.println("{\"type\":\"error\",\"where\":\"predict\",\"code\":-1}");
    g_busy = false;
    return;
  }

  int best_idx = 0; float best_val = out[0];
  for (int i = 1; i < OUT_LEN; ++i) if (out[i] > best_val) { best_val = out[i]; best_idx = i; }

  Serial.print("{\"type\":\"result\",\"class\":");
  Serial.print(best_idx);
  Serial.print(",\"score\":");
  Serial.print(best_val, 6);
  Serial.print(",\"latency_us\":");
  Serial.print(dt);
  Serial.println("}");

  if (alsoImage) { sendImagePPM(); }

  g_busy = false;
}

void setup() {
  Serial.begin(115200);
  RPC.begin();
  cam.begin(CAMERA_R160x120, CAMERA_RGB565, 30);

  uint32_t t0 = millis();
  while (!Serial && (millis() - t0) < 4000) delay(10);

  // NO banner prints here; keep serial clean
  if (qspi_init_16mb(&hqspi) != HAL_OK) {
    Serial.println("{\"type\":\"error\",\"where\":\"qspi_init\"}");
    for(;;) {}
  }
  if (enable_quad(&hqspi) != HAL_OK) {
    Serial.println("{\"type\":\"error\",\"where\":\"qspi_quad\"}");
    for(;;) {}
  }
  if (qspi_enter_mmap(&hqspi) != HAL_OK) {
    Serial.println("{\"type\":\"error\",\"where\":\"qspi_mmap\"}");
    for(;;) {}
  }
}

void loop() {
  if (readLine()) {
    String cmd = String(cmdBuf);
    cmd.trim();
    cmd.toUpperCase();

    if (cmd == "HELLO") {
      sendHello();
    } else if (cmd == "PING") {
      sendPong();
    } else if (cmd == "INFO") {
      sendInfo();
    } else if (cmd == "IMAGE") {
      sendImagePPM();
    } else if (cmd == "INFER") {
      runInfer(false);
    } else if (cmd == "INFER+IMAGE") {
      runInfer(true);
    } else if (cmd.length() == 0) {
      // ignore
    } else {
      Serial.print("{\"type\":\"error\",\"where\":\"command\",\"msg\":\"unknown: ");
      Serial.print(cmd); Serial.println("\"}");
    }
}
}
