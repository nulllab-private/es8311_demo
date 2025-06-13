#include <driver/i2s_std.h>

#include "clogger.h"
#include "es8311.h"

namespace {
#include "pcm.h"

constexpr uint32_t kSampeRate = 16000;
constexpr uint32_t kMclkMultiple = 384;  // 256 is enough if not using 24-bit data width
constexpr uint32_t kMclkFreqHz = kSampeRate * kMclkMultiple;

constexpr gpio_num_t kEs8311I2sMclk = GPIO_NUM_11;
constexpr gpio_num_t kEs8311I2sSclk = GPIO_NUM_10;
constexpr gpio_num_t kEs8311I2sWs = GPIO_NUM_8;
constexpr gpio_num_t kEs8311I2sDsdin = GPIO_NUM_7;
constexpr gpio_num_t kEs8311I2sAsdout = GPIO_NUM_9;
constexpr gpio_num_t kEs8311I2cScl = GPIO_NUM_12;
constexpr gpio_num_t kEs8311I2cSda = GPIO_NUM_13;

i2c_master_bus_handle_t g_i2c_master_bus = nullptr;
i2s_chan_handle_t g_es8311_tx_handle = 0;
i2s_chan_handle_t g_es8311_rx_handle = 0;

void InitEs8311I2s() {
  CLOGI();

  i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_1, I2S_ROLE_MASTER);
  chan_cfg.auto_clear = true;  // Auto clear the legacy data in the DMA buffer
  ESP_ERROR_CHECK(i2s_new_channel(&chan_cfg, &g_es8311_tx_handle, &g_es8311_rx_handle));
  //   ESP_ERROR_CHECK(i2s_new_channel(&chan_cfg, &g_es8311_tx_handle, nullptr));
  i2s_std_config_t std_cfg = {
      .clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(kSampeRate),
      .slot_cfg =
          {
              .data_bit_width = I2S_DATA_BIT_WIDTH_16BIT,
              .slot_bit_width = I2S_SLOT_BIT_WIDTH_AUTO,
              .slot_mode = I2S_SLOT_MODE_MONO,
              .slot_mask = I2S_STD_SLOT_LEFT,
              .ws_width = I2S_DATA_BIT_WIDTH_16BIT,
              .ws_pol = false,
              .bit_shift = true,
#ifdef I2S_HW_VERSION_2
              .left_align = true,
              .big_endian = false,
              .bit_order_lsb = false,
#endif
          },
      .gpio_cfg =
          {
              .mclk = kEs8311I2sMclk,
              .bclk = kEs8311I2sSclk,
              .ws = kEs8311I2sWs,
              .dout = kEs8311I2sDsdin,
              .din = kEs8311I2sAsdout,
              //   .din = GPIO_NUM_NC,
              .invert_flags =
                  {
                      .mclk_inv = false,
                      .bclk_inv = false,
                      .ws_inv = false,
                  },
          },
  };

  std_cfg.clk_cfg.mclk_multiple = static_cast<i2s_mclk_multiple_t>(kMclkMultiple);

  ESP_ERROR_CHECK(i2s_channel_init_std_mode(g_es8311_tx_handle, &std_cfg));
  ESP_ERROR_CHECK(i2s_channel_init_std_mode(g_es8311_rx_handle, &std_cfg));
  ESP_ERROR_CHECK(i2s_channel_enable(g_es8311_tx_handle));
  ESP_ERROR_CHECK(i2s_channel_enable(g_es8311_rx_handle));
}

void InitEs8311() {
  CLOGI();
  const i2c_master_bus_config_t i2c_master_bus_config = {
      .i2c_port = I2C_NUM_1,
      .sda_io_num = kEs8311I2cSda,
      .scl_io_num = kEs8311I2cScl,
      .clk_source = I2C_CLK_SRC_DEFAULT,
      .glitch_ignore_cnt = 7,
      .intr_priority = 0,
      .trans_queue_depth = 0,
      .flags =
          {
              .enable_internal_pullup = 1,
              .allow_pd = 0,
          },
  };

  ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_master_bus_config, &g_i2c_master_bus));
  CLOGI("g_i2c_master_bus: %p", g_i2c_master_bus);

  es8311_handle_t es_handle = es8311_create(g_i2c_master_bus, ES8311_ADDRRES_0);
  const es8311_clock_config_t es_clk = {
      .mclk_inverted = false, .sclk_inverted = false, .mclk_from_mclk_pin = true, .mclk_frequency = kMclkFreqHz, .sample_frequency = kSampeRate};
  ESP_ERROR_CHECK(es8311_init(es_handle, &es_clk, ES8311_RESOLUTION_16, ES8311_RESOLUTION_16));
  ESP_ERROR_CHECK(es8311_sample_frequency_config(es_handle, kSampeRate * kMclkMultiple, kSampeRate));
  ESP_ERROR_CHECK(es8311_voice_volume_set(es_handle, 80, nullptr));
  ESP_ERROR_CHECK(es8311_microphone_config(es_handle, false));
  ESP_ERROR_CHECK(es8311_microphone_gain_set(es_handle, ES8311_MIC_GAIN_18DB));
}
}  // namespace

void setup() {
  CLOGI();
  InitEs8311I2s();
  InitEs8311();

  size_t bytes_written = 0;
  CLOGI("i2s_channel_write");
  ESP_ERROR_CHECK(i2s_channel_write(g_es8311_tx_handle, kPcmData, sizeof(kPcmData), &bytes_written, UINT32_MAX));
  CLOGI("i2s_channel_write bytes_written: %u", bytes_written);
}

void loop() {
  constexpr size_t kBufferSize = 2 * 16000 / 1000 * 80;
  uint8_t s_buffer[kBufferSize] = {0};
  size_t bytes_read = 0;

  ESP_ERROR_CHECK(i2s_channel_read(g_es8311_rx_handle, s_buffer, kBufferSize, &bytes_read, UINT32_MAX));
  CLOGI("bytes_read: %zu", bytes_read);
  ESP_ERROR_CHECK(i2s_channel_write(g_es8311_tx_handle, s_buffer, bytes_read, nullptr, UINT32_MAX));
}