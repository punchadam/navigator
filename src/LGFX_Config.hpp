#pragma once

#define LGFX_USE_V1
#include <LovyanGFX.hpp>

class LGFX : public lgfx::LGFX_Device {
  lgfx::Panel_GC9A01 _panel_instance;
  lgfx::Bus_SPI      _bus_instance;
  lgfx::Light_PWM    _light_instance;

public:
  LGFX(void) {
    { // SPI bus
      auto cfg = _bus_instance.config();
      cfg.spi_host    = SPI2_HOST;
      cfg.spi_mode    = 0;
      cfg.freq_write  = 40000000;         // 40 MHz
      cfg.freq_read   = 16000000;
      cfg.spi_3wire   = true;             // no MISO line
      cfg.use_lock    = true;
      cfg.dma_channel = SPI_DMA_CH_AUTO;
      cfg.pin_sclk    = 7;
      cfg.pin_mosi    = 9;
      cfg.pin_miso    = -1;
      cfg.pin_dc      = 8;
      _bus_instance.config(cfg);
      _panel_instance.setBus(&_bus_instance);
    }

    { // -- Panel --
      auto cfg = _panel_instance.config();
      cfg.pin_cs          = -1; // tied to gnd
      cfg.pin_rst         = -1; // 10k pullup
      cfg.pin_busy        = -1;
      cfg.panel_width     = 240;
      cfg.panel_height    = 240;
      cfg.offset_x        = 0;
      cfg.offset_y        = 0;
      cfg.offset_rotation = 0;
      cfg.readable        = false;
      cfg.invert          = true;   // GC9A01 needs inversion ON
      cfg.rgb_order       = false;  // false = BGR (GC9A01 default)
      cfg.dlen_16bit      = false;
      cfg.bus_shared      = false;
      _panel_instance.config(cfg);
    }

    setPanel(&_panel_instance);
  }
};