//
// Created by kevinh on 9/1/20.
//

#include "HardwareSPI.h"
#include "SPIChip.h"
#include "Utility.h"
#include "logging.h"

#ifdef PORTDUINO_LINUX_HARDWARE

extern "C" {
#include "mcp2210.h"
}

#include "PortduinoGPIO.h"

#include <assert.h>
#include <linux/spi/spi.h>
#include <linux/spi/spidev.h>

#include <fcntl.h>

#include <mutex>
#include <map>
#include <memory>
#include <iostream>


class MCPSPIChip : public SPIChip {
  private:
    std::mutex SPIMutex;
    std::mutex ChipMutex;
    uint32_t defaultSpeed = 2000000;
    int fd;
    mcp2210_packet spi_config = {0,};
    
  public:
    MCPSPIChip(uint32_t default_frequency, std::string dev) {
      defaultSpeed = default_frequency;
      if (defaultSpeed <= 0) {
        defaultSpeed = 2000000;
      }

      fd = open(dev.c_str(), O_RDWR);
      assert(fd);

      // Configure the MCP2210
      int ret;
      mcp2210_packet spi_cancel_packet = {0,};
      ret = mcp2210_command(fd, spi_cancel_packet, MCP2210_SPI_CANCEL);
      assert(ret >= 0);

	    ret = mcp2210_command(fd, spi_config, MCP2210_SPI_GET);
      assert(ret >= 0);
      mcp2210_spi_set_transaction_size(spi_config, 8);
			mcp2210_spi_set_bitrate(spi_config, defaultSpeed);
      mcp2210_spi_set_mode(spi_config, 0);

      mcp2210_packet spi_temp = {0,};
      memcpy(spi_temp, spi_config, MCP2210_PACKET_SIZE);
      mcp2210_command(fd, spi_temp, MCP2210_SPI_SET);

      mcp2210_packet chip_packet = {0,};
	    ret = mcp2210_command(fd, chip_packet, MCP2210_CHIP_GET);
      assert(ret >= 0);
      for (ret = 0; ret <= MCP2210_GPIO_PINS; ret++) {
        mcp2210_chip_set_function(chip_packet, ret, MCP2210_CHIP_PIN_GPIO);
      }
      mcp2210_command(fd, chip_packet, MCP2210_CHIP_SET);

      mcp2210_packet gpio_dir_packet = {0,};
	    ret = mcp2210_command(fd, chip_packet, MCP2210_GPIO_DIR_GET);
      assert(ret >= 0);
      for (ret = 0; ret <= MCP2210_GPIO_PINS; ret++) {
        mcp2210_gpio_set_pin(gpio_dir_packet, ret, 1); // INPUT
      }
	    ret = mcp2210_command(fd, chip_packet, MCP2210_GPIO_DIR_SET);
      assert(ret >= 0);
    }

    /**
     * Do a SPI transaction to the selected device
     *
     * @param outBuf if NULL it will be not used (zeros will be sent)
     * @param inBuf if NULL it will not be used (device response bytes will be
     * discarded)
     * @param deassertCS after last transaction (if not set, it will be left
     * asserted)
     * @return 0 for success, else ERRNO fault code
     */
    int transfer(const uint8_t *outBuf, uint8_t *inBuf, size_t bufLen,
                bool deassertCS = true) override {
      unsigned char* packet = (unsigned char*)malloc(bufLen);
      memcpy(packet, outBuf, bufLen);

      printf("MCSPI::TX(%d): ", bufLen);
      for (int i = 0; i < bufLen; i++) {
        printf("%02x ", packet[i]);
      }
      printf("\n");

      ChipMutex.lock();

      if (bufLen != mcp2210_spi_get_transaction_size(spi_config)) {
        mcp2210_spi_set_transaction_size(spi_config, bufLen);

        mcp2210_packet spi_temp = {0,};
        memcpy(spi_temp, spi_config, MCP2210_PACKET_SIZE);
        mcp2210_command(fd, spi_temp, MCP2210_SPI_SET);
      }
      int ret = mcp2210_spi_transfer(fd, (char*)packet, bufLen);
      assert(ret == 0);

      ChipMutex.unlock();

      printf("MCSPI::RX(%d): ", bufLen);
      for (int i = 0; i < bufLen; i++) {
        printf("%02x ", packet[i]);
      }
      printf("\n");

      if (inBuf != NULL) {
        memcpy(inBuf, packet, bufLen);
      }

      free(packet);
      return 0;
    }

    int gpioRead(uint8_t pin) {
      int ret;

      ChipMutex.lock();

      mcp2210_packet gpio_packet = {0,};
	    ret = mcp2210_command(fd, gpio_packet, MCP2210_GPIO_VAL_GET);
      assert(ret >= 0);

      ret = mcp2210_gpio_get_pin(gpio_packet, pin);

      ChipMutex.unlock();
      return ret;
    }

    void gpioWrite(uint8_t pin, PinStatus status) {
      int ret;

      ChipMutex.lock();

      // For some reason we need to remind the MCP2210 that we want to output...
      this->gpioSetModeUnsafe(pin, OUTPUT);

      mcp2210_packet gpio_packet = {0,};
	    ret = mcp2210_command(fd, gpio_packet, MCP2210_GPIO_VAL_GET);
      assert(ret >= 0);

      mcp2210_gpio_set_pin(gpio_packet, pin, status == HIGH ? 1 : 0);

      ret = mcp2210_command(fd, gpio_packet, MCP2210_GPIO_VAL_SET);
      assert(ret >= 0);

      ChipMutex.unlock();
    }

    void gpioSetMode(uint8_t pin, PinMode mode) {
      ChipMutex.lock();
      this->gpioSetModeUnsafe(pin, mode);
      ChipMutex.unlock();
    }

    void beginTransaction(uint32_t clockSpeed) override {
      SPIMutex.lock();
      if (clockSpeed <= 0) {
        clockSpeed = defaultSpeed;
      }

      ChipMutex.lock();

			mcp2210_spi_set_bitrate(spi_config, clockSpeed);
      mcp2210_command(fd, spi_config, MCP2210_SPI_SET);

      mcp2210_packet spi_temp = {0,};
      memcpy(spi_temp, spi_config, MCP2210_PACKET_SIZE);
	    int ret = mcp2210_command(fd, spi_temp, MCP2210_SPI_GET);
      assert(ret >= 0);

      ChipMutex.unlock();
    }

    void endTransaction() override {
      SPIMutex.unlock();
    }

private:
    void gpioSetModeUnsafe(uint8_t pin, PinMode mode) {
      int ret;

      mcp2210_packet gpio_dir_packet = {0,};
      ret = mcp2210_command(fd, gpio_dir_packet, MCP2210_GPIO_DIR_GET);
      assert(ret >= 0);

      mcp2210_gpio_set_pin(gpio_dir_packet, pin, (mode == OUTPUT) ? 0 : 1);

      ret = mcp2210_command(fd, gpio_dir_packet, MCP2210_GPIO_DIR_SET);
      assert(ret >= 0);
    }
};

class MCPGPIO : public GPIOPin
{
private:
    uint8_t pin;
    std::shared_ptr<MCPSPIChip> chip;
public:
    MCPGPIO(pin_size_t pin, String name, std::shared_ptr<MCPSPIChip> chip) : GPIOPin(pin, name), pin(pin), chip(chip) {}

    void setPinMode(PinMode m) override {
      this->chip->gpioSetMode(this->pin, m);
      GPIOPin::setPinMode(m);
    }

    void writePin(PinStatus s) override {
      this->chip->gpioWrite(this->pin, s);
      GPIOPin::writePin(s);
    }

protected:
    PinStatus readPinHardware() override {
      return this->chip->gpioRead(this->pin) ? HIGH : LOW;
    }
};

namespace arduino {

class MCSPICls : HardwareSPI {
private:
  std::string chipName;
  std::shared_ptr<MCPSPIChip> modSpiChip;

public:
  MCSPICls() {
    this->chipName = "";
  }

  void begin(uint32_t freq) override {
    if (this->modSpiChip == nullptr) {
      this->modSpiChip = std::make_shared<MCPSPIChip>(freq, this->chipName);
      this->spiChip = this->modSpiChip;

      for (int i = 0; i <= MCP2210_GPIO_PINS; i++) {
        std::string name = "MCP_GPIO" + std::to_string(i);
        MCPGPIO *gpio = new MCPGPIO(i, name.c_str(), this->modSpiChip);
        gpio->setSilent();
        gpioBind(gpio);
      }
    }
  }

  void begin(const char *name, uint32_t freq) override {
    if (name != NULL) {
      this->chipName = std::string(name);
    }
    this->begin(freq);
  }

  int gpioRead(const uint8_t pin) {
    return this->modSpiChip->gpioRead(pin);
  }
};

} // namespace arduino

MCSPICls MCSPI;

#endif
