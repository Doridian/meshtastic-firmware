// Initialize i2c bus. We need a bus to not hang on boot
#define HAS_SCREEN 0
#define I2C_SDA 39
#define I2C_SCL 40

// GPS
#undef GPS_RX_PIN
#undef GPS_TX_PIN
#define HAS_GPS 0

#undef PIN_BUZZER

#define BUTTON_PIN 0
#define BUTTON_NEED_PULLUP

#define LORA_SCK 7
#define LORA_MISO 16
#define LORA_MOSI 15
#define LORA_CS 6

#define LORA_RESET 17
#define LORA_BUSY 18
#define LORA_DIO1 11
// #define LORA_DIO2 10
#define LORA_RXEN 4
#define LORA_TXEN 5

#define USE_SX1262

#define SX126X_CS LORA_CS
#define SX126X_DIO1 LORA_DIO1
// #define SX126X_DIO2 LORA_DIO2
#define SX126X_BUSY LORA_BUSY
#define SX126X_RESET LORA_RESET
#define SX126X_RXEN LORA_RXEN
#define SX126X_TXEN LORA_TXEN

#define SX126X_DIO3_TCXO_VOLTAGE 1.8
#undef TCXO_OPTIONAL
#undef SX126X_DIO2_AS_RF_SWITCH
