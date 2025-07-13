#include <Arduino.h>
#include <SPI.h>
#include "mcp_canbus.h"
#include "driver/twai.h"

#define POLLING_RATE_MS 20

#define CAN1_SEND_ID     0x420
#define CAN1_RPM_FILTER_ID 0x102
#define CAN2_LAMBDA_ID   0x400
#define CAN2_SEND_RPM_ID 0x5E8

MCP_CAN CAN(CS);  // Use your existing CS define

float lambda;
uint64_t formula_factor = 8578426943158620;
uint64_t formula_offset = 843769539405756;

// Send calculated AFR via CAN1 (TWAI)
void sendCAN1(float d) {
  uint64_t message_data = d * formula_factor + formula_offset;

  twai_message_t message;
  message.identifier = CAN1_SEND_ID;
  message.extd = 0;
  message.rtr = 0;
  message.data_length_code = 8;

  for (int i = 0; i < 8; i++) {
    message.data[i] = (message_data >> (8 * (7 - i))) & 0xFF;
  }

  if (twai_transmit(&message, pdMS_TO_TICKS(1000)) == ESP_OK) {
    Serial.println("CAN1: Sent AFR message");
  } else {
    Serial.println("CAN1: Failed to send AFR message");
  }
}

// Send fixed RPM message to CAN2
void sendCAN2RPMMessage() {
  byte txData[8] = {0};
  txData[2] = (1000 >> 8) & 0xFF;
  txData[3] = 1000 & 0xFF;

  if (CAN.sendMsgBuf(CAN2_SEND_RPM_ID, 0, 8, txData) == CAN_OK) {
    Serial.println("CAN2: Sent 1000 RPM message");
  } else {
    Serial.println("CAN2: Failed to send RPM message");
  }
}

// Read Lambda from CAN2
void readCAN2() {
  unsigned char len = 0;
  unsigned char buf[8];

  if (CAN_MSGAVAIL == CAN.checkReceive()) {
    CAN.readMsgBuf(&len, buf);

    if (CAN.getCanId() == CAN2_LAMBDA_ID) {
      uint16_t high_byte = buf[0];
      uint16_t low_byte = buf[1];
      lambda = (high_byte << 8 | low_byte) / 1000.0;

      uint8_t status = buf[3];
      float d;

      if (status == 1) d = 1;
      else if (status == 2) d = 2;
      else if (status == 3) d = lambda * 14.7;
      else return;

      Serial.print("Lambda: ");
      Serial.println(lambda);
      Serial.print("AFR: ");
      Serial.println(d);

      sendCAN1(d);

      digitalWrite(LED_BUILTIN, LOW);
      delay(100);
      digitalWrite(LED_BUILTIN, HIGH);
    }
  }
}

// Read RPM from CAN1
void readCAN1() {
  twai_message_t message;

  if (twai_receive(&message, pdMS_TO_TICKS(10)) == ESP_OK) {
    if (message.identifier == CAN1_RPM_FILTER_ID && message.data_length_code >= 2) {
      uint16_t rpm = (message.data[0] << 8) | message.data[1];
      Serial.print("CAN1: RPM Received = ");
      Serial.println(rpm);

      if (rpm > 1000) {
        sendCAN2RPMMessage();
      }
    }
  }
}

void setup() {
  Serial.begin(115200);
  pinMode(LED_BUILTIN, OUTPUT);

  Serial.println("Initializing CAN1 (TWAI)");

  // Use your original CAN1 GPIOs here — assumed to be defined or hardcoded
  twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT((gpio_num_t)CAN1_TX, (gpio_num_t)CAN1_RX, TWAI_MODE_NORMAL);
  twai_timing_config_t t_config = TWAI_TIMING_CONFIG_500KBITS();

  // Accept only messages with ID 0x102 on CAN1
  twai_filter_config_t f_config = {
    .acceptance_code = (CAN1_RPM_FILTER_ID << 21),
    .acceptance_mask = ~(0x7FF << 21),
    .single_filter = true
  };

  if (twai_driver_install(&g_config, &t_config, &f_config) == ESP_OK) {
    Serial.println("CAN1 Driver initialized");
  } else {
    Serial.println("Failed to initialize CAN1 driver");
    return;
  }

  if (twai_start() == ESP_OK) {
    Serial.println("CAN1 interface started");
  } else {
    Serial.println("Failed to start CAN1");
    return;
  }

  if (twai_reconfigure_alerts(TWAI_ALERT_RX_DATA, NULL) == ESP_OK) {
    Serial.println("CAN1 Alerts reconfigured");
  } else {
    Serial.println("Failed to reconfigure alerts");
    return;
  }

  // Initialize MCP2515 CAN2
  if (CAN_OK == CAN.begin(CAN_500KBPS)) {
    Serial.println("CAN2 interface started");
  } else {
    Serial.println("Failed to start CAN2");
    while (1);
  }
}

void loop() {
  readCAN1();
  readCAN2();
}
