#include "Arduino.h"
#include "LoRa_E220.h"

LoRa_E220 e220ttl(&Serial2, 15, 21, 19); // Configuração do módulo LoRa (RX, AUX, M0, M1)

void setup() {
  Serial.begin(9600); // Inicializa a comunicação serial com baud rate de 9600
  delay(500); // Aguarda 500ms

  e220ttl.begin(); // Inicializa o módulo LoRa

  Serial.println("Start receiving!"); // Mensagem de inicialização
}

void loop() {
  // Verifica se há dados disponíveis no módulo LoRa
  if (e220ttl.available() > 1) {
    Serial.println("Message received!"); // Mensagem indicando que uma mensagem foi recebida

#ifdef ENABLE_RSSI
    // Recebe a mensagem juntamente com o RSSI (Received Signal Strength Indication)
    ResponseContainer rc = e220ttl.receiveMessageRSSI();
#else
    // Recebe a mensagem sem o RSSI
    ResponseContainer rc = e220ttl.receiveMessage();
#endif

    // Se ocorrer algum erro, imprime a descrição do erro
    if (rc.status.code != 1) {
      Serial.println(rc.status.getResponseDescription());
    } else {
      // Imprime os dados recebidos com sucesso
      Serial.println(rc.status.getResponseDescription());
      Serial.println("Received Data: " + rc.data);

#ifdef ENABLE_RSSI
      // Se o RSSI estiver habilitado, imprime o valor do RSSI
      Serial.print("RSSI: ");
      Serial.println(rc.rssi, DEC);
#endif
    }
  }
}
