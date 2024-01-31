#include "Arduino.h"
#include "LoRa_E220.h"
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>

#define SEALEVELPRESSURE_HPA (1013.25)

LoRa_E220 e220ttl(&Serial2, 15, 21, 19); // Configuração do módulo LoRa (RX, AUX, M0, M1)
Adafruit_BME280 bme; // Objeto para o sensor de pressão, temperatura e umidade (I2C)

void setup() {
  Serial.begin(9600); // Inicializa a comunicação serial com baud rate de 9600
  delay(500); // Aguarda 500ms

  e220ttl.begin(); // Inicializa o módulo LoRa

  Serial.println("Hi, I'm going to send sensor data!"); // Mensagem de inicialização

  bool status = bme.begin(0x76); // Inicializa o sensor BME280 com endereço I2C 0x76
  if (!status) {
    Serial.println("Could not find a valid BME280 sensor, check wiring!"); // Mensagem de erro se o sensor não for encontrado
    while (1); // Loop infinito em caso de erro
  }

  delay(1000); // Aguarda 1 segundo
}

void loop() {
  float temperature = bme.readTemperature(); // Lê a temperatura do sensor
  float pressure = bme.readPressure() / 100.0F; // Lê a pressão atmosférica e converte para hPa
  float altitude = bme.readAltitude(SEALEVELPRESSURE_HPA); // Lê a altitude com base na pressão
  float humidity = bme.readHumidity(); // Lê a umidade do sensor

  // Constrói uma string com os dados do sensor
  String sensorData = "Temp:" + String(temperature) + " Pressure:" + String(pressure) +
                      " Altitude:" + String(altitude) + " Humidity:" + String(humidity);

  // Adiciona mensagens de depuração
  Serial.println("Sending sensor data: " + sensorData);

  // Envia dados do sensor usando o módulo LoRa
  ResponseStatus rs = e220ttl.sendBroadcastFixedMessage(23, sensorData);
  Serial.println("Send status: " + rs.getResponseDescription());

  delay(10000); // Ajuste conforme necessário para evitar congestionamento (10 segundos de intervalo entre transmissões)
}
