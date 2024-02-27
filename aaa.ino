#include <SoftwareSerial.h>
#include <HardwareSerial.h>
#include "FS.h"
#include "SPI.h"
#include <SD.h>
//TCS ->CONFERIR OS PINOS COM O TCS/EPS
#include <OneWire.h>
#include <DallasTemperature.h>
#define sensores 12
#define RELE 13
#define TEMPERATURE_PRECISION 3
//EPS
#include <Wire.h>
#include <INA3221.h>
#define PRINT_DEC_POINTS 4
//INICIO HOUSEKEEPING
//HK e TTeC
const int TX_HK_TTeC = 4; // Pino TX manda
const int RX_HK_TTeC = 15;  // Pino RX recebe
//HK e ADCS
const int TX_HK_ADCS = 32;
const int RX_HK_ADCS = 35;
//HK e Payload(artificial)
const int TX_Payload = 16;
const int RX_Payload = 17;
SoftwareSerial espSerialTTeC(RX_HK_TTeC, TX_HK_TTeC);//Telecomando
SoftwareSerial espSerialADCS(RX_HK_ADCS, TX_HK_ADCS);//ADCS,
SoftwareSerial espSerialPayload(RX_Payload, TX_Payload);//ADCS,
#define rele_Payload 33
//--------------------------------------------------------------------//
String mensagem_new = "";
String mensagem_old = "";
String PLD;
String PLD_string;
String ADCS;
String ADCS_string;
File data_record;
const int chipSelect = 2;
// FIM HOUSEKEEPINGnos
// INICIO TCS
OneWire oneWire(sensores);
DallasTemperature sensors(&oneWire);
DeviceAddress battery, inside, enviroment1, enviroment2;
float temp_battery;
float temp_inside;
float temp_enviroment1;
float temp_enviroment2;
String addr_battery;
String addr_inside;
String addr_enviroment1;
String addr_enviroment2;
// FIM TCS
//INICIO EPS
INA3221 ina_0(INA3221_ADDR40_GND);
const int stepUpEnablePin = 5;       // Pino de controle para ativar/desativar o conversor step-up / COLOCAR OS OUTROS 3 E MUDAR OS ARGUMENTOS QUE USAM ELE NO CÓDIGO
const int stepUpEnablePin2 = 27;       // Pino de controle para ativar/desativar o conversor step-up / COLOCAR OS OUTROS 3 E MUDAR OS ARGUMENTOS QUE USAM ELE NO CÓDIGO
const int stepUpEnablePin3 = 25;       // Pino de controle para ativar/desativar o conversor step-up / COLOCAR OS OUTROS 3 E MUDAR OS ARGUMENTOS QUE USAM ELE NO CÓDIGO
const int stepUpEnablePin4 = 14;       // Pino de controle para ativar/desativar o conversor step-up / COLOCAR OS OUTROS 3 E MUDAR OS ARGUMENTOS QUE USAM ELE NO CÓDIGO
const float solarVoltageMax = 6.0;    // Tensão máxima dos painéis solares
const float solarVoltageMin = 0.1;    // Tensão mínima dos painéis solares
const float batteryVoltageMax = 4.2;  // Tensão máxima da bateria
const float batteryVoltageMin = 2.5;  // Tensão mínima da bateria

float solarVoltage1 = 0.0;
float solarCurrent1 = 0.0;
float conversorVoltage1 = 0.0;
float conversorCurrent1 = 0.0;
float previousPower1 = 0.0;
float solarPower1 = 0.0;
float conversorPower1 = 0.0;

float solarVoltage2 = 0.0;
float solarCurrent2 = 0.0;
float conversorVoltage2 = 0.0;
float conversorCurrent2 = 0.0;
float previousPower2 = 0.0;
float solarPower2 = 0.0;

float solarVoltage3 = 0.0;
float solarCurrent3 = 0.0;
float conversorVoltage3 = 0.0;
float conversorCurrent3 = 0.0;
float previousPower3 = 0.0;
float solarPower3 = 0.0;

float solarVoltage4 = 0.0;
float solarCurrent4 = 0.0;
float conversorVoltage4 = 0.0;
float conversorCurrent4 = 0.0;
float previousPower4 = 0.0;
float solarPower4 = 0.0;

float batteryVoltage = 0.0;
float batteryCurrent = 0.0;
float batteryPower = 0.0;


int incremento0 = 0;
int incremento1 = 0;
int incremento2 = 0;
int incremento3 = 0;
//FIM EPS
// STRINGS PARA COMUNICAÇÃO ENTRE CONTROLADORES
String dados_EPS;
String dados_corrente;
String dados_tensao;
String dados_TCS;
//--------------------------------------//
String battery_Voltage;
String battery_Current;
String solar_Voltage1;
String solar_Current1;
String solar_Voltage2;
String solar_Current2;
String solar_Voltage3;
String solar_Current3;
String solar_Voltage4;
String solar_Current4;
String conversor_Voltage1;
String conversor_Current1;
String conversor_Voltage2;
String conversor_Current2;
String conversor_Voltage3;
String conversor_Current3;
String conversor_Voltage4;
String conversor_Current4;
//-------------------------------------//
String temp_battery_string;
String temp_inside_string;
String temp_environment1_string;
String temp_environment2_string;
//------------------------------------//
String dados_HK;
String dados_HK_PLD;
int tempo_passado;
int tempo_atual;
int tempo_ADCS;
int flag_HK;
int flag_PLD;
int flag_ADCS;
int flag_timing;
int flag_control;
int tentativas;
void current_measure_init() {
  ina_0.begin(&Wire);
  ina_0.reset();
  ina_0.setShuntRes(1, 1, 1);
}
void mult(uint8_t bus) {
  Wire.beginTransmission(0x70);  // TCA9548A address
  Wire.write(1 << bus);          // send byte to select bus
  Wire.endTransmission();
}
//FIM EPS
void setup(){
  if(!SD.begin(chipSelect)){
    Serial.println("Card Mount Failed");
  }

  Serial.begin(9600);
  espSerialPayload.begin(9600);
  espSerialTTeC.begin(9600);
  espSerialADCS.begin(9600);
  Serial.setTimeout(300);
  espSerialPayload.setTimeout(300);
  espSerialTTeC.setTimeout(300);
  espSerialADCS.setTimeout(300);

  flag_HK = 0;
  flag_ADCS = 0;
  flag_PLD = 1;
  flag_timing = 0;

  //INICIO TCS
  sensors.begin();
  pinMode(RELE, OUTPUT);
  digitalWrite(RELE, LOW);
  Serial.print("Device 0 Address: ");
  addr_battery = sensors.getAddress(battery, 0);
  Serial.println(addr_battery);
  Serial.print("Device 1 Address: ");
  addr_inside = sensors.getAddress(inside, 1);
  Serial.println(addr_inside);
  Serial.print("Device 2 Address: ");
  addr_enviroment1 = sensors.getAddress(enviroment1, 2);
  Serial.println(addr_enviroment1);
  Serial.print("Device 3 Address: ");
  addr_enviroment2 = sensors.getAddress(enviroment2, 3);
  Serial.println(addr_enviroment2);
  sensors.setResolution(battery, TEMPERATURE_PRECISION);
  sensors.setResolution(inside, TEMPERATURE_PRECISION);
  sensors.setResolution(enviroment1, TEMPERATURE_PRECISION);
  sensors.setResolution(enviroment2, TEMPERATURE_PRECISION);
  //FIM TCS

  //INICIO EPS
  pinMode(stepUpEnablePin, OUTPUT);
  //TROCAR OS PINOS PARA OS CORRETOS !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
  analogWrite(stepUpEnablePin, incremento0);  // Desativa o conversor inicialmente
  analogWrite(stepUpEnablePin2, incremento1);  // Desativa o conversor inicialmente
  analogWrite(stepUpEnablePin3, incremento2);  // Desativa o conversor inicialmente
  analogWrite(stepUpEnablePin4, incremento3);  // Desativa o conversor inicialmente
  current_measure_init();
  while (!Serial) {
    delay(10);
  }
  //FIM EPS
  //CARTÃO SD
  data_record.close();
  //----------------------RELE PAYLOAD--------------------//
  pinMode(rele_Payload, OUTPUT);
  digitalWrite(rele_Payload, HIGH);
}

void loop() {
  //-------------------------------SD---------------------------//
  data_record = SD.open("/data_record.txt", FILE_APPEND);
  //------------------RECEBER DADOS DO TT&C-----------//
  tentativas = 0;
  while (!espSerialTTeC.available() && !espSerialADCS.available() && tentativas < 2){
    delay(10);
    tentativas += 1;
  }
  if (espSerialTTeC.available() > 0) {
    mensagem_new = espSerialTTeC.readStringUntil('\n');
    if(mensagem_new != mensagem_old){
      Serial.print("Mensagem Recebida pelo Serial do TT&C: "); Serial.println(mensagem_new);
      if (mensagem_new.startsWith("H")){
        Serial.println(mensagem_new);
        //espSerialTTeC.println("HK Recebeu");
      }
      else if (mensagem_new.startsWith("P")){
        Serial2.println(mensagem_new); //Serial HK-PLD
        Serial.println("Enviado ao Payload");
        if (mensagem_new[1] == 'D') { //------------------------->COMANDO DE LIGAR E DESLIGAR O PAYLOAD, AJUSTAR A NECESSIDADE
          digitalWrite(rele_Payload, HIGH);
        }
         if (mensagem_new[1] == 'L') {
          digitalWrite(rele_Payload, LOW);
        }
      }
      else if (mensagem_new.startsWith("A") ){
        espSerialADCS.println(mensagem_new); //Serial HK-ADCS
        Serial.println("Enviado ao ADCS(ESP_4)");
      }
    }
  }
  //--------------------LER DADOS DO ADCS-----------------//
  if (espSerialADCS.available() > 0){
    ADCS = espSerialADCS.readStringUntil('\n');
  }
  Serial.println("----------------------");
  //----------------------INICIO TCS--------------------------//
  //CONFIRMAR SE PRECISA COLOCAR ENDERECO addr_battery AO INVES DE battery
  sensors.requestTemperatures();
  temp_battery = sensors.getTempC(battery);
  temp_inside = sensors.getTempC(inside);
  temp_enviroment1 = sensors.getTempC(enviroment1);
  temp_enviroment2 = sensors.getTempC(enviroment2);
  Serial.print("Battery Temp(C): ");Serial.println(sensors.getTempC(battery));
  Serial.print("Inside Temp(C): ");Serial.println(temp_inside);
  Serial.print("Enviroment1 Temp(C): ");Serial.println(temp_enviroment1);
  Serial.print("Enviroment2 Temp(C): ");Serial.println(temp_enviroment2);
  Serial.println("\n---------------------------\n");
  if(temp_battery < 5){
    digitalWrite(RELE, LOW);
    Serial.println("RELE LOW");
  }
  if(temp_battery > 8){
    digitalWrite(RELE, HIGH);
    Serial.println("RELE HIGH");
  }
  //FIM TCS
  //INICIO EPS

  mult(0);
  conversorVoltage2 = ina_0.getVoltage(INA3221_CH1); //readSolarVoltatage
  conversorCurrent2 = ina_0.getCurrent(INA3221_CH1); //readSolarCurrent
  conversorVoltage1 = ina_0.getVoltage(INA3221_CH2; //readSolarVoltatage
  conversorCurrent1 = ina_0.getCurrent(INA3221_CH2); //readSolarCurrent
  conversorVoltage4 = ina_0.getVoltage(INA3221_CH3); //readSolarVoltatage
  conversorCurrent4 = ina_0.getCurrent(INA3221_CH3); //readSolarCurrent

  mult(1);
  conversorVoltage3 = ina_0.getVoltage(INA3221_CH1); //readSolarVoltatage
  conversorCurrent3 = ina_0.getCurrent(INA3221_CH1);; //readSolarCurrent
  solarVoltage3 = ina_0.getVoltage(INA3221_CH2); //readSolarVoltatage
  conversorCurrent3 = ina_0.getCurrent(INA3221_CH2);; //readSolarCurrent
  batteryVoltage =  ina_0.getVoltage(INA3221_CH3);
  batteryCurrent = ina_0.getCurrent(INA3221_CH3);

  mult(7);
  solarVoltage4 = ina_0.getVoltage(INA3221_CH1); //readSolarVoltatage
  solarCurrent4 = ina_0.getCurrent(INA3221_CH1); //readSolarCurrent
  solarVoltage2 = ina_0.getVoltage(INA3221_CH2); //readSolarVoltatage
  solarCurrent2 = ina_0.getCurrent(INA3221_CH2); //readSolarCurrent
  solarVoltage1 = ina_0.getVoltage(INA3221_CH3); //readSolarVoltatage
  solarCurrent1 = ina_0.getCurrent(INA3221_CH3); //readSolarCurrent

  solarPower1 = conversorVoltage1 * conversorCurrent1;
  solarPower2 = conversorVoltage2 * conversorCurrent2;
  solarPower3 = conversorVoltage3 * conversorCurrent3;
  solarPower4 = conversorVoltage4 * conversorCurrent4;

  //VERIFICAR OS PINOS COM O EPS E ALTERAR NO CÓDIGO!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
  if (solarPower1 > previousPower1) {
    analogWrite(stepUpEnablePin, incremento0);
    incremento0 ++;
  }
  else {
    // Reduz a tensão para encontrar o ponto de máxima potência
    analogWrite(stepUpEnablePin, incremento0);
    incremento0 --;
    if (incremento0 <= 0){
      incremento0 = 0;
    }
  }

  if (solarPower2 > previousPower2) {
    analogWrite(stepUpEnablePin2, incremento1);
    incremento1 ++;
  }
  else {
    // Reduz a tensão para encontrar o ponto de máxima potência
    analogWrite(stepUpEnablePin2, incremento1);
    incremento1 --;
    if (incremento1 <= 0){
      incremento1 = 0;
    }
  }

  if (solarPower3 > previousPower3) {
    analogWrite(stepUpEnablePin3, incremento2);
    incremento2 ++;
  }
  else {
    // Reduz a tensão para encontrar o ponto de máxima potência
    analogWrite(stepUpEnablePin3, incremento2);
    incremento2 --;
    if (incremento2 <= 0){
      incremento2 = 0;
    }
  }

  if (solarPower4 > previousPower4) {
    analogWrite(stepUpEnablePin4, incremento3);
    incremento3 ++;
  }
  else {
    // Reduz a tensão para encontrar o ponto de máxima potência
    analogWrite(stepUpEnablePin4, incremento3);
    incremento3 --;
    if (incremento3 <= 0){
      incremento3 = 0;
    }
  }

  previousPower1 = solarPower1;
  previousPower2 = solarPower2;
  previousPower3 = solarPower3;
  previousPower4 = solarPower4;

  //FIM EPS
  //CONVERSÕES PRA STRING
  battery_Voltage = String(batteryVoltage, 2);
  battery_Current = String(batteryCurrent*10.0, 2);
  solar_Voltage1 = String(solarVoltage1, 2);
  solar_Current1 = String(solarCurrent1*10.0, 2);
  solar_Voltage2 = String(solarVoltage2, 2);
  solar_Current2 = String(solarCurrent2*10.0, 2);
  solar_Voltage3 = String(solarVoltage3, 2);
  solar_Current3 = String(solarCurrent3*10.0, 2);
  solar_Voltage4 = String(solarVoltage4, 2);
  solar_Current4 = String(solarCurrent4*10.0, 2);
  conversor_Voltage1 = String(conversorVoltage1, 2);
  conversor_Current1 = String(conversorCurrent1*10.0, 2);
  onversor_Voltage2 = String(conversorVoltage2, 2);
  conversor_Current2 = String(conversorCurrent2*10.0, 2);
  onversor_Voltage3 = String(conversorVoltage3, 2);
  conversor_Current3 = String(conversorCurrent3*10.0, 2);
  conversor_Voltage4 = String(conversorVoltage4, 2);
  conversor_Current4 = String(conversorCurrent4*10.0, 2);
  //---------------------------------------------//
  temp_battery_string = String(temp_battery, 2);
  temp_environment1_string = String(temp_enviroment1, 2);
  //------------------------------------------------------//
  dados_tensao = "V;" + battery_Voltage + ";" + conversor_Voltage1 + ";" + conversor_Voltage2 + ";" + conversor_Voltage3 + ";" + conversor_Voltage4;
  dados_corrente = "C;" + battery_Current + ";" + conversor_Current1 + ";" + conversor_Current2 + ";" + conversor_Current3 + ";" + conversor_Current4;
  dados_TCS = "T;" + temp_battery_string + ";" + temp_environment1_string;
  //---------------------------------------------------------------------//
  //dados_HK = dados_EPS + dados_TCS;
  Serial.print("flag_HK: "); Serial.println(flag_HK);
  Serial.print("flag_ADCS: "); Serial.println(flag_ADCS);
  Serial.print("flag_PLD: "); Serial.println(flag_PLD);
  Serial.print("flag_timing: "); Serial.println(flag_timing);
  tempo_atual = millis();
  //CONTROLES
  if ((flag_HK == 0)){
    if ((flag_PLD == 0)){ //usado para testes
      Serial.print("Dados HK: "); Serial.println(dados_HK);
      flag_HK = 1;
      espSerialTTeC.println(dados_HK);
    }
    if ((flag_PLD == 1)){
      if ((flag_timing == 0)){ //->Ações que acontecem a cada 0.5 segundos de forma alternada
        if (espSerialPayload.available() > 0){
          PLD = espSerialPayload.readStringUntil('\n');//Serial2.readStringUntil('\n');
          PLD_string = "P;" + PLD;
          Serial.println(PLD_string);
          espSerialTTeC.print(PLD_string);
        }
        Serial.println("Teste cena 1 iniciando");
        Serial.println("Teste cena 1 ok");
      }
      if ((flag_timing == 1)) {
        data_record.println(dados_tensao);//Serial2.println(dados_tensao);
        espSerialTTeC.println(dados_tensao);
      }
      if ((flag_timing == 2)) {
        data_record.println(dados_corrente);//Serial2.println(dados_corrente);
        espSerialTTeC.println(dados_corrente);
      }
      if ((flag_timing == 3)) {
        data_record.println(dados_TCS);//Serial2.println(dados_TCS);
        espSerialTTeC.println(dados_TCS);
      }
      if ((flag_timing == 4)) {
        ADCS_string = "I;" + ADCS;
        data_record.println(ADCS_string);//Serial2.println(ADCS);
        espSerialTTeC.println(ADCS_string);
      }
      flag_HK = 1;
      flag_timing += 1; //->Ações que acontecem a cada 0.5 segundos
    }
  }
  if ((flag_timing >= 5)){
    flag_timing = 0;
  }
  if ((flag_ADCS == 0)){
    espSerialADCS.println(solar_Current1 + ";" + solar_Current2 + ";" + solar_Current3 + ";" + solar_Current4);
    flag_ADCS = 1;
    Serial.println("Corrente enviada");
  }
  if ((tempo_atual - tempo_passado) >= 500){
    flag_HK = 0;
    tempo_passado = millis();
  }
  if ((tempo_atual - tempo_ADCS) >= 300){
    flag_ADCS = 0;
    tempo_ADCS = millis();
  }
  Serial.println("----------------------");
  //ADCS = "";
  mensagem_old = mensagem_new;
  data_record.close();
}
