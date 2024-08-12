//Lora Parameter
#include <lorawan.h>
const sRFM_pins RFM_pins = { //pin yang dipakai untuk modul lora RFM95
  .CS = 13, // chip select
  .RST = 5, // chip reset
  .DIO0 = 27, //data in out 0
  .DIO1 = 26, //data in out 1
};
//parameter pada console telkomIoT
String devAddr = "6631ef72"; //parameter device address
String appSKey = "6631ef72cfab3605a4b8640d0410d0f6"; //parameter application session key
String nwkSKey = "6631ef72a04bf35a6f7f59b35a3c28f0"; //parameter network session key

//Ultrasonic Parameter
byte trigPin = 15; //pin trigger
byte echoPin = 4;  //pin echo
float y1Cal = 45.0; //jarak dasar kolam ke permukaan air
float y2Cal = 74.0; //jarak sensor ke permukaan air

//DallasTemp Parameter
#include <OneWire.h> //library komunikasi onewire
#include <DallasTemperature.h> //library dallas temp
#define dallasPin 32  //pin yang terhubung dengan sensor dallas
OneWire oneWire(dallasPin);
DallasTemperature dallas(&oneWire);

//ADS Parameter
#include <Adafruit_ADS1X15.h>  //library untuk ads1115
Adafruit_ADS1115 ads; 

//MAF 
#include "MovingAverage.h" //library untuk moving average filter, menyetabilkan pembacaan sensor
MovingAverage mafWL(4); //filter moving average water level
MovingAverage mafTemp(4); //filter moving average temperature
MovingAverage mafTds(4);  //filter moving average TDS

uint8_t uplinkInterval = 10; //interval dalam menit, jeda pengiriman jika menggunakan mode tidur

//variabel pengukuran
float tempValue;  //nilai temperature air
float tdsValue; //nilai tds air
float waterLevelValue; //ketinggian air
float yValue; //jarak pengukuran ultrasonik


void setup() {
  Serial.begin(115200); //inisiasi serial
  loraSetup(); //inisiasi lora
  sensorSetup(); //inisiasi sensor ultrasonic dan dallas temperature
  waterLevelValue = getWaterLevel(); //ambil data water level
  tempValue = getTemp(); //ambil data suhu
  tdsValue = getTds(tempValue); //ambil data TDS
  Serial.println("WL\t\tTmp\t\tTds");
  Serial.println(String(waterLevelValue)+"\t\t"+String(tempValue)+"\t\t"+String(tdsValue)); //menampilkan pengukuran ke serial monitor
  String data = "{\"WL\":" + String(waterLevelValue, 2) + ",\"T\":" + String(tempValue, 2) + ",\"TDS\":" + String(tdsValue, 2) + "}"; //mempersiapkan data yang akan dikirim
  sendData_Lora(data); // mengirimkan data melalui LoRaWAN
  // goToSleep();  //memanggil fungsi deepsleep
}

void loop() {
  waterLevelValue = getWaterLevel(); //ambil data water level
  tempValue = getTemp(); //ambil data suhu
  tdsValue = getTds(tempValue); //ambil data TDS
  Serial.println("WL\t\tTmp\t\tTds");
  Serial.println(String(waterLevelValue)+"\t\t"+String(tempValue)+"\t\t"+String(tdsValue)); //menampilkan pengukuran ke serial monitor
  String data = "{\"WL\":" + String(waterLevelValue, 2) + ",\"T\":" + String(tempValue, 2) + ",\"TDS\":" + String(tdsValue, 2) + "}"; //mempersiapkan data yang akan dikirim
  sendData_Lora(data); // mengirimkan data melalui LoRaWAN
  delay(3000);
}

void goToSleep(){ //fungsi mode sleep
  uint64_t sleepTime = uplinkInterval * 60;
  esp_sleep_enable_timer_wakeup(sleepTime * 1000000ULL);
  Serial.println("Going to sleep now");
  Serial.flush();  //clear serial buffer
  esp_deep_sleep_start();
}

void sensorSetup() {
  //Ultrasonic setup
  pinMode(trigPin, OUTPUT); //pin trigger (kirim sinyal)
  pinMode(echoPin, INPUT); //pin echo (terima sinyal)

  //DallasTemp setup
  dallas.begin();

  //ADS setup (TDS)
  if (!ads.begin(0x48)) { //inisialisasi ads
    Serial.println("Problem : ADS");
    delay(1000);
    if (!ads.begin()) {  //failover
      ESP.restart();
    }
    Serial.println("Status  : ADS Oke");
    return;
  }
}

float getWaterLevel() { //fungsi untuk mengukur tinggi air ke permukaan / water level
  float duration;
  float distance;
  float waterLevel;
  for (int i = 0; i < 4; i++) {
    digitalWrite(trigPin, LOW);
    delayMicroseconds(5);
    digitalWrite(trigPin, HIGH); //mulai mengirim gelombang
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);
    duration = pulseIn(echoPin, HIGH); //menangkap gelombang
    distance = mafWL.addSample(duration * 0.034 / 2); //melakukan konversi ke jarak, cepat rambat suara di udara 340m/s = 0.034cm/µs
    delay(100);
  }
  Serial.println("Distance " + String(distance) + " cm");
  waterLevel = (y1Cal + y2Cal) - distance; //menghitung ketinggian air

  return waterLevel;
}

float getTemp() {  //fungsi untuk mengukur suhu
  float temp;
  dallas.setResolution(10); //mengatur resolusi pembacaan
  for (int i = 0; i < 4; i++) {
    dallas.requestTemperatures();
    temp = mafTemp.addSample(dallas.getTempCByIndex(0));  //mengambil nilai rerata tegangan
    delay(50);
  }
  return temp;
}

float getTds(float temperature) { //fungsi untuk mengukur nilai TDS
  int adc;
  float totalVoltage = 0; // Variabel untuk menyimpan total tegangan

  for (int i = 0; i < 10; i++) {
    adc = ads.readADC_SingleEnded(0); // Baca nilai ADC dari saluran 0
    totalVoltage += ads.computeVolts(adc); // Tambahkan nilai tegangan ke totalVoltage
    delay(100); // Tunggu sebentar sebelum membaca data lagi
  }
  float averageVoltage = totalVoltage / 10.0; // Hitung rata-rata tegangan
  float compensationCoefficient=1.0+0.02*(temperature-25.0);    //temperature compensation formula: fFinalResult(25^C) = fFinalResult(current)/(1.0+0.02*(fTP-25.0));
  float compensationVoltage=averageVoltage/compensationCoefficient;  //temperature compensation
  float rawTdsValue=(133.42*compensationVoltage*compensationVoltage*compensationVoltage - 255.86*compensationVoltage*compensationVoltage + 857.39*compensationVoltage)*0.5; //convert voltage value to tds value
  float finalTdsValue = 0.0023*(rawTdsValue*rawTdsValue) - (0.1103*rawTdsValue) + 90.722; // rumus regresi polynomial y = 0,0023x2 - 0,1103x + 90,722, dimana Y adalah finalTds dan X adalah rawTds
  if (rawTdsValue < 5) finalTdsValue = 0.0;
  return finalTdsValue;
}

void loraSetup() { //inisiasi lora
  if (!lora.init()) {
    Serial.println("Problem : Lora");
    delay(2000);
    return;
  }
  lora.setDeviceClass(CLASS_A);  // Set LoRaWAN Class change CLASS_A 
  lora.setDataRate(SF10BW125); //datarate menentukan max jarak dan max data
  lora.setFramePortTx(5);
  lora.setChannel(MULTI); // channel pengiriman lora
  lora.setTxPower(20); //power transmisi lora
  lora.setNwkSKey(nwkSKey.c_str()); // parameter di platform
  lora.setAppSKey(appSKey.c_str()); // parameter di platform
  lora.setDevAddr(devAddr.c_str()); // parameter di platform
}

void sendData_Lora(String msg) { //fungsi untuk kirim data ke platform via LoRaWAN
  char myStr[msg.length() + 1]; //membuat variabel char sesuai panjang data yang akan dikirim
  msg.toCharArray(myStr, msg.length() + 1); //mengkonversi data String ke char sebelum dikirim
  lora.sendUplink(myStr, strlen(myStr), 0); //mengirimkan data
  lora.update(); //melakukan update atau cek downlink
  Serial.println(myStr);  //print data ke serial monitor untuk memastikan data yang dikirim sesuai
  Serial.println(" ");
  delay(3000); //delay 3 detik
}
