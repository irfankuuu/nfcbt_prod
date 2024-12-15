#include <Wire.h>
#include <Adafruit_PN532.h>
#include <BluetoothSerial.h>

//---- Pin untuk SPI---------

#define PN532_SCK (18)
#define PN532_MISO (19)
#define PN532_MOSI (23)
#define PN532_SS (5)

//---------------------------

//---inisiasi NFC dan Bluetooth---

Adafruit_PN532 nfc(PN532_SS);
BluetoothSerial SerialBT;

//--------------------------------

//---- Variable untuk state awal bluetooth dan interval pembacaan NFC -----------------

bool bluetoothConnected = false;
unsigned long lastCheck = 0;
const unsigned long checkInterval = 1000;   // Interval untuk memeriksa koneksi Bluetooth
const unsigned long nfcReadTimeout = 1000;  // Timeout untuk pembacaan NFC

//-------------------------------------------------------------------------------------

//--------------------- pin setup untuk baterai, LED & Buzzer ------------------

#define ledBT 4         //pin LED
#define ledBat 25       //pin notif batterai
#define buzzer 27       //pin buzzer
#define BATTERY_PIN 32  // pin untuk pembacaan baterai dari out pembagi tegangan

//------------------------------------------------------------------------------

//-------- Setting Untuk nilai-nilai, Nama Bluetooth yang dibutuhkan -------------------- (r10k)// 

const int ledValue = 25;             // nilai output untuk kecerahan led
const float maxVoltage = 3.55;        // Tegangan maksimal
const float minVoltage = 3.2;        // Tegangan minimal
const float referenceVoltage = 3.3;  // Tegangan referensi (3.6)
const int adcMaxValue = 3099;        // Nilai maksimum ADC (12-bit ADC)  (3474.65)                                                                  //cr by Mitsal Ghapiqi
const float ralatBaterai = 0.04;  
const int numSamples = 1000;          // Jumlah sampel untuk rata-rata (stabilizator nilai presentase baterai)(500)
const int minPercentage = 35;        // presentase minimal untuk notif low batterry (10)
const int maxPercentage = 92;        // presentase maksimal untuk notif battery ready (80)
String namaBluetooth = "NBT_TEST";     // Nama Bluetooth

//--------------------------------------------------------------------------------------//
String nfcID = "";  //variable penyimpanan ID NFC

void bateraiIndikator() {
  long adcSum = 0;
  for (int i = 0; i < numSamples; i++) {
    adcSum += analogRead(BATTERY_PIN);
    delay(1);  // Delay pendek untuk memberikan waktu antara pembacaan sampel
  }

  //--------------- Logic Rumus presentase batterai----------------------------

  int adcValue = adcSum / numSamples;
  float voltage = (adcValue * referenceVoltage) / adcMaxValue;
  if (voltage > maxVoltage) voltage = voltage;
  if (voltage < minVoltage) voltage = voltage;
  int percentage = ((voltage - minVoltage) / (maxVoltage - minVoltage)) * 100; //(100)

  //-----------------------------------------------------------------------------

  //------------notification battery logic----------------

  if (percentage <= minPercentage) {
    analogWrite(ledBat, ledValue);
    digitalWrite(buzzer, HIGH);
    delay(80);
    analogWrite(ledBat, 0);
    digitalWrite(buzzer, LOW);
    delay(40);
    
      
  } else if (percentage >= maxPercentage) {
    analogWrite(ledBat, 0);
  }

  //------------------------------------------------------

  //--------------Print Nilai Input Batterai ke Serial Monitor----------------
  Serial.print("ADC : ");
  Serial.print(adcValue);
  Serial.print(" ");
  Serial.print("Voltage: ");
  Serial.print(voltage);
  Serial.print(" V, ");
  Serial.print("Battery Percentage: ");
  Serial.print(percentage);
  Serial.println(" %");

  //------- Uncommand If needed to send to bluetooth ------------------------ //
  //SerialBT.println(voltage);
  //SerialBT.println(percentage);

  //--------------------------------------------------------------------------
}

void setup(void) {

  Serial.begin(115200);           //Baud Rate Serial Monitor
  SerialBT.begin(namaBluetooth);  // Nama Bluetooth
  Serial.println("NFC reader initializing...");
  pinMode(ledBT, OUTPUT);
  pinMode(buzzer, OUTPUT);
  pinMode(BATTERY_PIN, INPUT);
  pinMode(ledBat, OUTPUT);
  analogReadResolution(12);  // ADC diubah jadi 12-bit
  nfc.begin();               // NFC inisiator

  uint32_t versiondata = nfc.getFirmwareVersion();
  //------------ ketika PN532 Error Indikasi tidak Koneksi PN532--------------------
  if (!versiondata) {
    Serial.print("Didn't find PN53x board");
    while (1) {
      analogWrite(ledBT, ledValue);
      analogWrite(ledBat, ledValue);
      digitalWrite(buzzer, HIGH);
      delay(100);
      digitalWrite(buzzer, LOW);
      analogWrite(ledBT, 0);
      analogWrite(ledBat, 0);
      delay(100);
    };  // halt
    //------------------------------------------------------------------------------
  }

  
  nfc.SAMConfig();
  Serial.println("System Ready Waiting for an NFC card ...");
  for (int x = 1; x <= 3; x++) {  // notification logic System Ready
    analogWrite(ledBT, ledValue);
    analogWrite(ledBat, ledValue);
    digitalWrite(buzzer, HIGH);
    delay(100);
    digitalWrite(buzzer, LOW);
    analogWrite(ledBT, 0);
    analogWrite(ledBat, 0);
    delay(100);
  }
}
 
void loop(void) {

  unsigned long currentMillis = millis();

  // Memeriksa koneksi Bluetooth setiap interval tertentu
  if (currentMillis - lastCheck >= checkInterval) {
    lastCheck = currentMillis;
    bateraiIndikator();
    bluetoothConnected = SerialBT.hasClient();
  }

  if (bluetoothConnected) {  // logic untuk bluetooth terkoneksi
    uint8_t uid[7];          // Maximum size of the UID
    uint8_t uidLength;

    // Memulai pendeteksian non-blocking
    nfc.startPassiveTargetIDDetection(PN532_MIFARE_ISO14443A);

    unsigned long startTime = millis();
    bool cardDetected = false;

    // Coba membaca kartu NFC dengan timeout
    while (millis() - startTime < nfcReadTimeout) {
      // Coba baca apakah ada kartu yang terdeteksi
      if (nfc.readDetectedPassiveTargetID(uid, &uidLength)) {
        cardDetected = true;
        break;
      }

      // Periksa kembali koneksi Bluetooth selama menunggu
      if (!SerialBT.hasClient()) {
        bluetoothConnected = false;
        break;
      }

      delay(10);  // Tambahkan sedikit delay untuk mengurangi beban CPU
    }

    if (cardDetected) {
      Serial.print("NFC ID Terdeteksi UID : ");
      String CardID = "";
      String Link = "";
      unsigned int hex_num;
      hex_num = uid[3] << 24;
      hex_num += uid[2] << 16;
      hex_num += uid[1] << 8;
      hex_num += uid[0];
      CardID = String(hex_num);
      if (CardID.length() == 8) {
        CardID = '0' + CardID;
      }
      if (CardID.length() == 9)  //panjang karakter untuk nfc id
      {
        CardID = '0' + CardID;
      } else {
        nfcID = CardID;
      }
      nfcID = CardID;
      delay(50);

      digitalWrite(buzzer, HIGH);
      delay(300);
      digitalWrite(buzzer, LOW);
      delay(300);

      digitalWrite(ledBT, LOW);
      delay(100);
      Serial.println(nfcID);
      SerialBT.print(nfcID + String("\n"));  // kirim NFC ID to bluetooth
      analogWrite(ledBT, ledValue);
    }
  } else {  // Logic Bluetooth terputus
    Serial.println("Bluetooth disconnected, waiting for connection...");
    for (int x = 1; x <= 4; x++) {
      digitalWrite(buzzer, HIGH);
      delay(200);
      digitalWrite(buzzer, LOW);
      delay(200);
    }

    while (!SerialBT.hasClient()) {  //perulangan atau logic untuk menunggu koneksi bluetooth
      delay(500);                    // Menunggu hingga ada koneksi Bluetooth
      analogWrite(ledBT, ledValue);
      delay(100);
      analogWrite(ledBT, 0);
      delay(100);
      bateraiIndikator();
      delay(1000);
    }
    Serial.println("Bluetooth connected, resuming NFC reading...");
    //--------- Control Notification ketika tap card -------------//
    analogWrite(ledBT, ledValue);
    for (int x = 1; x <= 3; x++) {
      digitalWrite(buzzer, HIGH);
      delay(200);
      digitalWrite(buzzer, LOW);
      delay(200);
    }
  }
  delay(500);  // Delay untuk menghindari loop terlalu cepat
}

// Cr by Mitsal Ghapiqi
// dev by Irfan_ku