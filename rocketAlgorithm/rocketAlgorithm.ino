#include <SoftwareSerial.h> 
#include <TinyGPS++.h> //GPS KÜTÜPHANESİ
#include "LoRa_E32.h" //LORA KÜTÜPHANESİ
#include <Adafruit_BME280.h> //BME280 KÜTÜPHANESİ
#include <Adafruit_BNO055.h> //BNO055 KÜTÜPHANESİ
#include <SimpleKalmanFilter.h> //KALMAN KÜTÜPHANESİ

//PİN TANIMLAMALARI
#define loraRx 12
#define loraTx 11
#define loraM0 6
#define loraM1 7
#define gpsRx 10
#define gpsTx 9
#define buzzer 8

//ATESLEME PINLERI
#define ateslemePinBurun 3
#define ateslemePinPayload 4
#define ateslemePinAna 5


//BAĞLANTILAR
SoftwareSerial SoftwareSerialLora(loraTx, loraRx); //LORA
SoftwareSerial SoftwareSerialGps(gpsTx, gpsRx); //GPS

//TANIMLAMALAR

LoRa_E32 lora(&SoftwareSerialLora); //LORA TANIMLAMA
TinyGPSPlus gps; //GPS TANIMLAMA
Adafruit_BME280 bmp; //BME TANIMLAMA
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x29); //BNO TANIMLAMA
SimpleKalmanFilter kalmanFiltresi = SimpleKalmanFilter(1, 1, 0.1);  //KALMAN TANIMLAMA


//GELEN VERİLER İÇİN TANIMLANAN DEGİSKENLER
float alt = 0; //yükseklik verisi
float Gx=0, Gy=0, Gz=0; // eğim verileri


//ATEŞLEME İÇİN TANIMLANAN DEGISKENLER
float dusmeHakki = 0;
float maxYukseklik = 0;
float tepeNoktasiAtesleme = 0;
float payloadAtesleme = 0;
float anaParasutAtesleme = 0;
float roketDustu = 0;


//MESAJLAŞMA İÇİN TANIMLANAN DEGISKENLER
byte ucusMesaj = 0;
byte bnoMesaj = 0;
byte bmpMesaj = 0;
byte gpsMesaj = 0; 
byte loraMesaj = 0; 
 
struct MessageMain //LORA İLE GÖNDERECEĞİMİZ MESAJ CLASS I
{
    char type[5] = "SRA";
    char message[8]= "TEST";
    byte ucusMesaj = 0;
    byte bnoMesaj = 0;
    byte bmpMesaj = 0;
    byte gpsMesaj = 0; 
    byte loraMesaj = 0; 
    float enlem=0, boylam=0, irtifa=0, temp=0, hiz=0;
    float Gx=0, Gy=0, Gz=0;
    byte uydu=0;
} messageMain;

void setup() 
{
  Serial.begin(9600); //BASILAN DEĞERLERİ GÖRECEĞİMİZ PORT.
  
  pinMode(ateslemePinBurun,OUTPUT); //TEPE NOKTASI ATEŞLEME PİNİ TANIMI, BURUN -> APOGEE
  pinMode(ateslemePinPayload,OUTPUT); //TEPE NOKTASINDAN SONRA İKİNCİ GÜVENLİK ATEŞEME PİNİ TANIMI, PAYLOAD KISMI BIRAKMA -> 2000 m
  pinMode(ateslemePinAna,OUTPUT); //PAYLOAD ATEŞLEME PİNİ TANIMI, ANA PARASUT -> 600 m ASSA INERKEN
  
  digitalWrite(ateslemePinBurun, LOW);
  digitalWrite(ateslemePinPayload, LOW);
  digitalWrite(ateslemePinAna, LOW);

  
  buzzer_Songs(1); //SETUP GİRİŞ SESİ
  

  //GPS BAŞLATILMASI  
   SoftwareSerialGps.begin(9600);
   gpsMesaj = 1; // 1 : gps başlatıldı.
   delay(500);    

  //BNO BAŞLATILMASI                 
  if(bno.begin())
  {
    bnoMesaj = 1; // 1 : bno başlatıldı. 
  }
  else
  {
    bnoMesaj = 0; // 0 : bno başlatıldı.
  }
  delay(500);
  
  //LORA BAŞLATILMASI
  if(lora.begin())
  {
    loraMesaj = 1; // 1 : lora başlatıldı
  }
  else
  {
    loraMesaj = 0; // 0 : lora başlatılamadı.
  }
  delay(500);
  pinMode(loraM0,OUTPUT);
  pinMode(loraM1,OUTPUT);
  pinMode(13,OUTPUT);
  pinMode(buzzer, OUTPUT);
  digitalWrite(loraM0, LOW);
  digitalWrite(loraM1, LOW);
  digitalWrite(13, LOW);
  
  buzzer_Songs(0); //SETUP ÇIKIŞ SESİ
}

void loop() 
{
  buzzer_Songs(3); //SENSÖR ÇALIŞMA GİRİŞ SESİ
  GPS();  //GPS GELEN VERİ
  BMP_BILGI(); //BMP GELEN VERİ
  BNO(); //BNO GELEN VERİ 
  buzzer_Songs(4); //SENSÖR ÇALIŞMA ÇIKIŞ SESİ
 
  
  
  BURUN_ALGORITMA(); //TEPE NOKTASI ATEŞLEME ALGORİTMASI
  PAYLOAD_ALGORITMA(); //PAYLOAD EMİN OLMAK İÇİN ATEŞLEME ALGORİTMASI (NOT: NORMALDE PAYLOOAD BURUN KONİSİNDEKİ İPE BAĞLI OLDUĞU İÇİN BURUN ALGORİTMASINDA ÇIKACAKTIR)
  ANA_PARASUT(); //ANA PARAŞÜTÜN ATEŞLEME ALGORİTMASI
  
  
  (messageMain.ucusMesaj) = ucusMesaj;
  
  delay(500); //LORA İÇİN 0.5 SN BEKLETİYORUZ
  LORA(); //LORA İLE DİĞER LORAYA VERİ GÖNDERME
}

void BNO() //BNO KODLARI
{
  sensors_event_t event;
  bno.getEvent(&event);
  Gx = event.orientation.x;
  Gy = event.orientation.y;
  Gz = event.orientation.z;
  (messageMain.Gx) = Gx;
  (messageMain.Gy) = Gy;
  (messageMain.Gz) = Gz;

  delay(100);
}

float BmpYukseklikHesapla(float P0, float P, float T) //BMP KODLARI
{
  float altitude = ((pow((P0/P),(1/5.257)) - 1.0) * (T+273.15)) / 0.0065;
  float kalmanliAltitude =  kalmanFiltresi.updateEstimate(altitude);
  return kalmanliAltitude;
}

void BMP_BILGI() //BMP KODLARI
{
  float P0_Kutahya = 1008.2; //KUTAHYA DENIZ SEVIYESINDEN ATMOSFER BASINCI https://www.mgm.gov.tr/tahmin/il-ve-ilceler.aspx?il=K%C3%BCtahya
  float temp = bmp.readTemperature(); //Okunan sıcaklık
  float pressure = bmp.readPressure(); //Okunan basınc Pa
  alt = BmpYukseklikHesapla(P0_Kutahya, pressure/100.0, temp);
  (messageMain.irtifa) = alt;
} 

void GPS()
{
  float flat=0, flon=0, hiz=0;
  byte sat=0;
  bool newData = false;

  SoftwareSerialGps.listen();
  for (unsigned long start = millis(); millis() - start < 1000;)
  {
    while (SoftwareSerialGps.available() > 0)
    { 
      buzzer_Songs(5); //GPS ÇALIŞMADI SESİ
      if (gps.encode(SoftwareSerialGps.read())) 
        newData = true;      
    }
  }
  
  if (newData)
  {
    buzzer_Songs(6); //GPS ÇALIŞTI SESİ
    
    if(gps.location.isValid())
    {
      flat = gps.location.lat();
      flon = gps.location.lng();
    }else
    {
      gpsMesaj = 2;  //2 : lokasyan çalışmıyor!
    }

    if(gps.satellites.isValid())
    {
      sat = gps.satellites.value();
    }else
    {
      gpsMesaj = 3; //3 : uyduya bağlanmıyor!
    }
    
    if(gps.speed.isValid())
    {
      hiz = gps.speed.mps();
    }else
    {
      gpsMesaj = 4;  //4 : hız gelmiyor!
    }
       
  }

  else
  {
    gpsMesaj = 5;  // 5 : data gelmiyor!
  }
  
  (messageMain.enlem) = flat;
  (messageMain.boylam) = flon;
  (messageMain.uydu) = sat;
  (messageMain.hiz) = hiz;

  smartDelay(100);//GPS DELAY'I
}

void LORA()
{
  SoftwareSerialLora.listen();
  ResponseStatus rs = lora.sendFixedMessage(0,1,23,&messageMain, sizeof(MessageMain)); 
}

void buzzer_Songs(byte value) //BUZZER KODLARI
{
  switch(value)
  {
    case 1:
    tone(buzzer,100,500);
    tone(buzzer,850,500);
    tone(buzzer,1600,500);
    break;
    case 3 :
    tone(buzzer,100,250);
    tone(buzzer,850,500);
    tone(buzzer,769,300);
    break;
    case 4 :
    tone(buzzer,100,100);
    tone(buzzer,280,500);
    tone(buzzer,380,300);
    break;
    case 5 :
    tone(buzzer,20,100);
    tone(buzzer,280,175);
    tone(buzzer,700,300);
    break;
    case 6 :
    tone(buzzer,280,100);
    tone(buzzer,280,320);
    tone(buzzer,380,265);
    break;
    default:
    tone(buzzer,1600,500);
    tone(buzzer,850,500);
    tone(buzzer,100,500);
    break;
  }
}

static void smartDelay(unsigned long ms)
{
  unsigned long start = millis();
  do 
  {
    while (SoftwareSerialGps.available())
      gps.encode(SoftwareSerialGps.read());

  } while (millis() - start < ms);
}

void BURUN_ALGORITMA()
{
    if(tepeNoktasiAtesleme == 0)
    {
      if(alt >= 3000 && Gx == 0) //yükseklik yarışma alanının deniz seviyesinden yüksekliği ile hesaplanacaktır.Bu yükseklik denemedir.
      {
        digitalWrite(3,HIGH);
        ucusMesaj = 1 ;   // 1 : TEPE NOKTASI 'ANA' ALGORİTMA BURUN KONİSİ ATEŞLENDİ ,TEBRİKLER
        tepeNoktasiAtesleme = 1;              
      }
      
      if(alt > maxYukseklik)
      {
        maxYukseklik = alt;
        ucusMesaj = 2;  // 2 : ROKET YÜKSELMEYE DEVAM EDİYOR

      }
    
      else if(alt < maxYukseklik)                                //not : mesajları yer istasyonuna göndereceğiz.
      {
        dusmeHakki += 1;
        if(dusmeHakki >= 2)
        {
          digitalWrite(3,HIGH);
          ucusMesaj = 4; // 4 : TEPE NOKTASI 'GÜVENLİK' ALGORİTMA BURUN KONİSİ ATEŞLENDİ ,TEBRİKLER
          tepeNoktasiAtesleme = 1;
        }  
      }
    }
    
    if(tepeNoktasiAtesleme == 1)
    {
      ucusMesaj = 5; // 5 : TEPE NOKTASI BURUN KONİSİ ALGORİTMASI KAPATILDI
    }
    if(alt < maxYukseklik && roketDustu == 0)
    {
      ucusMesaj = 10; // 10 : Roket Alçalıyor.
    }
    if(alt <= 0)
    {
      for(int i = 0; i < 4; i++)
      {
        ucusMesaj = 11; // 11 : Roket Düştü.
      }
        
    }  
    maxYukseklik = alt;
    
}
void PAYLOAD_ALGORITMA()
{
  if(tepeNoktasiAtesleme == 1)
  {
    if(alt <= 2000 && payloadAtesleme == 0)   //yükseklik yarışma alanının deniz seviyesinden yüksekliği ile hesaplanacaktır.Bu yükseklik denemedir.
    {
      digitalWrite(4,HIGH);
      ucusMesaj = 6;  // 6 : "PAYLOAD 'ANA' ALGORİTMA ATEŞLENDİ ,TEBRİKLER"
      payloadAtesleme = 1;
    }
    else if(payloadAtesleme == 1)
    {
      ucusMesaj = 12; // 12 : PAYLOAD ALGORİTMASI KAPATILDI
    }
  }
     
}

void ANA_PARASUT()
{
  if(anaParasutAtesleme == 0)
  {
      if(alt <= 600 && tepeNoktasiAtesleme == 1 && payloadAtesleme == 1)   //yükseklik yarışma alanının deniz seviyesinden yüksekliği ile hesaplanacaktır.Bu yükseklik denemedir.
    {
      digitalWrite(5,HIGH);
      ucusMesaj = 8;  // 8 : "ANA PARAŞÜT 'ANA' ALGORİTMA ATEŞLENDİ ,TEBRİKLER"
      anaParasutAtesleme = 1;
    }
  }
  else if(anaParasutAtesleme == 1)
  {
    ucusMesaj = 13; // 13 : ANA PARASUT ALGORİTMASI KAPATILDI.
  }
  
}


//YAPILACAKLAR:
//YEDEK SİSTEME GEÇİŞ //YEDEK SİSTEME GEÇ MESAJI GÖNDERİRİZ YEDEK SİSTEME. ÖRNEĞİN YEDEK SİSTEMDE BİR yedekGecis = ""; boş bir değer olacak oraya gelecek ve sonra if li bir yapıda yedekGecis = "YEDEK SİSTEM GEÇ" ise loop u başlat gibisinden.
//PAYLOAD VE ANA PARASUT ALGORİTMALARINA GÜVENLİK ALGORİTMASI DEVAM EDİLECEK.
