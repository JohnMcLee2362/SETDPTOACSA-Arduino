#include <SoftwareSerial.h>

//Libreria del GPS
#include <TinyGPSPlus.h>
//Librerias de sensores de altitud y movimiento
#include <Adafruit_BMP280.h>
#include <MPU9250_asukiaaa.h> 
#include <Servo.h>

Servo myservo; 

Adafruit_BMP280 bmp;
MPU9250_asukiaaa mySensor;
// The TinyGPSPlus object
TinyGPSPlus gps;

//Libreria de la sd
#include <SPI.h>
#include <SD.h>

#define buzzer 4
#define led 5

static const int Gps_RXPin = 3, Gps_TXPin = 2;
static const uint32_t GPSBaud = 9600;


int pos_Servo = 0;    // variable to store the servo position
SoftwareSerial gps_ss(Gps_RXPin,Gps_TXPin);//rx tx 
float Temp, Pres, Alt, aX, aY, aZ, aSqrt, gX, gY, gZ, mX, mY, mZ, mDirection,Latitud,Longitud,hora,minuto,segundo;
float datos_AltMov[14];
float datos_gps[14];
uint8_t sensorId;
int result;


void setup() {

  pinMode(buzzer, OUTPUT);
  pinMode(led, OUTPUT);

  myservo.attach(6);  // attaches the servo on pin A3 to the servo object

  Serial.begin(9600);
  gps_ss.begin(9600); 
  //Comprobamos sensor de altitud movimiento:
  mySensor.beginAccel();
  mySensor.beginGyro();
  mySensor.beginMag();
  // You can set your own offset for mag values
  // mySensor.magXOffset = -50;
  // mySensor.magYOffset = -55;
  // mySensor.magZOffset = -10;
  bmp.begin(BMP280_ADDRESS_ALT, BMP280_CHIPID);
  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
                  Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
                  Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
                  Adafruit_BMP280::FILTER_X16,      /* Filtering. */
                  Adafruit_BMP280::STANDBY_MS_500); /* Standby time. */

  //Comprobamos la tarjeta SD
  Serial.print("Initializing SD card...");
  if (!SD.begin(10)) {
    Serial.println("Card failed, or not present");
    // don't do anything more:  }
  }
    else{
  Serial.println("card initialized.");
  }
}



void loop() {
  //Guardamos los datos en dos arrays
  altitudmovimiento();
  gpsFuncion();


//Imprimimos los datos de cada cosa
    Serial.write("GY-91:");  

  Serial.println();

  for(int i = 0; i < 15; i++){
    String envio = String(datos_AltMov[i]);
    for (int j = 0; j < 10; j++){
    Serial.write(envio[j]);
    }
    Serial.write(",");
    Serial.println();
    delay(100);
  }  
  Serial.print("Gps:");
  Serial.println();
  for(int i = 0; i < 6; i++){
    String envio = String(datos_gps[i]);
    for (int j = 0; j < 10; j++){
    Serial.write(envio[j]);
    }
    Serial.println();
    delay(100);
  }  

// //Ahora la altitud en cdmx es de 2,240, en teoria el cansat deberia caer desde 2,640 y abrir el autogiro en 2,390
//   if(datos_AltMov[3] == 250){
//     //Desplegamos el autogiro
//       for (pos_Servo = 0; pos_Servo <= 180; pos_Servo += 1) { // goes from 0 degrees to 180 degrees
//       // in steps of 1 degree
//       myservo.write(pos_Servo);              // tell servo to go to position in variable 'pos'
//       delay(15);                       // waits 15 ms for the servo to reach the position
//     }
//   }

//   if(datos_AltMov[3] ==2240){
//     //alarma();
//     Serial.println("Ya llegamossssss");
//   }

}


//La funcion para encontrar el cansat una vez que caiga
void alarma(){
  //if(ya aterrizamos ==True) 
    for(int i = 0; i < 10; i++){
    tone(buzzer, 440);
    digitalWrite(led, HIGH);  // turn the LED on (HIGH is the voltage level)
    delay(55);
    noTone(buzzer);
    digitalWrite(LED_BUILTIN, LOW);   // turn the LED off by making the voltage LOW
    delay(55);
  }
}

//La funcion guardadito toma un arreglo y lo guarda en la ssd
//La funcion guardadito toma un arreglo y lo guarda en la ssd
void guardadito(float* arreglo){
  //int tamano = sizeof(arreglo[0])+1;
  int tamano = 15;
    // make a string for assembling the data to log:
  String dataString = "";

  for (int i = 0; i < tamano; i++) {
    dataString += String(arreglo[i]); // Agregar cada elemento del arreglo al String
    if (i < tamano - 1) {
      dataString += ", "; // Agregar una coma y un espacio después de cada elemento, excepto el último
    }
  }

  // open the file. note that only one file can be open at a time,
  // so you have to close this one before opening another.
  File dataFile = SD.open("datalog.txt", FILE_WRITE);

  // if the file is available, write to it:
  if (dataFile) {
    Serial.println("Guardando Datos en sd AAAAAAAAAAAAAAAAAAAAAAAAA");
    dataFile.println(dataString);
    dataFile.close();
    // print to the serial port too:
    Serial.println(dataString);
    Serial.println("guardadito complited AAAAAAAAAAAAAAAAAAAAAAAAA");

    delay(1000);

  }
  // if the file isn't open, pop up an error:
  else {
    Serial.println("error opening datalog.txt");
  }
}

//La funcion para leer los datos del gps, queremos que entregue latitud, longitud, y tiempo
// la posicion este gps
void gpsFuncion(){
  Serial.print(F("Location: ")); 
  if (gps.location.isValid())
  {
    Latitud = gps.location.lat();
    Longitud = gps.location.lng();
    Serial.print(gps.location.lat(), 6);
    Serial.print(F(","));
    Serial.print(gps.location.lng(), 6);
  }
  else
  {
    Latitud = 19.327450;
    Longitud =-99.178960;
    Serial.print(F("INVALID"));
  }

  Serial.print(F("  Time: "));

  Serial.print(F(" "));
  if (gps.time.isValid())
  {
    if (gps.time.hour() < 10) Serial.print(F("0"));
    hora = gps.time.hour();
    minuto=gps.time.minute();
    segundo=gps.time.second();
    Serial.print(gps.time.hour());
    Serial.print(F(":"));
    if (gps.time.minute() < 10) Serial.print(F("0"));
    Serial.print(gps.time.minute());
    Serial.print(F(":"));
    if (gps.time.second() < 10) Serial.print(F("0"));
    Serial.print(gps.time.second());
    Serial.print(F("."));
    if (gps.time.centisecond() < 10) Serial.print(F("0"));
    Serial.print(gps.time.centisecond());
  }
  else
  {
    Serial.print(F("INVALID"));
    hora = 0.0;
    minuto=0.0;
    segundo=0.0;
  }
  //Serial.println(gps.speed.mps()); // Speed in meters per second (double)
  Serial.println();
    datos_gps[1] = Latitud;
    datos_gps[2] = Longitud;
    datos_gps[3] = hora;
    datos_gps[4] = minuto;
    datos_gps[5] = segundo;
}

//Esta funcion te entrega los valores del sensor gy 91
void altitudmovimiento(){
 //Declaramos la lista que regresara el valor de cada variable
  Temp = bmp.readTemperature();
  Pres = bmp.readPressure();
  Alt = bmp.readAltitude(1013.25); /* Adjusted to local forecast! */
 
  mySensor.accelUpdate();
  aX = mySensor.accelX();
  aY = mySensor.accelY();
  aZ = mySensor.accelZ();
  aSqrt = mySensor.accelSqrt();

  mySensor.gyroUpdate();
  gX = mySensor.gyroX();
  gY = mySensor.gyroY();
  gZ = mySensor.gyroZ();

  mySensor.magUpdate();
  mX = mySensor.magX();
  mY = mySensor.magY();
  mZ = mySensor.magZ();
  mySensor.magHorizDirection();
  mDirection = mySensor.magHorizDirection();
  
  datos_AltMov[1] = Temp;
  datos_AltMov[2] = Pres;
  datos_AltMov[3] = Alt;
  datos_AltMov[4] = aX;
  datos_AltMov[5] = aY;
  datos_AltMov[6] = aZ;
  datos_AltMov[7] = aSqrt;
  datos_AltMov[8] = gX;
  datos_AltMov[9] = gY;
  datos_AltMov[10] = gZ;
  datos_AltMov[11] = mX;
  datos_AltMov[12] = mY;
  datos_AltMov[13] = mZ;
  datos_AltMov[14] = mDirection;
}
