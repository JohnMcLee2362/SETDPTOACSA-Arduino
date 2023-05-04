#include <stdio.h>
#include <SoftwareSerial.h>

//Librerias de sensores de altitud y movimiento
#include <Adafruit_BMP280.h>
#include <MPU9250_asukiaaa.h> 
#include <Servo.h>

Servo myservo; 

Adafruit_BMP280 bmp;
MPU9250_asukiaaa mySensor;

//Libreria de la sd
#include <SPI.h>
#include <SD.h>

#define buzzer 4
#define led 5


int pos_Servo = 0;    // variable to store the servo position
SoftwareSerial gps(3,2);//rx tx 
float Temp, Pres, Alt, aX, aY, aZ, aSqrt, gX, gY, gZ, mX, mY, mZ, mDirection;
uint8_t sensorId;
int result;

void setup() {

  pinMode(buzzer, OUTPUT);
  pinMode(led, OUTPUT);

  myservo.attach(6);  // attaches the servo on pin A3 to the servo object

  Serial.begin(9600);
  gps.begin(9600); 
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
    // don't do anything more:
    while (1);
  }
  Serial.println("card initialized.");
}

void loop() {
  float* datos_AltMov = altitudmovimiento();

  guardadito(datos_AltMov);//Guardamos los datos
  delay(100);


//Estos comentarios son lo que falta programar 
  // float* datos_GPS = GPS();

  // void guardadito(float* datos_GPS);//Guardamos los datos
  // delay(100);

  //if(ya pasamos 250m){
    //   for (pos = 0; pos <= 180; pos += 1) { // goes from 0 degrees to 180 degrees
    //   // in steps of 1 degree
    //   myservo.write(pos);              // tell servo to go to position in variable 'pos'
    //   delay(15);                       // waits 15 ms for the servo to reach the position
    // }
  //}

  // if(Ya aterrizamos ==True){
  //   alarma();
  // }

}


//Esta funcion te entrega los valores del sensor gy 91
float* altitudmovimiento(){
 //Declaramos la lista que regresara el valor de cada variable
  Temp = bmp.readTemperature();
  Pres = bmp.readPressure();
  Alt = bmp.readAltitude(1013.25); /* Adjusted to local forecast! */
  Serial.print(Temp);
  Serial.print(",");
  Serial.print(Pres);
  Serial.print(",");
  Serial.print(Alt);
  Serial.print(",");

  mySensor.accelUpdate();
  aX = mySensor.accelX();
  aY = mySensor.accelY();
  aZ = mySensor.accelZ();
  aSqrt = mySensor.accelSqrt();
  Serial.print(aX);
  Serial.print(",");
  Serial.print(aY);
  Serial.print(",");
  Serial.print(aZ);
  Serial.print(",");
  Serial.print(aSqrt);
  Serial.print(",");
  
  mySensor.gyroUpdate();
  gX = mySensor.gyroX();
  gY = mySensor.gyroY();
  gZ = mySensor.gyroZ();
  Serial.print(gX);
  Serial.print(",");
  Serial.print(gY);
  Serial.print(",");
  Serial.print(gZ);
  Serial.print(",");

  mySensor.magUpdate();
  mX = mySensor.magX();
  mY = mySensor.magY();
  mZ = mySensor.magZ();
  mySensor.magHorizDirection();
  mDirection = mySensor.magHorizDirection();
  Serial.print(mX);
  Serial.print(",");
  Serial.print(mY);
  Serial.print(",");
  Serial.print(mZ);
  Serial.print(",");
  Serial.println(mDirection);
  float valores[14]={Temp, Pres, Alt, aX, aY, aZ, aSqrt, gX, gY, gZ, mX, mY, mZ, mDirection};
  return(valores);
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
void guardadito(float* arreglo){
  int tamano = sizeof(arreglo);
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

  }
  // if the file isn't open, pop up an error:
  else {
    Serial.println("error opening datalog.txt");
  }
}

//La funcion para leer los datos del gps, a esta parte falta darle formato para que entregue unicamente la posicion del cansat. Con el link que te mande puedes ver como entrega 
// la posicion este gps
void gpsFuncion(){
   char gpsDatos=' ';
  if(gps.available()){
    gpsDatos=gps.read();
    Serial.print(gpsDatos);
  }
}
