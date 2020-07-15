#include <Mouse.h>

/// I2C device class (I2Cdev) demonstration Arduino sketch for MPU6050 class using DMP (MotionApps v2.0)
// 6/21/2012 by Jeff Rowberg <jeff@rowberg.net>
// Updates should (hopefully) always be available at https://github.com/jrowberg/i2cdevlib
//

// I2Cdev and MPU6050 must be installed as libraries, or else the .cpp/.h files
// for both classes must be in the include path of your project
//#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
//#include "MPU6050.h" // not necessary if using MotionApps include file


// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for SparkFun breakout and InvenSense evaluation board)
// AD0 high = 0x69
MPU6050 mpu;
//MPU6050 mpu(0x69); // <-- use for AD0 high

/* =========================================================================
   NOTE: In addition to connection 3.3v, GND, SDA, and SCL, this sketch
   depends on the MPU-6050's INT pin being connected to the Arduino's
   external interrupt #0 pin. On the Arduino Uno and Mega 2560, this is
   digital I/O pin 2.
 * ========================================================================= */

/* =========================================================================
   NOTE: Arduino v1.0.1 with the Leonardo board generates a compile error
   when using Serial.write(buf, len). The Teapot output uses this method.
   The solution requires a modification to the Arduino USBAPI.h file, which
   is fortunately simple, but annoying. This will be fixed in the next IDE
   release. For more info, see these links:
   http://arduino.cc/forum/index.php/topic,109987.0.html
   http://code.google.com/p/arduino/issues/detail?id=958
 * ========================================================================= */



// uncomment "OUTPUT_READABLE_QUATERNION" if you want to see the actual
// quaternion components in a [w, x, y, z] format (not best for parsing
// on a remote host such as Processing or something though)
//#define OUTPUT_READABLE_QUATERNION

// uncomment "OUTPUT_READABLE_EULER" if you want to see Euler angles
// (in degrees) calculated from the quaternions coming from the FIFO.
// Note that Euler angles suffer from gimbal lock (for more info, see
// http://en.wikipedia.org/wiki/Gimbal_lock)
//#define OUTPUT_READABLE_EULER

// uncomment "OUTPUT_READABLE_YAWPITCHROLL" if you want to see the yaw/
// pitch/roll angles (in degrees) calculated from the quaternions coming
// from the FIFO. Note this also requires gravity vector calculations.
// Also note that yaw/pitch/roll angles suffer from gimbal lock (for
// more info, see: http://en.wikipedia.org/wiki/Gimbal_lock)
#define OUTPUT_READABLE_YAWPITCHROLL

// uncomment "OUTPUT_READABLE_REALACCEL" if you want to see acceleration
// components with gravity removed. This acceleration reference frame is
// not compensated for orientation, so +X is always +X according to the
// sensor, just without the effects of gravity. If you want acceleration
// compensated for orientation, us OUTPUT_READABLE_WORLDACCEL instead.
//#define OUTPUT_READABLE_REALACCEL

// uncomment "OUTPUT_READABLE_WORLDACCEL" if you want to see acceleration
// components with gravity removed and adjusted for the world frame of
// reference (yaw is relative to initial orientation, since no magnetometer
// is present in this case). Could be quite handy in some cases.
//#define OUTPUT_READABLE_WORLDACCEL

// uncomment "OUTPUT_TEAPOT" if you want output that matches the
// format used for the InvenSense teapot demo
//#define OUTPUT_TEAPOT



#define INTERRUPT_PIN 7 // use pin 2 on Arduino Uno & most boards
#define LED_PIN 8 // (Arduino is 13, Teensy is 11, Teensy++ is 6)
bool blinkState = false;

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
float yaw, pitch, roll;
// packet structure for InvenSense teapot demo
uint8_t teapotPacket[14] = { '$', 0x02, 0,0, 0,0, 0,0, 0,0, 0x00, 0x00, '\r', '\n' };

//Botao
#define BOTAO 6
//Modo Sleep
#define TIMEOUT_SLEEP 2
bool g_ativaMouse = true; 
//Timing
long previousMillis = 0;
long interval = 0;

//Mouse
const int sensitivity = 45;
float vertZero, horzZero;
float vertValue, horzValue;  // Stores current analog output of each axis


// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
    mpuInterrupt = true;
}
void callback()
{

}

unsigned long countSecs()
{
  static unsigned long  segundos = 0;
  static int interval  = 1000;
  static bool inicializa = true;
  unsigned long currentMillis;
  currentMillis = millis(); 
  
  if(inicializa)
  {
    previousMillis =  currentMillis ;
    inicializa = false;
  }

  if(currentMillis - previousMillis > interval) 
  {  
    previousMillis = currentMillis; 
    segundos ++;
  }
   
  // Serial.print(segundos);
  // Serial.print("\n");
  return segundos;
}
bool botaoMouse()
{
  //Fazer debounce
  static int valorBotao = 0;
  static int botaoDebounce = 0;
  valorBotao = digitalRead(BOTAO);
  if(!valorBotao)
  {
    botaoDebounce =  false;
  }
  else
  {
    botaoDebounce = true;
  }
  return botaoDebounce;
}
enum 
{
  Estado_resetaContador = 0,
  Estado_Timeout2Sleep,
  Estado_EntraModoSleep,
};
void mouseSleep(float vertical ,float horizontal,unsigned long segundos)
{
  static int counter = 0;
  static int estado = 0;
  static bool botao = true;
  static bool mexeu = true;
 // static int estado = 0;
  static int previousSecs = 0;
  static int tempo = 0;
  //unsigned long currentMillis = millis(); 
  vertical = abs(vertical) ;
  horizontal = abs(horizontal) ;

  if(vertical <= 0.03  && horizontal <= 0.03 )
  {
      mexeu = false;

  }
  else
  {
    mexeu = true;
  }
 
  switch(estado)
  {
    case Estado_resetaContador:
      tempo = 0;
      previousSecs = segundos;
      if(mexeu == false)
      {
        estado = Estado_Timeout2Sleep;
      }
      break;
    case Estado_Timeout2Sleep:
      tempo = segundos - previousSecs;
      if(tempo  < TIMEOUT_SLEEP )
      {
        if(mexeu)
        {
          estado =  Estado_resetaContador;
        }
      }
      else if(tempo>=TIMEOUT_SLEEP)
      {
        estado =  Estado_EntraModoSleep;
      }
      break;
    case Estado_EntraModoSleep:
      g_ativaMouse = false;
      botao = botaoMouse();
      if(!botao)
      {
        g_ativaMouse = true;
        estado = Estado_resetaContador;
      }
      break;
    default:
      break;
  }
  // Serial.print(vertical);
  // Serial.print(" ");
  // Serial.print(horizontal);
  //  Serial.print(" ");
  // Serial.print(mexeu);
  //  Serial.print(" ");
  // Serial.print(tempo);
  // Serial.print("\n");
}


// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================

void setup() {
    static int leBotao;
    pinMode(BOTAO, INPUT);
    // join I2C bus (I2Cdev library doesn't do this automatically)
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif

    // initialize serial communication
    // (115200 chosen because it is required for Teapot Demo output, but it's
    // really up to you depending on your project)
    // Serial.begin(115200);
    //while (!Serial); // wait for Leonardo enumeration, others continue immediately

    // NOTE: 8MHz or slower host processors, like the Teensy @ 3.3V or Arduino
    // Pro Mini running at 3.3V, cannot handle this baud rate reliably due to
    // the baud timing being too misaligned with processor ticks. You must use
    // 38400 or slower in these cases, or use some kind of external separate
    // crystal solution for the UART timer.

    // initialize device
    //Serial.println(F("Initializing I2C devices..."));
    mpu.initialize();
    pinMode(INTERRUPT_PIN, INPUT);

    // verify connection
    // Serial.println(F("Testing device connections..."));
    // Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

    // wait for ready
    //Serial.println(F("\nSend any character to begin DMP programming and demo: "));
   // Serial.println(F("\nPress button to begin DMP programming and demo: "));
    //Inicializar com botão**
    // while (Serial.available() && Serial.read()); // empty buffer
    // while (!Serial.available());                 // wait for data
    // while (Serial.available() && Serial.read()); // empty buffer again
    
    do
    {
      leBotao = digitalRead(BOTAO); 
    }
    while(leBotao);

    // load and configure the DMP
    //Serial.println(F("Initializing DMP..."));
    devStatus = mpu.dmpInitialize();

    // supply your own gyro offsets here, scaled for min sensitivity
    mpu.setXGyroOffset(220);
    mpu.setYGyroOffset(76);
    mpu.setZGyroOffset(-85);
    mpu.setZAccelOffset(1788); // 1688 factory default for my test chip

    // make sure it worked (returns 0 if so)
    if (devStatus == 0) {
        // Calibration Time: generate offsets and calibrate our MPU6050
        mpu.CalibrateAccel(6);
        mpu.CalibrateGyro(6);
        mpu.PrintActiveOffsets();
        // turn on the DMP, now that it's ready
        //Serial.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);

        // enable Arduino interrupt detection
        //Serial.print(F("Enabling interrupt detection (Arduino external interrupt "));
        //Serial.print(digitalPinToInterrupt(INTERRUPT_PIN));
        //Serial.println(F(")..."));
        attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();

        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        //Serial.println(F("DMP ready! Waiting for first interrupt..."));
        dmpReady = true;

        // get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();
    } else {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
        // Serial.print(F("DMP Initialization failed (code "));
        // Serial.print(devStatus);
        // Serial.println(F(")"));
    }

    // configure LED for output
    pinMode(LED_PIN, OUTPUT);
   
    yaw = 0.0;
    pitch = 0.0;
    roll = 0.0;
}



// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================

void loop() {
  static int estadoBotao;
  unsigned long segundos = 0;
    // if programming failed, don't try to do anything
    if (!dmpReady) return;
    // read a packet from FIFO
    if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) { // Get the Latest packet 
       
        #ifdef OUTPUT_READABLE_YAWPITCHROLL
            // display Euler angles in degrees
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
            // Serial.print("ypr\t");
            // Serial.print(ypr[0] * 180/M_PI);
            // Serial.print("\t");
            // Serial.print(ypr[1] * 180/M_PI);
            // Serial.print("\t");
            // Serial.println(ypr[2] * 180/M_PI);
            yaw = ypr[1] /PI * 180;
            pitch = ypr[2] /PI * 180;
            roll = ypr[0] /PI * 180;
        #endif
        vertValue = yaw - vertZero;
        horzValue = roll - horzZero;
        vertZero = yaw;
        horzZero = roll;
        // estadoBotao = digitalRead(BOTAO);
        estadoBotao = botaoMouse();

        //Acionamentos Mouse
        
          if (vertValue != 0)
            Mouse.move(0, vertValue * sensitivity, 0);                                      // move mouse on y axis
          if (horzValue != 0)
            Mouse.move(horzValue * sensitivity, 0, 0);                                      // move mouse on x axis
          
          //if (estadoBotao == false)
          //{
          //  Mouse.press();
         // }
          //else
         // {
          //  Mouse.release();
          //}
        
        // segundos = countSecs();
        // mouseSleep(vertValue,horzValue,segundos);
        /* Serial.print(yaw);
         Serial.print(" ");
         Serial.print(pitch);
         Serial.print(" ");
         Serial.print(roll);
         Serial.print(" ");
         Serial.print("\n");*/
        // blink LED to indicate activity
    }
}
