#define BLYNK_PRINT Serial
#include <ESP8266WiFi.h>
#include <BlynkSimpleEsp8266.h>
#include <Wire.h>
#include <EEPROM.h>
extern "C" {
#include <espnow.h>
}

char auth[] = "auth token";
char ssid[] = "hotspot name";
char pass[] = "hotspot password";

#define CALSTEPS 256 // gyro and acc calibration steps
#define MPU6050_ADDRESS 0x68
#define SMPLRT_DIV 0
#define DLPF_CFG   4

#define ACC_ORIENTATION(X, Y, Z)  {accADC[ROLL]  = -Y; accADC[PITCH]  = X; accADC[YAW]  =  Z;}
#define GYRO_ORIENTATION(X, Y, Z) {gyroADC[ROLL] =  -X; gyroADC[PITCH] = -Y; gyroADC[YAW] = -Z;}

#define ACCRESO 4096
#define CYCLETIME 2500
#define CALITIME 10
#define MINTHROTTLE 1000
#define MIDRUD 1500
#define THRCORR 0

#define ROL 0
#define PIT 1
#define THR 2
#define RUD 3
#define AU1 4
#define AU2 5

#define GYRO     0
#define STABI    1
#define RTH      2

#define GYRO_I_MAX 10000.0
#define ANGLE_I_MAX 6000.0

#define FRONT_L 15
#define FRONT_R 14
#define REAR_R 13
#define REAR_L 12

#define RED_LED 2
#define BLUE_LED 16
#define GREEN_LED 0

WidgetLED led1(V10);
WidgetLED led2(V11);

extern int16_t accZero[3];
extern float yawRate;
extern float rollPitchRate;
extern float P_PID;
extern float I_PID;
extern float D_PID;
extern float P_Level_PID;
extern float I_Level_PID;
extern float D_Level_PID;
volatile boolean recv = true;

enum ang { ROLL, PITCH, YAW };
static int16_t gyroADC[3];
static int16_t accADC[3];
static int16_t gyroData[3];
static float anglerad[3]    = {0.0, 0.0, 0.0};
static float angle[3]    = {0.0, 0.0, 0.0};
extern int calibratingA;


static int16_t rcCommand[] = {0, 0, 0};

static int8_t flightmode;
static int8_t oldflightmode;
boolean armed = false;
uint8_t armct = 0;
int debugvalue = 0;


float yawRate = 6.0;
float rollPitchRate = 5.0;

float P_PID = 0.33;    // P8
float I_PID = 0.003;    // I8
float D_PID = 2.8;    // D8

float P_Level_PID = 0.35;   // P8
float I_Level_PID = 0.003;   // I8
float D_Level_PID = 2.8;   // D8

static int16_t axisPID[3];
static int16_t lastError[3] = {0, 0, 0};
static float errorGyroI[3] = {0, 0, 0};
static float errorAngleI[3] = {0, 0, 0};

//----------PID controller----------
int plotct;
int16_t deltab[6][3];
int8_t  deltabpt = 0;
int32_t deltasum[3];

#define CHANNELS 8
int16_t rcValue[CHANNELS];  // in us, center = 1500
static uint16_t servo[4];
void buf_to_rc()
{
  rcValue[0] = 1000;
  rcValue[1] = 1000;
  rcValue[2] = 1000;
  rcValue[3] = 1000;
  rcValue[4] = 1000;
  rcValue[5] = 1000;
  rcValue[6] = 1000;
  rcValue[7] = 1000;
}

//======================================IMU=========================//
enum cart { X, Y, Z };
#define GYRO_SCALE ((1998 * PI)/(32767.0f * 180.0f * 1000.0f)) // MPU6050
#define F_GYRO_SCALE 0.001 * GYRO_SCALE // in usec
/* Set the Gyro Weight for Gyro/Acc complementary filter
   Increasing this value would reduce and delay Acc influence on the output of the filter*/
#define GYR_CMPF_FACTOR 90
#define INV_GYR_CMPF_FACTOR   (1.0f / (GYR_CMPF_FACTOR  + 1.0f))

typedef struct fp_vector
{
  float X, Y, Z;
} t_fp_vector_def;

typedef union
{
  float A[3];
  t_fp_vector_def V;
} t_fp_vector;


float InvSqrt (float x)
{
  union {
    int32_t i;
    float   f;
  } conv;
  conv.f = x;
  conv.i = 0x5f3759df - (conv.i >> 1);
  return 0.5f * conv.f * (3.0f - x * conv.f * conv.f);
}

// Rotate Estimated vector(s) with small angle approximation, according to the gyro data
void rotateV(struct fp_vector *v, float* delta)
{
  fp_vector v_tmp = *v;
  v->Z -= delta[ROLL]  * v_tmp.X + delta[PITCH] * v_tmp.Y;
  v->X += delta[ROLL]  * v_tmp.Z - delta[YAW]   * v_tmp.Y;
  v->Y += delta[PITCH] * v_tmp.Z + delta[YAW]   * v_tmp.X;
}

void normalizeV(struct fp_vector *vec)
{
  float length;
  length = sqrtf(vec->X * vec->X + vec->Y * vec->Y + vec->Z * vec->Z);
  if (length != 0)
  {
    vec->X /= length;
    vec->Y /= length;
    vec->Z /= length;
  }
}

static t_fp_vector EstG = { 0.0, 0.0, (float)ACCRESO }; // x,y,z
static t_fp_vector EstM = { 0.0, 1.0, 0.0 }; // x,y,z
static float accData[3]  = {0, 0, 0};

uint32_t  tnow;

void getEstimatedAttitude()
{
  uint8_t axis;
  float deltaGyroAngle[3];
  uint32_t  tdiff;
  float scale;

  tdiff = micros() - tnow;
  tnow = micros();
  scale = (float)tdiff * F_GYRO_SCALE;

  // Initialization
  for (axis = 0; axis < 3; axis++)
  {
    deltaGyroAngle[axis] = (float)gyroADC[axis] * scale;
    accData[axis]  = (float)accADC[axis];
    gyroData[axis] = gyroADC[axis];
  }

  // AZ, EL
  rotateV(&EstG.V, deltaGyroAngle);

  // Apply complimentary filter (Gyro drift correction)
  for (axis = 0; axis < 3; axis++) EstG.A[axis] = (EstG.A[axis] * GYR_CMPF_FACTOR + accData[axis]) * INV_GYR_CMPF_FACTOR;

  // Attitude of the estimated vector
  float sqGX_sqGZ = sq(EstG.V.X) + sq(EstG.V.Z);
  float invmagXZ  = InvSqrt(sqGX_sqGZ);
  anglerad[ROLL]  = atan2(EstG.V.X , EstG.V.Z);
  anglerad[PITCH] = atan2(EstG.V.Y , invmagXZ * sqGX_sqGZ);
  angle[ROLL]  = 572.95f * anglerad[ROLL];
  angle[PITCH] = 572.95f * anglerad[PITCH];

  // Yaw
  rotateV(&EstM.V, deltaGyroAngle);
#if MAG
  for (axis = 0; axis < 3; axis++)
    EstM.A[axis]  += (imu.magADC[axis] - EstM.A[2 * axis + 1]) << (16 - GYR_CMPFM_FACTOR);
#else
  normalizeV(&EstM.V);
#endif

  float invG = InvSqrt(sqGX_sqGZ + EstG.V.Y * EstG.V.Y);
  anglerad[YAW] = atan2(EstM.V.Z * EstG.V.X - EstM.V.X * EstG.V.Z,
                        (EstM.V.Y * sqGX_sqGZ - (EstM.V.X * EstG.V.X + EstM.V.Z * EstG.V.Z) * EstG.V.Y) * invG );
  angle[YAW] = 572.95f * anglerad[YAW];

}
BLYNK_CONNECTED() {

  Blynk.syncVirtual(V0);
  Blynk.syncVirtual(V1);
  Blynk.syncVirtual(V2);
  Blynk.syncVirtual(V3);
  Blynk.syncVirtual(V5);
  Blynk.syncVirtual(V6);
  Blynk.syncVirtual(V10);
  Blynk.syncVirtual(V11);//sync value

}
//====================================MPU6050===================================//
void i2cRead(uint8_t Address, uint8_t Register, uint8_t Nbytes, uint8_t* Data)
{
  Wire.beginTransmission(Address);
  Wire.write(Register);
  Wire.endTransmission();
  Wire.requestFrom(Address, Nbytes);
  uint8_t index = 0;
  while (Wire.available())
    Data[index++] = Wire.read();
}

void i2cWriteByte(uint8_t Address, uint8_t Register, uint8_t Data)
{
  Wire.beginTransmission(Address);
  Wire.write(Register);
  Wire.write(Data);
  Wire.endTransmission();
}

#define ADDRESS 0x03
int calibratingG = CALSTEPS;
int calibratingA = 0;
int16_t gyroZero[3] = {0, 0, 0};
int16_t accZero[3] = {0, 0, 0};

// ****************
// GYRO common part
// ****************
void GYRO_Common()
{
  static int32_t g[3];
  uint8_t axis, tilt = 0;

  if (calibratingG > 0)
  {
    for (axis = 0; axis < 3; axis++)
    {
      // Reset g[axis] at start of calibration
      if (calibratingG == CALSTEPS) g[axis] = 0;
      // Sum up 512 readings
      g[axis] += gyroADC[axis];
      // Clear global variables for next reading
      gyroADC[axis] = 0;
      gyroZero[axis] = 0;
      if (calibratingG == 1)
      {
        if (g[axis] >= 0) g[axis] += CALSTEPS / 2;
        else            g[axis] -= CALSTEPS / 2;
        gyroZero[axis] = g[axis] / CALSTEPS;
      }
    }
    calibratingG--;
  }

  for (axis = 0; axis < 3; axis++)
    gyroADC[axis] = gyroADC[axis] - gyroZero[axis];
}

// ****************
// ACC common part
// ****************

void ACC_Common()
{
  static int32_t a[3];

  if (calibratingA > 0)
  {
    for (uint8_t axis = 0; axis < 3; axis++)
    {
      // Reset a[axis] at start of calibration
      if (calibratingA == CALSTEPS) a[axis] = 0;
      // Sum up 512 readings
      a[axis] += accADC[axis];
      // Clear global variables for next reading
      accADC[axis] = 0;
      accZero[axis] = 0;
    }
    // Calculate average, shift Z down by acc_1G and store values in EEPROM at end of calibration
    if (calibratingA == 1)
    {
      if (a[0] >= 0) a[0] += CALSTEPS / 2; else a[0] -= CALSTEPS / 2;
      if (a[1] >= 0) a[1] += CALSTEPS / 2; else a[1] -= CALSTEPS / 2;
      if (a[2] >= 0) a[2] += CALSTEPS / 2; else a[2] -= CALSTEPS / 2;
      accZero[0] = a[0] / CALSTEPS;
      accZero[1] = a[1] / CALSTEPS;
      accZero[2] = a[2] / CALSTEPS - ACCRESO;
      Serial.print("  "); Serial.print(accZero[0]); Serial.println();
      Serial.print("  "); Serial.print(accZero[1]); Serial.println();
      Serial.print("  "); Serial.print(accZero[2]); Serial.println();
    }
    calibratingA--;
  }
  accADC[0] -=  accZero[0];
  accADC[1] -=  accZero[1];
  accADC[2] -=  accZero[2];
}

void Gyro_getADC ()
{
  uint8_t rawADC[6];
  i2cRead(MPU6050_ADDRESS, 0x43, 6, rawADC);
  GYRO_ORIENTATION( ((rawADC[0] << 8) | rawADC[1]) ,
                    ((rawADC[2] << 8) | rawADC[3]) ,
                    ((rawADC[4] << 8) | rawADC[5]) );
  GYRO_Common();
}

void ACC_getADC ()
{
  uint8_t rawADC[6];
  i2cRead(MPU6050_ADDRESS, 0x3B, 6, rawADC);
  ACC_ORIENTATION( ((rawADC[0] << 8) | rawADC[1]) ,
                   ((rawADC[2] << 8) | rawADC[3]) ,
                   ((rawADC[4] << 8) | rawADC[5]) );
  ACC_Common();
}

void MPU6050_readId()
{
  uint8_t id;
  i2cRead(MPU6050_ADDRESS, 0x75, 1, &id);
  if (id == 0x68) Serial.println("6050 ID OK");
  else Serial.println("6050 ID Failed");
}

void MPU6050_init()
{
  Wire.begin();
  Wire.setClock(400000);

  //Gyro_init
  i2cWriteByte(MPU6050_ADDRESS, 0x6B, 0x80);             //PWR_MGMT_1    -- DEVICE_RESET 1
  delay(50);

  i2cWriteByte(MPU6050_ADDRESS, 0x6B, 0x03);             //PWR_MGMT_1    -- SLEEP 0; CYCLE 0; TEMP_DIS 0; CLKSEL 3 (PLL with Z Gyro reference)
  i2cWriteByte(MPU6050_ADDRESS, 0x1A, DLPF_CFG);         //CONFIG        -- EXT_SYNC_SET 0 (disable input pin for data sync) ; default DLPF_CFG = 0 => ACC bandwidth = 260Hz  GYRO bandwidth = 256Hz)
  i2cWriteByte(MPU6050_ADDRESS, 0x1B, 0x18);             //GYRO_CONFIG   -- FS_SEL = 3: Full scale set to 2000 deg/sec
  delay(50);

  //ACC_init
  i2cWriteByte(MPU6050_ADDRESS, 0x1C, 0x10);             //ACCEL_CONFIG
  delay(50);
}

//=====================================EEPROM======================//
typedef union int16_ty
{
  int16_t d;
  byte    b[2];
};
typedef union float_ty
{
  float d;
  byte  b[4];
};
void write_int16(int pos, int16_t d)
{
  int16_ty loc;
  loc.d = d;
  EEPROM.write(pos++, loc.b[0]);
  EEPROM.write(pos++, loc.b[1]);
}
int16_t read_int16(int pos)
{
  int16_ty loc;
  loc.b[0] = EEPROM.read(pos++);
  loc.b[1] = EEPROM.read(pos++);
  return loc.d;
}

void write_float(int pos, float d)
{
  float_ty loc;
  loc.d = d;
  EEPROM.write(pos++, loc.b[0]);
  EEPROM.write(pos++, loc.b[1]);
  EEPROM.write(pos++, loc.b[2]);
  EEPROM.write(pos++, loc.b[3]);
}
float read_float(int pos)
{
  float_ty loc;
  loc.b[0] = EEPROM.read(pos++);
  loc.b[1] = EEPROM.read(pos++);
  loc.b[2] = EEPROM.read(pos++);
  loc.b[3] = EEPROM.read(pos++);
  return loc.d;
}

void ACC_Read()
{
  accZero[0] = read_int16(0);
  accZero[1] = read_int16(2);
  accZero[2] = read_int16(4);
}
void ACC_Store()
{
  write_int16(0, accZero[0]);
  write_int16(2, accZero[1]);
  write_int16(4, accZero[2]);
  EEPROM.write(63, 0x55);
  EEPROM.commit();
}
void PID_Read()
{
  yawRate       = read_float(10);
  rollPitchRate = read_float(14);
  P_PID         = read_float(18);
  I_PID         = read_float(22);
  D_PID         = read_float(26);
  P_Level_PID   = read_float(30);
  I_Level_PID   = read_float(34);
  D_Level_PID   = read_float(38);
}
void PID_Store()
{
  write_float(10, yawRate);
  write_float(14, rollPitchRate);
  write_float(18, P_PID);
  write_float(22, I_PID);
  write_float(26, D_PID);
  write_float(30, P_Level_PID);
  write_float(34, I_Level_PID);
  write_float(38, D_Level_PID);
  EEPROM.write(62, 0xAA);
  EEPROM.commit();
}

//==================================PID====================================//
void pid()
{
  uint8_t axis;
  float errorAngle;
  float AngleRateTmp, RateError;
  float PTerm, ITerm, DTerm;
  int16_t delta;


  //----------PID controller----------
  for (axis = 0; axis < 3; axis++)
  {
    if (axis == 2)
    { //YAW is always gyro-controlled
      AngleRateTmp = yawRate * rcCommand[YAW];
      RateError = AngleRateTmp - gyroData[axis];
      PTerm = RateError * P_PID;

      delta           = RateError - lastError[axis];
      lastError[axis] = RateError;
      deltasum[axis] += delta;
      deltasum[axis] -= deltab[deltabpt][axis];
      deltab[deltabpt][axis] = delta;
      DTerm = deltasum[axis] * D_PID;

      ITerm = 0.0;

      deltabpt++;
      if (deltabpt >= 6) deltabpt = 0;
    }
    else
    {
      if (flightmode == GYRO) // GYRO mode
      {
        //control is GYRO based (ACRO - direct sticks control is applied to rate PID
        AngleRateTmp = rollPitchRate * rcCommand[axis];
        RateError = AngleRateTmp - gyroData[axis];
        //-----calculate P-term
        PTerm = RateError * P_PID;
        //-----calculate D-term
        delta           = RateError - lastError[axis];
        lastError[axis] = RateError;

        deltasum[axis] += delta;
        deltasum[axis] -= deltab[deltabpt][axis];
        deltab[deltabpt][axis] = delta;

        DTerm = deltasum[axis] * D_PID;
        //-----calculate I-term
        ITerm = 0.0;
      }
      else // STABI mode
      {
        // calculate error and limit the angle to 45 degrees max inclination
        errorAngle = constrain(rcCommand[axis], -450, +450) - angle[axis]; //16 bits is ok here
        //it's the ANGLE mode - control is angle based, so control loop is needed
        //-----calculate P-term
        PTerm = errorAngle * P_Level_PID;
        //-----calculate D-term
        delta = - gyroData[axis];
        DTerm = delta * D_Level_PID;
        //-----calculate I-term
        errorAngleI[axis]  += errorAngle * I_Level_PID;
        errorAngleI[axis]  = constrain(errorAngleI[axis], -ANGLE_I_MAX, +ANGLE_I_MAX);
        ITerm = errorAngleI[axis] * 0.01;
      }
    }

    //-----calculate total PID output
    axisPID[axis] =  PTerm + ITerm + DTerm;

    /*
      if (axis==2)
      {
      Serial.print(AngleRateTmp); Serial.print("  ");
      Serial.print(RateError); Serial.print("  ");
      Serial.print(PTerm); Serial.print("  ");
      Serial.println();
      }
    */
    /*
      if (axis==0)
      {
      Serial.print(PTerm); Serial.print("  ");
      Serial.print(ITerm); Serial.print("  ");
      Serial.print(DTerm); Serial.print("  ");
      if      (plotct == 0) Serial.print(-2000);
      else if (plotct == 1) Serial.print( 2000);
      else                  Serial.print( 0);
      if (plotct == 300) plotct = 0; else plotct++;
      Serial.println();
      }
    */
  }
}

void zeroGyroAccI()
{
  for (int axis = 0; axis < 3; axis++)
  {
    errorAngleI[axis] = 0.0;
    errorGyroI[axis] = 0.0;
  }
}

//=====================================OUTPUT=====================//
void mix()
{
  if (armed & (rcValue[THR] > MINTHROTTLE))
  {
    servo[0] = constrain(rcValue[THR] + axisPID[ROLL] - axisPID[PITCH] - axisPID[YAW], 1000, 2000); //FRONT_L
    servo[1] = constrain(rcValue[THR] - axisPID[ROLL] - axisPID[PITCH] + axisPID[YAW], 1000, 2000); //FRONT_R
    servo[2] = constrain(rcValue[THR] - axisPID[ROLL] + axisPID[PITCH] - axisPID[YAW], 1000, 2000); //REAR_R
    servo[3] = constrain(rcValue[THR] + axisPID[ROLL] + axisPID[PITCH] + axisPID[YAW], 1000, 2000); //REAR_L
  }
  else
  {
    axisPID[0] = 0; axisPID[1] = 0; axisPID[2] = 0;
    servo[0] = 1000; servo[1] = 1000; servo[2] = 1000; servo[3] = 1000;
  }
  servo[0] = map(servo[0], 1000, 2000, 0, 1023);
  servo[1] = map(servo[1], 1000, 2000, 0, 1023);
  servo[2] = map(servo[2], 1000, 2000, 0, 1023);
  servo[3] = map(servo[3], 1000, 2000, 0, 1023);
}

void writeServo()
{

  analogWrite(FRONT_L, servo[0]);
  analogWrite(FRONT_R, servo[1]);
  analogWrite(REAR_R, servo[2]);
  analogWrite(REAR_L, servo[3]);
}

void initServo()
{
  pinMode(FRONT_L, OUTPUT);
  pinMode(FRONT_R, OUTPUT);
  pinMode(REAR_R, OUTPUT);
  pinMode(REAR_L, OUTPUT);

}

//==================================================RECEIVER==================================================//
uint32_t rxt; // receive time, used for falisave

BLYNK_WRITE(V0)
{
  rcValue[0] = param.asInt();
}
BLYNK_WRITE(V1)
{
  rcValue[1] = param.asInt();
}
BLYNK_WRITE(V2)
{
  rcValue[2] = param.asInt();
}
BLYNK_WRITE(V3)
{
  rcValue[3] = param.asInt();

}

BLYNK_WRITE(V5)
{
  rcValue[AU1] = param.asInt();
}
BLYNK_WRITE(V6)
{
  armed = param.asInt();
}

void setup() {
  Serial.begin(115200); Serial.println();
  pinMode(RED_LED, OUTPUT);
  pinMode(BLUE_LED, OUTPUT);
  pinMode(GREEN_LED, OUTPUT);
  digitalWrite(RED_LED, LOW);
  digitalWrite(BLUE_LED, LOW);
  digitalWrite(GREEN_LED, LOW);
  Blynk.begin(auth, ssid, pass);
  delay(3000); // give it some time to stop shaking after battery plugin
  MPU6050_init();
  MPU6050_readId(); // must be 0x68, 104dec
  led1.on();
  EEPROM.begin(64);
  if (EEPROM.read(63) != 0x55)
  {
    Serial.println("Need to do ACC calib");
    for (int i = 0; i < 10; i++)
    {
      digitalWrite(GREEN_LED, HIGH);
      delay(300);
      digitalWrite(GREEN_LED, LOW);
      delay(300);
    }
  }
  else
  {
    ACC_Read(); // eeprom is initialized
  }
  if (EEPROM.read(62) != 0xAA)
  {
    Serial.println("Need to check and write PID");
    for (int i = 0; i < 10; i++)
    {
      digitalWrite(GREEN_LED, HIGH);
      delay(300);
      digitalWrite(GREEN_LED, LOW);
      delay(300);
    }
  }
  else
  {
    PID_Read(); // eeprom is initialized
  }
  delay(1000);
  initServo();
  digitalWrite(GREEN_LED, HIGH);
}

void loop() {


  uint32_t now, mnow, diff;
  now = millis(); // actual time
  if (debugvalue == 5) mnow = micros();
  if (1)
  {
    if (debugvalue == 4) Serial.printf("%4d %4d %4d %4d \n", rcValue[0], rcValue[1], rcValue[2], rcValue[3]);

    if      (rcValue[AU1] < 1500) flightmode = GYRO;
    else flightmode = STABI;
    if (oldflightmode != flightmode)
    {

      if (flightmode == GYRO)
      {
        led1.on();
        led2.off();

      }
      else
      {
        led2.on();
        led1.off();
      }

      zeroGyroAccI();
      oldflightmode = flightmode;
    }

    if (armed)
    {
      digitalWrite(GREEN_LED, LOW);
      digitalWrite(RED_LED, HIGH);
      digitalWrite(BLUE_LED, HIGH);
      rcValue[THR]    -= THRCORR;
      rcCommand[ROLL]  = rcValue[ROL] - MIDRUD;
      rcCommand[PITCH] = rcValue[PIT] - MIDRUD;
      rcCommand[YAW]   = rcValue[RUD] - MIDRUD;
    }
    else
    {
      digitalWrite(GREEN_LED, HIGH);
      digitalWrite(RED_LED, LOW);
      digitalWrite(BLUE_LED, LOW);
      rcValue[THR] = 1000;
    }

    rxt = millis();
  }


  //==========================important==========================================================//
  Gyro_getADC();
  ACC_getADC();
  getEstimatedAttitude();
  pid();
  mix();
  if (debugvalue != 6)
  {
    writeServo();
  }
  //=============================================================================================//
  // Failsave part
  if (now > rxt + 90)
  {
    rcValue[THR] = MINTHROTTLE;
    if (debugvalue == 5) Serial.printf("RC Failsafe after %d \n", now - rxt);
    rxt = now;
  }
  //===========================================DEBUG============================================//
  // parser part
  if (Serial.available())
  {
    char ch = Serial.read();
    // Perform ACC calibration
    if (ch == 10) Serial.println();
    else if (ch == 'A')
    {
      Serial.println("Doing ACC calib");


      digitalWrite(GREEN_LED, LOW);

      calibratingA = CALSTEPS;
      while (calibratingA != 0)
      {
        delay(CALITIME);
        ACC_getADC();
      }
      ACC_Store();
      Serial.println("ACC calib Done");
      digitalWrite(GREEN_LED, HIGH);
    }
    else if (ch == 'R')
    {
      Serial.print("Act Rate : ");
      Serial.print(yawRate); Serial.print("  ");
      Serial.print(rollPitchRate); Serial.println();
      Serial.println("Act PID :");
      Serial.print(P_PID); Serial.print("  ");
      Serial.print(I_PID); Serial.print("  ");
      Serial.print(D_PID); Serial.println();
      Serial.print(P_Level_PID); Serial.print("  ");
      Serial.print(I_Level_PID); Serial.print("  ");
      Serial.print(D_Level_PID); Serial.println();
    }
    else if (ch == 'D')
    {
      digitalWrite(GREEN_LED, LOW);
      Serial.println("Loading default PID");
      yawRate = 6.0;
      rollPitchRate = 5.0;
      P_PID = 0.33;    // P8
      I_PID = 0.03;    // I8
      D_PID = 2.8;
      P_Level_PID = 0.35;   // P8
      I_Level_PID = 0.03;   // I8
      D_Level_PID = 2.8;
      PID_Store();
      digitalWrite(GREEN_LED, HIGH);
    }
    else if (ch == 'W')
    {
      char ch = Serial.read();
      int n = Serial.available();
      if (n == 3)
      {
        n = readsernum();
        if      (ch == 'p') {
          P_PID       = float(n) * 0.01 + 0.004;
          Serial.print("pid P ");
          Serial.print(P_PID);
        }
        else if (ch == 'i') {
          I_PID       = float(n) * 0.01 + 0.004;
          Serial.print("pid I ");
          Serial.print(I_PID);
        }
        else if (ch == 'd') {
          D_PID       = float(n) * 0.01 + 0.004;
          Serial.print("pid D ");
          Serial.print(D_PID);
        }
        else if (ch == 'P') {
          P_Level_PID = float(n) * 0.01 + 0.004;
          Serial.print("pid Level P ");
          Serial.print(P_Level_PID);
        }
        else if (ch == 'I') {
          I_Level_PID = float(n) * 0.01 + 0.004;
          Serial.print("pid Level I ");
          Serial.print(I_Level_PID);
        }
        else if (ch == 'D') {
          D_Level_PID = float(n) * 0.01 + 0.004;
          Serial.print("pid Level D ");
          Serial.print(D_Level_PID);
        }
        else Serial.println("unknown command");
      }
      else if (ch == 'S') {
        PID_Store();
        Serial.print("stored in EEPROM");
      }
      else
      {
        Serial.println("Input format wrong");
        Serial.println("Wpxx, Wixx, Wdxx - write gyro PID, example: Wd13");
        Serial.println("WPxx, WIxx, WDxx - write level PID, example: WD21");
      }
    }
    else if (ch >= '0' && ch <= '9') debugvalue = ch - '0';
    else
    {
      Serial.println("A - acc calib");
      Serial.println("D - write default PID");
      Serial.println("R - read actual PID");
      Serial.println("Wpxx, Wixx, Wdxx - write gyro PID");
      Serial.println("WPxx, WIxx, WDxx - write level PID");
      Serial.println("WS - Store PID in EEPROM");
      Serial.println("Display data:");
      Serial.println("0 - off");
      Serial.println("1 - Gyro values");
      Serial.println("2 - Acc values");
      Serial.println("3 - Angle values");
      Serial.println("4 - RC values");
      Serial.println("5 - Cycletime");
      Serial.println("6 - Servo Values");
    }

  }
  if      (debugvalue == 1) Serial.printf("%4d %4d %4d \n", gyroADC[0], gyroADC[1], gyroADC[2]);
  else if (debugvalue == 2) Serial.printf("%5d %5d %5d \n", accADC[0], accADC[1], accADC[2]);
  else if (debugvalue == 3) Serial.printf("%3f %3f \n", angle[0], angle[1]);
  else if (debugvalue == 6) Serial.printf("%3d %3d %3d %3d \n", servo[0], servo[1], servo[2], servo[3]);
  if (debugvalue == 5)
  {
    diff = micros() - mnow;
    Serial.println(diff);
  }
  Blynk.run();

  while ( micros() - mnow < CYCLETIME)
  {
  }
  //delay(CYCLETIME-1);

}

int readsernum()
{
  int num;
  char numStr[3];
  numStr[0] = Serial.read();
  numStr[1] = Serial.read();
  return atol(numStr);
}
