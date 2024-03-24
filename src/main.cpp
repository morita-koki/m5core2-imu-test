#include <M5Core2.h>
#include <string>

#include "filter.hpp"

// float acc_x = 0.f;
// float acc_y = 0.f;
// float acc_z = 0.f;

float acc_bias_x = 0.f;
float acc_bias_y = 0.f;
float acc_bias_z = 0.f;

// float gyro_x = 0.f;
// float gyro_y = 0.f;
// float gyro_z = 0.f;

float gyro_bias_x = 0.f;
float gyro_bias_y = 0.f;
float gyro_bias_z = 0.f;

// float roll = 0.f;
// float pitch = 0.f;
// float yaw = 0.f;

unsigned long prev_micros = 0UL;


int state = 0; // 0: main, 1: calibration, 2: resetGyro

TFT_eSprite sprite = TFT_eSprite(&M5.Lcd);

void print_loading(int now, int total)
{
  int maxwidth = 240;
  int height = 20;
  M5.Lcd.drawRect(40, 40, maxwidth, height, WHITE);
  M5.Lcd.fillRect(40, 40, maxwidth * now / total, height, WHITE);
}


void Calibration()
{
  delay(2000);

  float sum_gx = 0.f;
  float sum_gy = 0.f;
  float sum_gz = 0.f;
  float sum_ax = 0.f;
  float sum_ay = 0.f;
  float sum_az = 0.f;

  const int calibcount = 1000;
  for (size_t i = 0; i < calibcount; ++i) {
    M5.update();

    print_loading(i, calibcount);

    float gx, gy, gz;
    float ax, ay, az;
    M5.IMU.getGyroData(&gy, &gz, &gx);
    M5.IMU.getAccelData(&ay, &az, &ax);

    sum_gx += gx;
    sum_gy += gy;
    sum_gz += gz;
    sum_ax += ax;
    sum_ay += ay;
    sum_az += az - 1.00;

    M5.Lcd.setCursor(40, 140);
    M5.Lcd.printf("gyro bias");
    M5.Lcd.setCursor(50, 160);
    M5.Lcd.printf("%+.2f, %+.2f, %+.2f", gx, gy, gz);

    M5.Lcd.setCursor(40, 180);
    M5.Lcd.printf("acc bias");
    M5.Lcd.setCursor(50, 200);
    M5.Lcd.printf("%+.2f, %+.2f, %+.2f", ax, ay, az - 1.00);
    // M5.Lcd.clear();
  }

  gyro_bias_x = sum_gx / calibcount;
  gyro_bias_y = sum_gy / calibcount;
  gyro_bias_z = sum_gz / calibcount;

  acc_bias_x = sum_ax / calibcount;
  acc_bias_y = sum_ay / calibcount;
  acc_bias_z = sum_az / calibcount;

  M5.Lcd.clear();

}


void Reset()
{
  acc_bias_x = 0.f;
  acc_bias_y = 0.f;
  acc_bias_z = 0.f;

  gyro_bias_x = 0.f;
  gyro_bias_y = 0.f;
  gyro_bias_z = 0.f;

  prev_micros = 0UL;
}

void Button()
{
  M5.update();
  if (M5.BtnA.wasPressed())
  {
    state--;
    M5.Lcd.clear();
  }

  if (M5.BtnC.wasPressed())
  {
    state++;
    M5.Lcd.clear();
  }
}


MatrixXd calc_u(float gx, float gy, float gz, double dt)
{
  MatrixXd u(3,1);
  u << gx * dt,
       gy * dt,
       gz * dt;
  return u;
}

MatrixXd calc_z(float ax, float ay, float az)
{
  MatrixXd z(2,1);
  double roll = RAD2DEG(atan2(ay, az));
  double pitch = RAD2DEG(-atan2(ax, sqrt(pow(ay,2) + pow(az,2))));
  z << roll,
       pitch;
  return z;
}

void drawTrapezoid(int x0, int y0, int x1, int y1, int x2, int y2, int x3, int y3, uint32_t color) {
  sprite.fillTriangle(x0, y0, x1, y1, x2, y2, color);
  sprite.fillTriangle(x1, y1, x2, y2, x3, y3, color);
}

void Main()
{
  // kalman filter
  KalmanFilter kf;

  MatrixXd x(3, 1);
  MatrixXd P(3, 3);

  prev_micros = 0;
  auto dt = (micros() - prev_micros) / 1000000.F;
  prev_micros = micros();
  x << 0, 0, 0;
  P << 0.0174 * pow(dt, 2), 0, 0,
       0, 0.0174 * pow(dt, 2), 0,
       0, 0, 0.0174 * pow(dt, 2);
  
  
  for (;;) {
    M5.update();

    float gx, gy, gz;
    float ax, ay, az;
    M5.IMU.getGyroData(&gy, &gz, &gx);
    M5.IMU.getAccelData(&ay, &az, &ax);


    gx -= gyro_bias_x;
    gy -= gyro_bias_y;
    gz -= gyro_bias_z;

    ax -= acc_bias_x;
    ay -= acc_bias_y;
    az -= acc_bias_z;


    // M5.Lcd.setCursor(20, 140);
    // M5.Lcd.printf("gyro");
    // M5.Lcd.setCursor(20, 160);
    // M5.Lcd.printf("%+5.2f, %+5.2f, %+5.2f", gx, gy, gz);

    // M5.Lcd.setCursor(20, 180);
    // M5.Lcd.printf("acc");
    // M5.Lcd.setCursor(30, 200);
    // M5.Lcd.printf("%+5.2f, %+5.2f, %+5.2f", ax, ay, az);

    auto dt_micros = micros() - prev_micros;
    prev_micros = micros();
    double dt = dt_micros / 1000000.F;

    MatrixXd u = calc_u(gx, gy, gz, dt);
    MatrixXd z = calc_z(ax, ay, az);

    MatrixXd R(2,2); 
    R << 1.0 * pow(dt, 2), 0,
         0, 1.0 * pow(dt, 2);
    MatrixXd Q(3,3);
    Q << 0.0174 * pow(dt, 2), 0, 0,
         0, 0.0174 * pow(dt, 2), 0,
         0, 0, 0.0174 * pow(dt, 2); 
    Q *= 10;

    auto t = kf.update(x, u, z, P, R, Q);
    x = get<0>(t);
    P = get<1>(t);
    // M5.Lcd.drawString("Angle", 100, 40);
    // M5.Lcd.setCursor(40, 80);
    // M5.Lcd.printf("%05.2f, %05.2f, %05.2f", x(0,0), x(1,0), x(2,0));
    // M5.Lcd.printf("%05.2f, %05.2f, %05.2f", u(0,0), u(1,0), u(2,0));


    // 水平線を引く
    double theta = x(0,0);
    
    double tan_theta = tan(DEG2RAD(theta));
    int start_pos_x = 0;
    int start_pos_y = 0;
    int end_pos_x = 0;
    int end_pos_y = 0;

    if (atan(-0.75) < theta || theta < atan(0.75)) {
      start_pos_y = 120 - 160 * tan_theta;
      end_pos_y = 120 + 160 * tan_theta;
      end_pos_x = 320;
    } else {
      start_pos_x = 120 - 160 / tan_theta;
      end_pos_x = 120 + 160 / tan_theta;
      end_pos_y = 240;
    }
    sprite.fillScreen(WHITE);
    sprite.drawLine(start_pos_x, start_pos_y, end_pos_x, end_pos_y, RED);
    
    sprite.pushSprite(0, 0);
  }
}


void setup()
{
  M5.begin(true);
  
  // IMU setup
  M5.IMU.Init();

  // LCD setup
  M5.Lcd.init();
  M5.Lcd.setCursor(20, 60);
  M5.Lcd.setTextSize(2);

  sprite.setColorDepth(8);
  sprite.setTextSize(2);
  sprite.createSprite(M5.Lcd.width(), M5.Lcd.height());
  
  delay(10);
}



void loop()
{
  Button();

  switch (state) {
  case 0:
    M5.Lcd.setCursor(140, 120);
    M5.Lcd.printf("Main");
    if (M5.BtnB.wasPressed()) Main();
    break;

  case 1:
    M5.Lcd.setCursor(90, 120);
    M5.Lcd.printf("Calibration");

    M5.Lcd.setCursor(40, 140);
    M5.Lcd.printf("gyro bias");
    M5.Lcd.setCursor(50, 160);
    M5.Lcd.printf("%.2f, %.2f, %.2f", gyro_bias_x, gyro_bias_y, gyro_bias_z);

    M5.Lcd.setCursor(40, 180);
    M5.Lcd.printf("acc bias");
    M5.Lcd.setCursor(50, 200);
    M5.Lcd.printf("%.2f, %.2f, %.2f", acc_bias_x, acc_bias_y, acc_bias_z);

    if (M5.BtnB.wasPressed()) Calibration();
    break;

  case 2:
    M5.Lcd.setCursor(120, 120);
    M5.Lcd.printf("Reset");
    if (M5.BtnB.wasPressed()) {
      Reset();
      M5.Lcd.drawString("Reset Completely", 80, 150);
    }
    break;

  default:
    state = 0;
    break;
  }
}