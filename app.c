/**
 * TOPPERS/EV3　サンプル　「黒い線に沿って進む」.
 * 動作説明　　白と黒の境界線に沿って進む。
 *           カラーセンサの出力の目標値を白と黒の中間値としている  
 */

#include "ev3api.h"
#include "app.h"

/**
 * ポートの接続対応
 * Touch sensor: Port 2
 * Color sensor: Port 3
 * USonic sensor: Port 4
 * Left  motor:  Port B
 * Right motor:  Port C
 */

const int touch_sensor = EV3_PORT_2;
const int color_sensor = EV3_PORT_3;
const int u_sonic_sensor = EV3_PORT_4;
const int left_motor = EV3_PORT_B;
const int right_motor = EV3_PORT_C;

//メインタスク
void main_task(intptr_t unused) {
  //モーターポートを設定 
  ev3_motor_config(left_motor, LARGE_MOTOR);
  ev3_motor_config(right_motor, LARGE_MOTOR);

  //センサーポートを設定
  ev3_sensor_config(touch_sensor, TOUCH_SENSOR);
  ev3_sensor_config(color_sensor, COLOR_SENSOR);
  ev3_sensor_config(u_sonic_sensor, ULTRASONIC_SENSOR);

  //変数宣言
  const int target_val = 20;  //明るさの目標値
  const int power = 60;       //モーターパワーj
  int steer = 0;              //ハンドル操作量

  const double DELTA_T = 0.001;  //処理周期（msec）
  const double KP = 3.00;     //比例項
  const double KI = 0.0;      //積分項
  const double KD = 0.0;      //微分項

  double diff[2] = {0};
  double integral = 0;


  double p, i, d;

  while(1){
    //明るさ取得
    int sensor_val = ev3_color_sensor_get_reflect(color_sensor);

    diff[0] = diff[1];
    diff[1] = sensor_val - target_val;
    integral += ( diff[1] + diff[0] ) / 2.0 * DELTA_T;

    p = KP * diff[1];
    i = KI * integral;
    d = KD * ( diff[1] - diff[0] ) / DELTA_T;

    steer = p + i + d;
    if ( steer < -100 ) {
      steer = -100;
    }
    else if ( steer > 100 ) {
      steer = 100;
    }

    //モータ操作量を更新
    ev3_motor_steer(left_motor, right_motor, power, steer);

    //明るさとハンドル操作量を表示
    char msg[256]= {0};
//    sprintf(msg, "reflect_val:%03d steer:%03d", sensor_val, steer);
    sprintf(msg, "sensor:%03d err:%04.3f", sensor_val, diff[1]);
    ev3_lcd_draw_string(msg,0,20);
    sprintf(msg, "p:%03.3f i:%03.3f d:%03.3f", p, i, d);
    ev3_lcd_draw_string(msg,0,40);
    sprintf(msg, "steer:%03d", steer);
    ev3_lcd_draw_string(msg,0,60);

    tslp_tsk(DELTA_T*1000);
  }

  return;
}
