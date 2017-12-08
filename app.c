/**
 * TOPPERS/EV3　サンプル　「黒い線に沿って進む」.
 * 動作説明　　白と黒の境界線に沿って進む。
 *           カラーセンサの出力の目標値を白と黒の中間値としている  
 */

#include "ev3api.h"
#include "app.h"

/**
 * ポートの接続対応
 * USonic sensor: Port 1
 * Color sensor_Red: Port 2
 * Color sensor: Port 3
 * Left  motor:  Port B
 * Right motor:  Port C
 */
const int u_sonic_sensor = EV3_PORT_1;
const int color_sensor_red = EV3_PORT_2;
const int color_sensor = EV3_PORT_3;
const int left_motor = EV3_PORT_B;
const int right_motor = EV3_PORT_C;

//const int left_arm = EV3_PORT_A;
//const int right_arm = EV3_PORT_D;


//メインタスク
void main_task(intptr_t unused) {
  //モーターポートを設定 
  ev3_motor_config(left_motor, LARGE_MOTOR);
  ev3_motor_config(right_motor, LARGE_MOTOR);
  //ev3_motor_config(left_arm,LARGE_MOTOR);
  //ev3_motor_config(right_arm,LARGE_MOTOR);
  
  
  //センサーポートを設定
  ev3_sensor_config(color_sensor_red, COLOR_SENSOR);
  ev3_sensor_config(color_sensor, COLOR_SENSOR);
  ev3_sensor_config(u_sonic_sensor, ULTRASONIC_SENSOR);

  //変数宣言
  const int target_val = 55;  //明るさの目標値
  const int power = 100;       //モーターパワーj
  int steer = 0;              //ハンドル操作量

  const double DELTA_T = 0.001;  //処理周期（msec）
  const double KP = 2.0;     //比例項
  const double KI = 0.05;      //積分項
  const double KD = 0.0;      //微分項
    
  double diff[2] = {0};
  double integral = 0;

  double p, i, d;

	int cntred	=	0;
	int isred_old	=	0;
	int isred	=	0;
	int cntpid	=	0;

  const int thr_dist1 = 10;
  const int thr_dist2 = 100;
  int dist = 0;
	
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

    //距離情報の取得
    dist = ev3_ultrasonic_sensor_get_distance(u_sonic_sensor);
    //距離取得エラーの場合無視する
    if(dist == 0) continue;

    //壁が近いか判断する
    if(thr_dist2 < dist){
      steer = 55;
    }
    else if(thr_dist1 > dist){
      steer = 0;
    }
    //モータ操作量を更新
    ev3_motor_steer(left_motor, right_motor, power, steer);

    //明るさとハンドル操作量を表示
    char msg[256]= {0};
//   sprintf(msg, "reflect_val:%03d steer:%03d", sensor_val, steer);

	//PID�������܂������J�E���g�A�b�v
	cntpid	+=1;
	
	
	if( cntpid==40){
	
		cntpid = 0;
		const int RED = COLOR_RED;  //���C���̐F��F������A����́u�ԁv
		int color =  ev3_color_sensor_get_color(color_sensor_red);


		// 1�O�̃J���[���茋�ʂ�ێ����Ă���
		isred_old = isred;
		
		// �J���[���茋��
		if( color == RED) {
			isred = 1;
		}
		else {
			isred = 0;
		}

		// �J���[���茋�ʂ�0����1�ɕω�������J�E���g�A�b�v
		if( (isred_old==0) && (isred==1) ) {
			cntred += 1;
			
		//	if(cntred == 5){
		//	 ev3_motor_steer(left_motor, right_motor, 0 , 0);
		//	 ev3_motor_steer(left_arm, right_arm, 30 , 0);
		//	}
		}
	}



	//PID�̃J�E���g����\��
	char msg3[256]= {0};
	char msg2[256]= {0};

	
    sprintf(msg, "sensor:%03d err:%04.3f", sensor_val, diff[1]);
    ev3_lcd_draw_string(msg,0,20);
    sprintf(msg, "p:%03.3f i:%03.3f d:%03.3f", p, i, d);
    ev3_lcd_draw_string(msg,0,40);
    sprintf(msg, "steer:%03d  dist:%03d", steer, dist);
    ev3_lcd_draw_string(msg,0,60);
	sprintf(msg2, "redcount:%03d ",cntred);
	ev3_lcd_draw_string(msg2,0,80);
	sprintf(msg3, "pidcount:%03d ",cntpid);
	ev3_lcd_draw_string(msg3,0,100);
	
    tslp_tsk(DELTA_T*1000);
  }

  
  
  return;
}
