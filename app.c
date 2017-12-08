/**
 * TOPPERS/EV3ã€€ã‚µãƒ³ãƒ—ãƒ«ã€€ã€Œé»’ã„ç·šã«æ²¿ã£ã¦é€²ã‚€ã€.
 * å‹•ä½œèª¬æ˜ã€€ã€€ç™½ã¨é»’ã®å¢ƒç•Œç·šã«æ²¿ã£ã¦é€²ã‚€ã€‚
 *           ã‚«ãƒ©ãƒ¼ã‚»ãƒ³ã‚µã®å‡ºåŠ›ã®ç›®æ¨™å€¤ã‚’ç™½ã¨é»’ã®ä¸­é–“å€¤ã¨ã—ã¦ã„ã‚‹  
 */

#include "ev3api.h"
#include "app.h"

/**
 * ãƒãƒ¼ãƒˆã®æ¥ç¶šå¯¾å¿œ
 * Color sensor_Red: Port 2
 * Color sensor: Port 3
 * USonic sensor: Port 4
 * Left  motor:  Port B
 * Right motor:  Port C
 */
const int color_sensor_red = EV3_PORT_2;
const int color_sensor = EV3_PORT_3;
const int u_sonic_sensor = EV3_PORT_4;
const int left_motor = EV3_PORT_B;
const int right_motor = EV3_PORT_C;

//const int left_arm = EV3_PORT_A;
//const int right_arm = EV3_PORT_D;


//ãƒ¡ã‚¤ãƒ³ã‚¿ã‚¹ã‚¯
void main_task(intptr_t unused) {
  //ãƒ¢ãƒ¼ã‚¿ãƒ¼ãƒãƒ¼ãƒˆã‚’è¨­å®š 
  ev3_motor_config(left_motor, LARGE_MOTOR);
  ev3_motor_config(right_motor, LARGE_MOTOR);
  //ev3_motor_config(left_arm,LARGE_MOTOR);
  //ev3_motor_config(right_arm,LARGE_MOTOR);
  
  
  //ã‚»ãƒ³ã‚µãƒ¼ãƒãƒ¼ãƒˆã‚’è¨­å®š
  ev3_sensor_config(color_sensor_red, COLOR_SENSOR);
  ev3_sensor_config(color_sensor, COLOR_SENSOR);
  ev3_sensor_config(u_sonic_sensor, ULTRASONIC_SENSOR);

  //å¤‰æ•°å®£è¨€
  const int target_val = 55;  //æ˜ã‚‹ã•ã®ç›®æ¨™å€¤
  const int power = 100;       //ãƒ¢ãƒ¼ã‚¿ãƒ¼ãƒ‘ãƒ¯ãƒ¼j
  int steer = 0;              //ãƒãƒ³ãƒ‰ãƒ«æ“ä½œé‡

  const double DELTA_T = 0.001;  //å‡¦ç†å‘¨æœŸï¼ˆmsecï¼‰
  const double KP = 2.0;     //æ¯”ä¾‹é …
  const double KI = 0.05;      //ç©åˆ†é …
  const double KD = 0.0;      //å¾®åˆ†é …
    
  double diff[2] = {0};
  double integral = 0;


  double p, i, d;

	int cntred	=	0;
	int isred_old	=	0;
	int isred	=	0;
	int cntpid	=	0;
	
	
  while(1){
    //æ˜ã‚‹ã•å–å¾—
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

    //ãƒ¢ãƒ¼ã‚¿æ“ä½œé‡ã‚’æ›´æ–°
    ev3_motor_steer(left_motor, right_motor, power, steer);

    //æ˜ã‚‹ã•ã¨ãƒãƒ³ãƒ‰ãƒ«æ“ä½œé‡ã‚’è¡¨ç¤º
    char msg[256]= {0};
//   sprintf(msg, "reflect_val:%03d steer:%03d", sensor_val, steer);

	//PID§Œä‚ğˆê‰ñ‚Ü‚í‚Á‚½‚çƒJƒEƒ“ƒgƒAƒbƒv
	cntpid	+=1;
	
	
	if( cntpid==40){
	
		cntpid = 0;
		const int RED = COLOR_RED;  //ƒ‰ƒCƒ“‚ÌF‚ğ”F¯‚·‚éA¡‰ñ‚ÍuÔv
		int color =  ev3_color_sensor_get_color(color_sensor_red);


		// 1ŒÂ‘O‚ÌƒJƒ‰[”»’èŒ‹‰Ê‚ğ•Û‚µ‚Ä‚¨‚­
		isred_old = isred;
		
		// ƒJƒ‰[”»’èŒ‹‰Ê
		if( color == RED) {
			isred = 1;
		}
		else {
			isred = 0;
		}

		// ƒJƒ‰[”»’èŒ‹‰Ê‚ª0‚©‚ç1‚É•Ï‰»‚µ‚½‚çƒJƒEƒ“ƒgƒAƒbƒv
		if( (isred_old==0) && (isred==1) ) {
			cntred += 1;
			
		//	if(cntred == 5){
		//	 ev3_motor_steer(left_motor, right_motor, 0 , 0);
		//	 ev3_motor_steer(left_arm, right_arm, 30 , 0);
		//	}
		}
	}



	//PID‚ÌƒJƒEƒ“ƒg”‚ğ•\¦
	char msg3[256]= {0};
	char msg2[256]= {0};

	
    sprintf(msg, "sensor:%03d err:%04.3f", sensor_val, diff[1]);
    ev3_lcd_draw_string(msg,0,20);
    sprintf(msg, "p:%03.3f i:%03.3f d:%03.3f", p, i, d);
    ev3_lcd_draw_string(msg,0,40);
    sprintf(msg, "steer:%03d", steer);
    ev3_lcd_draw_string(msg,0,60);
	sprintf(msg2, "redcount:%03d ",cntred);
	ev3_lcd_draw_string(msg2,0,80);
	sprintf(msg3, "pidcount:%03d ",cntpid);
	ev3_lcd_draw_string(msg3,0,100);
	
    tslp_tsk(DELTA_T*1000);
  }

  
  
  return;
}
