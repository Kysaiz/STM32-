#include "stm32f4xx.h"
#include "key.h"
#include "led.h"
#include "bit.h"
#include "it.h"
#include "beep.h"
#include "alphabet.h"
#include "delay.h"
#include "music.h"

int main(){  
//  //�������������鳤��
  beep_pwm_init(100);
  tim13_set_duty(0);
  while(1){
    //��ֻ�ϻ�
    beep_music(music1_tone,music1_time,sizeof(music1_tone));
  }
}

