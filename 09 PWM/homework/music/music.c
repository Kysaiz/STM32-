#include "music.h"
#include "delay.h"
#include "beep.h"
#include "alphabet.h"

void beep_music(unsigned char *tone,unsigned char *time,unsigned int size)
{
  //计算音符表数组长度
  uint16_t length = 0,i = 0;
  length = size/sizeof(tone[0]);
  for(i = 0;i<=length;i++){
    if(TONE[tone[i]] == 0){
      tim13_set_duty(0);
      delay_ms(500);
    }
    else{
        tim13_set_freq(TONE[tone[i]]);
        tim13_set_duty(12);
        delay_ms(time[i]*5);
    } 
  }
  //播放完毕后关闭蜂鸣器
  tim13_set_duty(0);
  delay_ms(2000);
}

