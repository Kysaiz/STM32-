#ifndef __MUSIC_H
#define __MUSIC_H

//��ֻ�ϻ� ������
static unsigned char music1_tone[] = {
  1,2,3,1,              1,2,3,1,            3,4,5,0,            3,4,5,0,
  5,6, 5,4, 3,1,        5,6, 5,4, 3,1,      1,12,1,0,           1,12,1,0
  };

  
  //��ֻ�ϻ� ���ı� 100��ʾһ������
static unsigned char music1_time[] = {
  100,100,100,100,              100,100,100,100,            100,100,100,100,            100,100,100,100,
  50,50, 50,50, 100,100,        50,50, 50,50, 100,100,      100,100,100,100,            100,100,100,100
  };

void beep_music(unsigned char *tone,unsigned char *time,unsigned int size);

#endif
  


  