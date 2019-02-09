#include <math.h>
#include <SPI.h>
#include <Mirf.h>
#include <nRF24L01.h>
#include <MirfHardwareSpiDriver.h>

#define JUDGEMENT 4.0
#define VOLTAGE_ERROR 0.5



float Kp = 5, Ki = 0.005, Kd = 2; //PID factors
float error = 0, P = 0, I = 0, D = 0, PID_value = 0;
float previous_error = 0, previous_I = 0;
float SL = 0, SML = 0, SMR = 0, SR = 0;
static int initial_motor_speed = 50; //expected speed
int left_motor_speed = 0;
int right_motor_speed = 0;
int hou=0;
char value = 'a';

//引脚定义
const int IR_PIN2[] = {42, 43, 44, 45}; //infrared sensors
//   42  45
//   43  44
float  SBR=0, SBL=0, SFR=0, SFL=0;
float  SSL=0, SSR=0;
const int IR_PIN3[] = {46, 47};
const int IR_PIN[] = {38, 39, 40, 48}; //infrared sensors
int pin1=22;        //left front wheel
int pin2=23;
int pin3=26;        //left behind wheel
int pin4=27;
int pin31=30;        //right front wheel
int pin32=31;
int pin41=34;        //right behind wheel
int pin42=35;
int sn1=11;        //pwn
int sn2=7;
int sn3=5;
int sn4=3;
/*
 * ISO15693 RFID 读卡器，测试程序
 * 
 * 材料：
 * Arduino(Mega2560)、D-Think_M50 V1.2 RFID读写器
 * 
 * 原理：
 * 通过串口，按照一定的通信协议与RFID读写器通信，详见D-Think_M50 V1.2 RFID读写器技术资料
 * 
 * 接线：
 * VCC-5V  GND-GND  TX-RX  RX-TX
 * 
 * 本例：
 * RFID不断读取，并在串口打印卡里的前四个字节
 * 1、程序思路：
        按商家提供的DataSheet对串口命令的描述，把功能封装成函数。本例提供了“读取标签号”和“读取标签的前4个字节”的函数。测试效果是不断尝试读取标签，当模块读到标签时在串口显示前4字节，否则显示"no card"

   2、材料：
      Arduino Mega2560（或其它拥有多个串口的Arduino）、D-Think_M50模块、电子标签(可以使用图书馆的书)

   3、接线：
      D-Think_M50模块                Arduino Mega2560
            RX                                            TX1(18)
            TX                                            RX1(19)
           VCC                                            5V
           GND                                            GND
      其它引脚不接
   4、电子标签内的数据是以块为单位的，每块4字节。对于勇攀高峰比赛，只有第0数据块的前两个字节有意义。
      第0字节：用于指示物块号，代表标签所处的物块号为m。对于地上的标签，该字节为0x00。
      第1字节：用于指示地上的节点号，代表标签所处的节点号为n。对于物块上的标签，该字节为0x00。
      比如：0x00 0x02代表该标签处在第2个节点上。 0x05 0x00代表该标签处在第5个物块上。
 */
 
// Rfid 类，用于控制一个D-Think_M50模块
class Rfid{
private:
  // D-Think_M50模块接的串口
  HardwareSerial *ser;
   
  // 按ISO15693协议对即将发送的数据封装数据包
  // msg是需要封装的字节数组，msg_len是msg字节数组的长度
  // id是D-Think_M50模块的识别号
  // 封装后的字节数组放在buf指针的位置
  // 返回值是封装后字节数组的长度
  static uint8_t rfid_encode(uint16_t id, uint8_t *buf, uint8_t *msg, uint8_t msg_len){
    uint8_t *p, *tmp, check=0;
    p = buf;
    *p++ = 0xAA;
    *p++ = 0xBB;
    *p = msg_len + 3;
    if(*p==0xAA){
      *++p = 0;
    }
    p++;
    *p++ = 0;
    *p = id>>8;
    check ^= *p;
    if(*p==0xAA){
      *++p = 0;
    }
    p++;
    *p = id & 0x00ff;
    check ^= *p;
    if(*p==0xAA){
      *++p = 0;
    }
    p++;
    for(tmp=msg;tmp-msg<msg_len;tmp++,p++){
      *p = *tmp;
      check ^= *p;
      if(*p==0xAA){
        *p = 0;
      }
    }
    *p++ = check;
    return p-buf;
  }
 
  // 读取发来的字节数组，并按ISO15693协议解析出其有效载荷
  int rfid_recv(uint8_t *msg){
    int recv,cnt;
    for(cnt=0;cnt<9;cnt++){
      recv = ser->read();
      if(recv==-1){
        ser->flush();
        return -1;
      }
      if(recv==0xAA&&cnt!=0) ser->read();
    }
    for(cnt=0;(recv=ser->read())!=-1;cnt++){
      msg[cnt] = recv;
      if(recv==0xAA) ser->read();
    }
    ser->flush();
    return cnt-1;
  }
 
  // 初始化Rfid模块，设置其工作状态为ISO15693协议
  void rfid_init(){
    int len;
    uint8_t buf[0x50], msg[] = {0x08,0x01,'1'};
    ser->begin(19200);
    len = rfid_encode(0,buf,msg,sizeof(msg));
    ser->write(buf,len);
    delay(200);
    rfid_recv(buf);
  }
public:
 
  // 构造函数
  Rfid(HardwareSerial &ser_in){
    ser = &ser_in;
    rfid_init();
  }
  // 读取电子标签的标签号，若标签不存在或失败返回-1，成功返回0，读到的9个字节放在data指针处
  int readID(uint8_t *data){
    int len;
    uint8_t msg[0x20] = {0x01,0x10}, buf[0x30];
     
    len = rfid_encode(0,buf,msg,2);
    ser->write(buf,len);
   
    delay(35);
    len = rfid_recv(data);
 
    if(len!=9){
      return -1;
    }else{
      return 0;
    }
  }
 
  // 读取电子标签的前4个字节，若标签不存在或失败返回-1，成功返回0，读到的4个字节放在data指针处
  int read4bytes(uint8_t *data){
    int len;
    uint8_t msg[0x20] = {0x01,0x10}, buf[0x30];
     
    len = rfid_encode(0,buf,msg,2);
    ser->write(buf,len);
   
    delay(35);
    len = rfid_recv(msg+2);
    if(len!=9){
      return -1;
    }else{    
      msg[0] = 0x05; msg[11] = 0x10; msg[11] = 0; msg[12] = 1;
      len = rfid_encode(0,buf,msg,13);
      ser->write(buf,len);
      delay(45);
      len = rfid_recv(msg);
      if(len!=4){
        return -1;
      }else{
        memcpy(data,msg,4);
        return 0;
      }
    }
  }
 
};
 
Rfid *r,*rr;
 

void read_ir_values(void);
void calculate_pid(void);
void motor_control(void);
void qian(int a, int b);
void qian2(int a, int b);
void selfL();
void selfRR();
void houtui();
void selfR();
void fanga(int flag);
void zhua(int j,int b);
void zhuaqu(int a,int b);
void pagan();
void qian1(int b);
void left();
void right();
void shibie(int a,int b,int s[10]);
void shibie2(int b);
void fangb();
void selfL1();
void tongxin(char receive);
void stopz(){
  int i=0;
  analogWrite(sn1,i);
  analogWrite(sn2,i);
  analogWrite(sn3,i);
  analogWrite(sn4,i);
  digitalWrite(pin1,LOW);        //
  digitalWrite(pin2,LOW);
  digitalWrite(pin3,LOW);
  digitalWrite(pin4,LOW);  
  digitalWrite(pin32,LOW);
  digitalWrite(pin31,LOW);
  digitalWrite(pin42,LOW);
  digitalWrite(pin41,LOW);
  delay(150);
}


void setup()
{
  pinMode(pin1,OUTPUT);          //left wheels
  pinMode(pin2,OUTPUT);
  pinMode(pin3,OUTPUT);
  pinMode(pin4,OUTPUT);
  pinMode(pin31,OUTPUT);        //right wheels
  pinMode(pin32,OUTPUT);
  pinMode(pin42,OUTPUT);
  pinMode(pin41,OUTPUT);
  pinMode(sn1, OUTPUT);
  pinMode(sn2, OUTPUT);
  pinMode(sn3, OUTPUT);
  pinMode(sn4, OUTPUT);
  Mirf.cePin = 9;
  Mirf.csnPin = 10;
  
  Mirf.spi = &MirfHardwareSpi;
  Mirf.init();
  Mirf.setRADDR((byte *)"ABCDE"); //设置自己的地址（发送端地址），使用5个字符
  Mirf.payload = sizeof(value);
  Mirf.channel = 90;              //设置所用信道
  Mirf.config();
  for (int i = 0; i < 4; i++) {
    pinMode(IR_PIN[i], INPUT);
  }
  for (int i = 0; i < 4; i++) {
    pinMode(IR_PIN2[i], INPUT);
  }
  for (int i = 0; i < 2; i++) {
    pinMode(IR_PIN3[i], INPUT);
  }
  r = new Rfid(Serial3);
  rr = new Rfid(Serial2);
  
  Serial.begin(9600); 
}



void loop() {
  int a,b,s[10]={0},i,j,b1,j1;    //b1是记录物块4所在节点，j1是最后一个拿到的物块，j是第三个
  uint8_t data[4]={0};
  Mirf.setTADDR((byte *)"FGHIJ");
 // selfL1();

 // stopz();
 // delay(500000);
//  s[10] = {0,4,5,7,3,6,8,2,0,1};
/*  tongxin('a');
  delay(20000);
   tongxin('d');
   delay(20000);
    tongxin('l');
    delay(20000);
     tongxin('r');
     delay(20000);
      tongxin('f');
      delay(20000);
       tongxin('g');
       delay(20000);
        tongxin('p');
//  tongxin('r');
 // delay(15000);
  delay(50000);*/

  //ceshi
  
  a=0;
  b=1;
//  qian(a,b);
//  stopz();
//  delay(1000);
  for(b=1;b<=9; ){  
  shibie(a,b,s);
  stopz();
  delay(500);
  if(b==7) {b=b+1;}
  
 // selfL();
 if(b!=9){
   left();
   
 delay(800);
 }
    while(b!=9){
    left();
    //right();
   // r->read4bytes(data);
    //if(data[0]==0&&data[1]==(b+1)) break;
    SML = digitalRead(IR_PIN[1]);
    SMR = digitalRead(IR_PIN[2]);
        SL = digitalRead(IR_PIN[0]);
    if(SML&&SMR) break;
    if(SML&&SL) break;
    if(SML) break;
    if(SMR) break;
    }
    stopz();
    delay(1000);
    
    b++;
  }
  delay(1000);
  //--------------------------------------------------------------------
  fanga(0);
  delay(1000);  
  fangb();
  //10.5
  a=0;
  for(i=9;i>=0;i--){
    if(i==8) ;
    else {if(s[i]==6) {
              j=6;
              b=i;
              break;}
         else if(s[i]==5) 
         {
          j=5;
          b=i;
          break;}
      }
    }
    for( i=(b-1);i>=0;i--){
          if(i==8) ;
    else {if(s[i]==6) {
              j1=6;
              b1=i;
              break;}
         else if(s[i]==5) 
         {
          j1=5;
          b1=i;
          break;}
      }
      }
  
  a=0;           //ground 
  qian(a,b);
  while(1){
  read_ir_values();
  calculate_pid();
  motor_control();
  SSL = digitalRead(IR_PIN3[0]);
  SSR = digitalRead(IR_PIN3[1]); 
  if(SSL||SSR)
      break;                     //前进一小段 中心停止
} 
  stopz();
  delay(500);
  selfL();
  zhua(j,b);

  //准备去抓第四个了



  
  
  
  
  
  selfR();
  a=0;
  b=b1;
  qian(a,b);
  while(1){
  read_ir_values();
  calculate_pid();
  motor_control();
  SSL = digitalRead(IR_PIN3[0]);
  SSR = digitalRead(IR_PIN3[1]); 
  if(SSL||SSR)
      break;                     //前进一小段 中心停止
} 
  stopz();
  delay(500);
  selfL();
  zhua(j1,b1);
  selfL();
  qian(0,9);
//快走到了
  while(1){
  read_ir_values();
  calculate_pid();
  motor_control();
  SSL = digitalRead(IR_PIN3[0]);
  SSR = digitalRead(IR_PIN3[1]); 
  if(SSL&&SSR)
      break;                     //前进一小段 中心停止
} 
stopz();
delay(500);
  selfL();
  for(int y=1;y<=120;y++)
  {
  read_ir_values();
  calculate_pid();
  motor_control();
  }
    while(1){
  read_ir_values();
  calculate_pid();
  motor_control();
  SSL = digitalRead(IR_PIN3[0]);
  SSR = digitalRead(IR_PIN3[1]); 
  if(SSL||SSR)
      break;                     //前进一小段 中心停止
} 
stopz();
delay(500);
  selfL();
  houtui();
  

  delay(500000);
  //
  //fanga(1);
}
//结束。。。。。。。。。。。。。。。。。。。。。。。。。

//------------------------------------
void fangb(){
    uint8_t data[4]={0};
    left();
    delay(500);
      while(1){
    left();
    //right();
    SML = digitalRead(IR_PIN[1]);
    SMR = digitalRead(IR_PIN[2]);
    if(SML&&SMR) break; }  //识别到九
    stopz();
    delay(1000);
  qianjin(0,150);
  stopz();
  delay(500);
     tongxin('r');
     delay(2000);
 
  //通信--
  
   analogWrite(sn3, 50);
   analogWrite(sn4, 50);
   analogWrite(sn1, 50);
   analogWrite(sn2, 50);
  digitalWrite(pin2,HIGH);        //
  digitalWrite(pin1,LOW);
  digitalWrite(pin4,HIGH);
  digitalWrite(pin3,LOW);  
  digitalWrite(pin31,HIGH);
  digitalWrite(pin32,LOW);
  digitalWrite(pin41,HIGH);
  digitalWrite(pin42,LOW);
  delay(600);
  stopz();
  delay(500);
  selfL1();
  stopz();
    while(1){
  right();
   SML = digitalRead(IR_PIN[1]);
    SMR = digitalRead(IR_PIN[2]);
    if(SML || SMR) break;
  }
  stopz();
}



void tongxin(char value){
    //胜利alue = 1; 
   int  i=1; 
      Mirf.send((byte *)&value);        //直到发送，退出循环
      
  delay(1000);

            //can be optimized (better)
  /*{
    Serial.write(receive);
    delay(1000);
    break;
    if (Serial.available() > 0)
    {
      if ((char)Serial.read() == '6')
      {
        Serial.flush();
        break;
      }
    }
  }
  */
  }
void shibie(int a,int b,int s[10]){
  int i;
  uint8_t data[4]={0};
  i=0;
  while(1){
  read_ir_values();
  calculate_pid();
  motor_control();
  rr->read4bytes(data);
  if(data[0]!=0&&data[1]==0) break;
}
   s[b]=data[0];
   if(s[b]==7) zhuaqu(0,b);
   else if(s[b]==8) zhuaqu(1,b);
   else {
   stopz();
   hou=1;
   qian1(b);    
   hou=0;}
   
}

void shibie2(int b){
  int i,a;
  uint8_t data[4]={0};
  i=0;
  while(1){
  read_ir_values();
  calculate_pid();
  motor_control();
  rr->read4bytes(data);
  if(data[0]!=0&&data[1]==0) break;
}
  a=0;
  stopz();
  delay(3000);
   hou=1;
   qian1(b);    
   hou=0;
}

void fanga(int flag){
  int m;
   for(m=1;m<=150;m++){
  read_ir_values();
  calculate_pid();
  motor_control();
   }
   
  while(1){
  read_ir_values();
  calculate_pid();
  motor_control();
  SSL = digitalRead(IR_PIN3[0]);
  SSR = digitalRead(IR_PIN3[1]); 
  if(SSL||SSR)
      break;
} 
  stopz();
  selfL();
  stopz();
  delay(500);
    qianjin(0,150);
stopz();
delay(500);
  tongxin('l');
//  delay(1000);

 // -------------------tongxin  gai! 

    stopz();
    delay(2000);
    qianjin(1,100);
  }

void zhua(int j,int b){
  char r;
  if(j==6) r='f';
  else r='g';
  //通信
  tongxin(r);
  stopz();
  delay(1000);
  int i,a;
  uint8_t data[4]={0};
  i=0;
  while(1){
  read_ir_values();
  calculate_pid();
  motor_control();
  rr->read4bytes(data);
  if(data[0]!=0&&data[1]==0) break;
}
  a=0;
  stopz();
  delay(3000);
  hou=1;
  for(i=0;i<=120;i++){
      read_ir_values();
      calculate_pid();
      motor_control();}
  
  if(b!=9){
  while(1){
  read_ir_values();
  calculate_pid();
  motor_control();
  SSL = digitalRead(IR_PIN3[0]);
  SSR = digitalRead(IR_PIN3[1]); 
  if(SSL&&SSR)
      break;
}
  }
  else{
   while(1){
  read_ir_values();
  calculate_pid();
  motor_control();
  SSL = digitalRead(IR_PIN3[0]);
  SSR = digitalRead(IR_PIN3[1]); 
  if(SSR&&SSL)
      break;
   }
}
  hou=0;
  
  stopz();
  delay(500);
}

void pagan(){
  char letter='g';
  char receive;
  for( int i=0; i<3; i++){
    Serial3.println(letter);
    }
  while(1){
    receive = (char)Serial3.read();
    if(receive == '6') break;
    }
  }
void qianjin(int f,int i){
  int j;
  if(f==0){
    for(j=0;j<=i;j++){
      read_ir_values();
      
      calculate_pid();
      motor_control();}
  stopz();
  }
  else {
    hou=1;
    qian1(1);
    stopz();
    delay(500);
    hou=0;
    }
}  
void zhuaqu(int flag,int b){   //flag==0,抓第一个，flag==1,抓第二个
  int i;
  char receive = 'd';
  stopz();
  hou=1;
  
  if(flag==0) receive = 'a';

  for(i=0;i<=150;i++){
      read_ir_values();
      calculate_pid();
      motor_control();}
  hou=0;
  stopz();

  tongxin(receive);
  /*while(1)
  {
    Serial.write(receive);
    delay(1000);
   
    if (Serial.available() > 0)
    {
      if ((char)Serial.read() == '6')
      {
        Serial.flush();
        break;
      }
    }*/
  //}
  stopz();
  
  delay(500);
  shibie2(b);
}

  
void chufa(int a,int b){
  while(1){
  read_ir_values();
  calculate_pid();
  motor_control();

  float voltage = 0.0;
  voltage = analogRead(A0) * 5 / 1023;
  if(fabs(voltage - JUDGEMENT) < VOLTAGE_ERROR) 
  break;
}
   stopz();
   
  
}
void selfR(){
  int i;
  digitalWrite(pin2,HIGH);
  digitalWrite(pin1,LOW);
  digitalWrite(pin4,HIGH);
  digitalWrite(pin3,LOW);  
  digitalWrite(pin32,HIGH);
  digitalWrite(pin31,LOW);
  digitalWrite(pin42,HIGH);
  digitalWrite(pin41,LOW);  
  i=100;
  analogWrite(sn1,i);
  analogWrite(sn2,i);
  analogWrite(sn3,i);
  analogWrite(sn4,i);
  delay(1025);  
  stopz();
  delay(500);
  }                                    //向上看是顺时针
void selfL(){
    int i;
  digitalWrite(pin1,HIGH);
  digitalWrite(pin2,LOW);
  digitalWrite(pin3,HIGH);
  digitalWrite(pin4,LOW);  
  digitalWrite(pin31,HIGH);
  digitalWrite(pin32,LOW);
  digitalWrite(pin41,HIGH);
  digitalWrite(pin42,LOW);  
  i=100;
  analogWrite(sn1,i);
  analogWrite(sn2,i);
  analogWrite(sn3,i);
  analogWrite(sn4,i);
  delay(1050); 
  stopz();
  delay(500);
                              //逆时针转
  }
  void selfL1(){
    int i;
  digitalWrite(pin1,HIGH);
  digitalWrite(pin2,LOW);
  digitalWrite(pin3,HIGH);
  digitalWrite(pin4,LOW);  
  digitalWrite(pin31,HIGH);
  digitalWrite(pin32,LOW);
  digitalWrite(pin41,HIGH);
  digitalWrite(pin42,LOW);  
  i=100;
  analogWrite(sn1,i);
  analogWrite(sn2,i);
  analogWrite(sn3,i);
  analogWrite(sn4,i);
  delay(1800); 
  stopz();
  delay(500);
                              //逆时针转
  }
void selfLL(){
    int i;
  digitalWrite(pin1,HIGH);
  digitalWrite(pin2,LOW);
  digitalWrite(pin3,HIGH);
  digitalWrite(pin4,LOW);  
  digitalWrite(pin31,HIGH);
  digitalWrite(pin32,LOW);
  digitalWrite(pin41,HIGH);
  digitalWrite(pin42,LOW);  
  i=100;
  analogWrite(sn1,i);
  analogWrite(sn2,i);
  analogWrite(sn3,i);
  analogWrite(sn4,i);
  delay(800); 
  stopz();
  analogWrite(sn3, 50);
    analogWrite(sn4, 50);
    analogWrite(sn1, 50);
    analogWrite(sn2, 50);
  digitalWrite(pin2,HIGH);        //
  digitalWrite(pin1,LOW);
  digitalWrite(pin4,HIGH);
  digitalWrite(pin3,LOW);  
  digitalWrite(pin31,HIGH);
  digitalWrite(pin32,LOW);
  digitalWrite(pin41,HIGH);
  digitalWrite(pin42,LOW);
  delay(800);
  stopz();
  digitalWrite(pin1,HIGH);
  digitalWrite(pin2,LOW);
  digitalWrite(pin3,HIGH);
  digitalWrite(pin4,LOW);  
  digitalWrite(pin31,HIGH);
  digitalWrite(pin32,LOW);
  digitalWrite(pin41,HIGH);
  digitalWrite(pin42,LOW);  
  i=100;
  analogWrite(sn1,i);
  analogWrite(sn2,i);
  analogWrite(sn3,i);
  analogWrite(sn4,i);
  delay(450);
  stopz();
  analogWrite(sn3, 50);
    analogWrite(sn4, 50);
    analogWrite(sn1, 50);
    analogWrite(sn2, 50);
  digitalWrite(pin2,HIGH);        //
  digitalWrite(pin1,LOW);
  digitalWrite(pin4,HIGH);
  digitalWrite(pin3,LOW);  
  digitalWrite(pin31,HIGH);
  digitalWrite(pin32,LOW);
  digitalWrite(pin41,HIGH);
  digitalWrite(pin42,LOW);
  delay(400);
  stopz();                              //逆时针转
  }
void selfRR(){
    int i;
  digitalWrite(pin2,HIGH);
  digitalWrite(pin1,LOW);
  digitalWrite(pin4,HIGH);
  digitalWrite(pin3,LOW);  
  digitalWrite(pin32,HIGH);
  digitalWrite(pin31,LOW);
  digitalWrite(pin42,HIGH);
  digitalWrite(pin41,LOW);  
  i=100;
  analogWrite(sn1,i);
  analogWrite(sn2,i);
  analogWrite(sn3,i);
  analogWrite(sn4,i);
  delay(800); 
  stopz();
  analogWrite(sn3, 50);
    analogWrite(sn4, 50);
    analogWrite(sn1, 50);
    analogWrite(sn2, 50);
  digitalWrite(pin2,HIGH);        //
  digitalWrite(pin1,LOW);
  digitalWrite(pin4,HIGH);
  digitalWrite(pin3,LOW);  
  digitalWrite(pin31,HIGH);
  digitalWrite(pin32,LOW);
  digitalWrite(pin41,HIGH);
  digitalWrite(pin42,LOW);
  delay(600);
  stopz();
  digitalWrite(pin2,HIGH);
  digitalWrite(pin1,LOW);
  digitalWrite(pin4,HIGH);
  digitalWrite(pin3,LOW);  
  digitalWrite(pin32,HIGH);
  digitalWrite(pin31,LOW);
  digitalWrite(pin42,HIGH);
  digitalWrite(pin41,LOW);  
  i=100;
  analogWrite(sn1,i);
  analogWrite(sn2,i);
  analogWrite(sn3,i);
  analogWrite(sn4,i);
  delay(450);
  stopz();
  analogWrite(sn3, 50);
    analogWrite(sn4, 50);
    analogWrite(sn1, 50);
    analogWrite(sn2, 50);
  digitalWrite(pin2,HIGH);        //
  digitalWrite(pin1,LOW);
  digitalWrite(pin4,HIGH);
  digitalWrite(pin3,LOW);  
  digitalWrite(pin31,HIGH);
  digitalWrite(pin32,LOW);
  digitalWrite(pin41,HIGH);
  digitalWrite(pin42,LOW);
  delay(200);
  stopz();                              //逆时针转
  }
  
void qian2(int a,int b)    //rfid  rr
{
  uint8_t data[4];
  while(1){
  read_ir_values();
  calculate_pid();
  motor_control();
  rr->read4bytes(data);
  if(a==data[0]&&b==data[1]) break;
}
}
void qian(int a,int b){
  uint8_t data[4];
  while(1){
  read_ir_values();
  calculate_pid();
  motor_control();
  r->read4bytes(data);
  if(a==data[0]&&b==data[1]) break;
}  
}

void read_ir_values()
{
    SL = digitalRead(IR_PIN[0]);
    SML = digitalRead(IR_PIN[1]);
    SMR = digitalRead(IR_PIN[2]);
    SR = digitalRead(IR_PIN[3]);
    if (!SL && !SML && !SMR && !SR){
      if (error < 0) {
        error = -7;
      } 
      else {
        error = 7;
      }
    }
    else if(!SL && !SML && !SMR && SR) {
      error = -5; 
    }
    else if(!SL && !SML && SMR && SR) {
      error = -3; 
    }
    else if(!SL && !SML && SMR && !SR) {
      error = -1;
    }
    else if(!SL && SML && SMR) {
      error = 0;
    }
    else if(!SL && SML && !SMR && !SR) {
      error = 1;
    }
    else if((SL && SML && !SMR && !SR)||(SL && SML && SMR && SR)) {
      error = 3;
    }
    else if(SL && !SML && !SMR && !SR) {
      error = 5;
    }    
  }


void calculate_pid()
{
  P = error;
  I = I + error;
  D = error - previous_error;

  PID_value = (Kp * P) + (Ki * I) + (Kd * D);
  PID_value = constrain(PID_value, -100, 100);

  previous_error = error;
}

void motor_control()
{
  //Calculate the speed of each wheels
  left_motor_speed = initial_motor_speed - PID_value;
  right_motor_speed = initial_motor_speed + PID_value;

  constrain(left_motor_speed, 0, 255); //the speed is limited in 0 to 255
  constrain(right_motor_speed, 0, 255);
  motorsWrite(left_motor_speed, right_motor_speed);
}

void motorsWrite(int speedL, int speedR)
{

  if(hou==0){
      analogWrite(sn3, speedR);
    analogWrite(sn4, speedR);
    analogWrite(sn1, speedL);
    analogWrite(sn2, speedL);
  digitalWrite(pin1,HIGH);        //
  digitalWrite(pin2,LOW);
  digitalWrite(pin3,HIGH);
  digitalWrite(pin4,LOW);  
  digitalWrite(pin32,HIGH);
  digitalWrite(pin31,LOW);

  digitalWrite(pin42,HIGH);
  digitalWrite(pin41,LOW);  
  delay(10);}
  else {
    analogWrite(sn3, 50);
    analogWrite(sn4, 50);
    analogWrite(sn1, 50);
    analogWrite(sn2, 50);
  digitalWrite(pin2,HIGH);        //
  digitalWrite(pin1,LOW);
  digitalWrite(pin4,HIGH);
  digitalWrite(pin3,LOW);  
  digitalWrite(pin31,HIGH);
  digitalWrite(pin32,LOW);
  digitalWrite(pin41,HIGH);
  digitalWrite(pin42,LOW);  
  delay(10);}
}
void left(){
/*  int i=40;
  SFL = digitalRead(IR_PIN2[0]);
  SBL = digitalRead(IR_PIN2[1]);
  SBR = digitalRead(IR_PIN2[2]);
  SFR = digitalRead(IR_PIN2[3]);
  int error1 = 0, error2 = 0;
  if(SFL && SBL && SBR && SFR){
       error1 = 0;
       error2 = 0;
  }
  else if(SFL && !SBL && !SBR && SFR){
      error1 = i;
      error2 = i;
  }
  else if(!SFL && SBL && SBR && !SFR){
      error1 = -i;
      error2 = -i;      
  }
  else if(SFL && SBL && !SBR && SFR){
      error1 = 0;
      error2 = i;
  }
  else if(SFL && !SBL && SBR && SFR){
      error1 = i;
      error2 = 0;
  }
  else if(SFL && SBL && SBR && !SFR){
      error1 = 0;
      error2 = -i;
  }
  else if(!SFL && SBL && SBR && SFR){
      error1 = -i;
      error2 = 0;
  }
  else if(!SFL && SBL && !SBR && SFR){
      error1 = -i;
      error2 = i;
  }
  else if(SFL && !SBL && SBR && !SFR){
      error1 = i;
      error2 = -i;
  }
//  else {stopz();}*/
  int i;
  
  i=70;
//    int error1 = 0, error2 = 0;
 // LB = LB + error1;
 // LF = LF - error1;
 // RB = RB - error2;
 // RF = RF + error2;
  digitalWrite(pin2,HIGH);
  digitalWrite(pin1,LOW);
  digitalWrite(pin3,HIGH);
  digitalWrite(pin4,LOW);  
  digitalWrite(pin32,HIGH);
  digitalWrite(pin31,LOW);
  digitalWrite(pin41,HIGH);
  digitalWrite(pin42,LOW);  
  analogWrite(sn1,i-3);
  analogWrite(sn2,i+60);//+
  analogWrite(sn3,i);
  analogWrite(sn4,i);
  delay(10);
}
void right(){
/*  SFL = digitalRead(IR_PIN[0]);
  SBL = digitalRead(IR_PIN[1]);
  SBR = digitalRead(IR_PIN[2]);
  SFR = digitalRead(IR_PIN[3]);
  int error1 = 0, error2 = 0;
  int i = 20;
  if(SFL && SBL && SBR && SFR){
       error1 = 0;
       error2 = 0;
  }
  else if(SFL && !SBL && !SBR && SFR){
      error1 = i;
      error2 = i;
  }
  else if(!SFL && SBL && SBR && !SFR){
      error1 = -i;
      error2 = -i;      
  }
  else if(SFL && SBL && !SBR && SFR){
      error1 = 0;
      error2 = i;
  }
  else if(SFL && !SBL && SBR && SFR){
      error1 = i;
      error2 = 0;
  }
  else if(SFL && SBL && SBR && !SFR){
      error1 = 0;
      error2 = -i;
  }
  else if(!SFL && SBL && SBR && SFR){
      error1 = -i;
      error2 = 0;
  }
  else if(!SFL && SBL && !SBR && SFR){
      error1 = -i;
      error2 = i;
  }
  else if(SFL && !SBL && SBR && !SFR){
      error1 = i;
      error2 = -i;
  }
//  else {stopz();}*/
int error1=0,error2=0;
  int LF = 80, LB = 80, RF = 80, RB = 80;
  LB = LB + error1;
  LF = LF - error1;
  RB = RB - error2;
  RF = RF + error2;
  digitalWrite(pin2,LOW);
  digitalWrite(pin1,HIGH);
  digitalWrite(pin3,LOW);
  digitalWrite(pin4,HIGH);  
  digitalWrite(pin32,LOW);
  digitalWrite(pin31,HIGH);
  digitalWrite(pin41,LOW);
  digitalWrite(pin42,HIGH);  
  analogWrite(sn1,RB);
  analogWrite(sn2,RF);
  analogWrite(sn3,LB);
  analogWrite(sn4,LF);
  delay(10);
}
void qian1(int b){
  if(b!=9){
  while(1){
  read_ir_values();
  calculate_pid();
  motor_control();
  SBL = digitalRead(IR_PIN2[1]);
  SBR = digitalRead(IR_PIN2[2]);
  SFL = digitalRead(IR_PIN2[0]);
  SFR = digitalRead(IR_PIN2[3]);
  if(SBL||SBR) break;
}  
  }
  else{
      while(1){
  read_ir_values();
  calculate_pid();
  motor_control();
  SBL = digitalRead(IR_PIN2[1]);
  SBR = digitalRead(IR_PIN2[2]);
  SFL = digitalRead(IR_PIN2[0]);
  SFR = digitalRead(IR_PIN2[3]);
  if(SBR) break;
}  

  }
}
void houtui(){
  int i,sth=1;   
  i=50;   //sth is 触发开关闭合的时候是0或1  自己挑
    analogWrite(sn3, i);
    analogWrite(sn4, i);
    analogWrite(sn1, i);
    analogWrite(sn2, i);
  digitalWrite(pin2,HIGH);        //
  digitalWrite(pin1,LOW);
  digitalWrite(pin4,HIGH);
  digitalWrite(pin3,LOW);  
  digitalWrite(pin31,HIGH);
  digitalWrite(pin32,LOW);
  digitalWrite(pin41,HIGH);
  digitalWrite(pin42,LOW); 
  delay(3500);
/*  hou=1;
  while(1){
    if(sth==0) break;
      read_ir_values();
  calculate_pid();
  motor_control();
  

    
    }*/
    stopz();
    delay(500);
  selfRR();
  tongxin('p');   
  stopz();
  delay(500000);
  
  
  }

  //perfect    

