
//version1.0 :gripper & pole-climbing are included
//version2.0 :communication part is added
//version3.0 :Servo.h is included

#include <math.h>
#include <Servo.h>

#include <SPI.h>
#include <Mirf.h>
#include <nRF24L01.h>
#include <MirfHardwareSpiDriver.h>
 
#define NUMBER_OF_PERIOD 10
#define TRIGGER_ERROR 0.5

// Swon555rrr
#define pinnode1 2
#define pinnode2 3
#define pinnode3 4
#define pinnode4 5
#define pinnode5 6

Servo servo1;
Servo servo2;
Servo servo3;
Servo servo4;
Servo servo5;

// Kai   
int Down_Data[9][2] = {{0, 1000},{1, 1000},{2, 1000},{3, 1000},{4, 1000},{5, 1000},{6, 1000},{7, 1000},{8, 1000}};
int Vertical_Data[9][2] = {{0, 2000},{1, 2000},{2, 2000},{3, 2000},{4, 2000},{5, 2000},{6, 2000},{7, 2000},{8, 2000}};
int Up_Data[9][2] = {{0, 1000},{1, 1000},{2, 1000},{3, 1000},{4, 1000},{5, 1000},{6, 1000},{7, 1000},{8, 1000}};

const int pinDown = 14;
const int pinVertical = 15;
const int pinUp = 16;
//const int pinTrigger = A0;

char choice='0';

void initiate()
{
  digitalWrite(pinDown, LOW);
  digitalWrite(pinUp, LOW);
    
  delay(2000);
}

void foreach(int number)
{
  digitalWrite(pinUp, HIGH);
  delay(Up_Data[number][1]);//上爪打开
  
  digitalWrite(pinVertical, LOW);
  delay(Vertical_Data[number][1]);//竖直上顶

  digitalWrite(pinUp, LOW);
  delay(Up_Data[number][1]);//上爪夹住

  digitalWrite(pinDown, HIGH);
  delay(Down_Data[number][1]);//下爪打开

  digitalWrite(pinVertical, HIGH);
  delay(Vertical_Data[number][1]);//竖直上拉

  digitalWrite(pinDown, LOW);
  delay(Down_Data[number][1]);//下爪夹住
}


bool trigger(float judgement)
{
  if (fabs(analogRead(A0) * 5 / 1023 - judgement) < TRIGGER_ERROR)
    return true;
  else
    return false;
}


void finish()
{
  digitalWrite(pinUp, HIGH);
  delay(50);

  digitalWrite(pinVertical, LOW);
  while(!trigger(4))
  {
    digitalWrite(pinUp, LOW);
    delay(50);
  }
}


//========================================================================================================================
bool SEmove1(int pinNode,int degree){     //controlling ONLY No.1 steering engine
    int i,t;                          //SE means the Steering Engine
//  if(degree<0 || degree>180){
//      return (false);
//  }
      t= 750 + 20*degree/3;                            //(750,0), (1950,180)
      servo1.writeMicroseconds(t);

      return (true);
}
bool SEmove2(int pinNode,int degree){     //controlling ONLY No.2 steering engine
    int t,i;                          //SE means the Steering Engine
    t= 780 + 20*degree/3;           //(780,0),(1980,180)
    servo2.writeMicroseconds(t);
    return true;
}
bool SEmove3(int pinNode,int degree){     //controlling ONLY No.3 steering engine
    int t,i;                          //SE means the Steering Engine
//  if(degree<0 || degree>180){
//      return (false);
//  }
    t= 560 + 97*degree/9;           //(560,0),(1530,180)
    servo3.writeMicroseconds(t);
    return true;
}
/*void throaway(int pinNode){
  digitalWrite(pinNode,HIGH);
  shake(pinnode1,180);
  delay(3000);
  digitalWrite(pinNode,LOW);
  return(0);
  }*/
//---------------------------------------------------------------------------------------------// below are packaged
//General�
//grasp(),hold(),pudding();

int grasp(int pinNode1,int pinNode2, int pinNode3,int graspnumber){               //if graspnumber is 0,grasp first object and then return 1;
    if(graspnumber== 0){                                                         //if graspnumber is 1,grasp first object and then return 2;
        SEmove1(pinNode1,175);
        SEmove2(pinNode2,0);        //  !!!!!!if you wants to grasp objects on the back of the robot    
        SEmove3(pinNode3,0);
        delay(5000);    // NEEDS ADJUSTED
        servo4.write(90);//          shouzhua de dushu
        return(1);
    }// if no OBJECT is held before, then the gripper hold one object.
    else if(graspnumber== 1){       
        SEmove1(pinNode1,175);       //  !!!!!!if you wants to grasp objects on the back of the robot,then 180 is settled
        SEmove2(pinNode2,0);                       
        SEmove3(pinNode3,90);
        delay(5000);   // NEEDS ADJUSTED
        servo5.write(100);
        return(2);
    }// if one OBJECT is held before, then the gripper hold two objects.
}//suction force is always provided in this function
//if graspnumber=0---se1==180,se2=180,se3=0
//if graspnumber=1---se1==180,se2=180,se3=90
/*int shake(int pinNode1,int degree){
    if(degree>10){
    for(int i=0;i<15;i++){
    SEmove1(pinNode1,degree+10);
    delay(200);
    SEmove1(pinNode1,degree);
    delay(200);
    SEmove1(pinNode1,degree-10);
    delay(200);
    }
  }
  else{
    for(int i=0;i<15;i++){
      SEmove1(pinNode1,degree+30);
      delay(300);
      SEmove1(pinNode1,degree);
      delay(300);
      }
    }
  return(true);
}
*/
int hold(int pinNode1,int pinNode2, int pinNode3){
    SEmove1(pinNode1,90);
    SEmove2(pinNode2,0);
    SEmove3(pinNode3,0);
    delay(1000);
}//suction force is always provided in this function
int Lpudding(int pinNode1,int pinNode2, int pinNode3,int graspnumber){
    if(graspnumber== 0){
      SEmove2(pinNode2,0);
      //delay(1000);
      SEmove1(pinNode1,180);
      SEmove3(pinNode3,0);
      delay(1000);
      servo4.write(0);//zhangkai
      delay(2000);
      return(1);
    }
    if(graspnumber== 1){
      SEmove2(pinNode2,0);
      //delay(1000); // hold enter
      SEmove1(pinNode1,180);
      SEmove3(pinNode3,90);
      delay(1000);
      servo5.write(0);
      delay(2000);
      return(1);
    }   
}
int Hpudding(int pinNode1,int pinNode2, int pinNode3){    //first place the ''opposite'' one, then you place the ''stick'' one 
    SEmove2(pinNode2,0);
    delay(500);
    SEmove1(pinNode1,180);
    SEmove3(pinNode3,90);
    delay(1000);
    servo5.write(0);
    delay(1000);
    
    SEmove2(pinNode2,180);
    delay(500);
    SEmove1(pinNode1,0);
    SEmove3(pinNode3,0);
    delay(1000);
    servo4.write(0);
    delay(1000);
}

//===================================================================================================//浠ヤ笂灏佽


int counter = 0;
char flag;

void setup() 
{
  pinMode(pinDown, OUTPUT);
  pinMode(pinVertical, OUTPUT);
  pinMode(pinUp, OUTPUT);
  
  digitalWrite(pinDown, HIGH);
  digitalWrite(pinUp, HIGH);
  digitalWrite(pinVertical, HIGH);

  pinMode(2,OUTPUT);
  pinMode(3,OUTPUT);
  pinMode(4,OUTPUT);
  pinMode(5,OUTPUT);
  pinMode(6,OUTPUT);
  servo1.attach(2);
  servo2.attach(3);
  servo3.attach(4);
  servo4.attach(5);
  servo5.attach(6);
//Swon

  Mirf.cePin = 9;
  Mirf.csnPin = 10;
  
  Mirf.spi = &MirfHardwareSpi;
  Mirf.init();
 
  Mirf.setRADDR((byte *)"FGHIJ"); //设置自己的地址（接收端地址），使用5个字符
  Mirf.payload = sizeof(choice);   
  Mirf.channel = 90;   //设置使用的信道
  Mirf.config(); 

  hold(pinnode1,pinnode2,pinnode3);
  servo4.write(0);
  servo5.write(0);
  delay(3000);

}

//停到需要爪的地方发信号！ 手爪下降 delay2000 抓！
//停到需要放的地方再发信号！delay(1000) 放
void loop() 
{

  while (1)
  {
    if(Mirf.dataReady()) {  //当接收到程序，便从串口输出接收到的数据
    Mirf.getData((byte *) &choice);
    }

    flag = choice;
    
      if (choice == 'a')
      {
      grasp(pinnode1,pinnode2,pinnode3,0);       
      delay(3000);
      SEmove1(pinnode1,90);
      delay(2000);
      break;
      }
      
      if (choice == 'd')
      {
      grasp(pinnode1,pinnode2,pinnode3,1);       
      delay(3000);
      SEmove1(pinnode1,90);
      delay(2000);
      break;
      }    
  }//1
  
    hold(pinnode1,pinnode2,pinnode3);

    while (1)
  {
    if(Mirf.dataReady()) {  //当接收到程序，便从串口输出接收到的数据
    Mirf.getData((byte *) &choice);
    }
    
    if (choice != flag)
    {
    
      if (choice == 'a' )
      {
      grasp(pinnode1,pinnode2,pinnode3,0);       
      delay(3000);
      SEmove1(pinnode1,90);
      delay(2000);
      break;
      }
      
      if (choice == 'd')
      {
      grasp(pinnode1,pinnode2,pinnode3,1);       
      delay(3000);
      SEmove1(pinnode1,90);     
      delay(2000); 
      break;
      }
    }

  }//2
  
    hold(pinnode1,pinnode2,pinnode3);

    while (1)
    {
      if(Mirf.dataReady()) {  //当接收到程序，便从串口输出接收到的数据
      Mirf.getData((byte *) &choice);
      }
      if (choice == 'l')
      {
        Lpudding(pinnode1,pinnode2,pinnode3,0);
        break;
      }
    }//pudding 1
    
    hold(pinnode1,pinnode2,pinnode3);

    while (1)
    {
      if(Mirf.dataReady()) {  //当接收到程序，便从串口输出接收到的数据
      Mirf.getData((byte *) &choice);
      }
      if (choice == 'r')
      {
        Lpudding(pinnode1,pinnode2,pinnode3,1);
        break;
      }
    }//pudding 2
    
    hold(pinnode1,pinnode2,pinnode3);
    

    while (1)
    {
      if(Mirf.dataReady()) {  //当接收到程序，便从串口输出接收到的数据
      Mirf.getData((byte *) &choice);
      }
      flag = choice;
      if (choice == '3')
      {
        grasp(pinnode1,pinnode2,pinnode3,0);       
        delay(3000);
        SEmove1(pinnode1,90);
        delay(2000);
        break;
      }
      if (choice == '4')
      { 
      grasp(pinnode1,pinnode2,pinnode3,1);       
      delay(3000);
      SEmove1(pinnode1,90);
      delay(2000);
      break;
      }      
    }//grasp 3
    
    hold(pinnode1,pinnode2,pinnode3);
    
    while (1)
    {
      if(Mirf.dataReady()) {  //当接收到程序，便从串口输出接收到的数据
      Mirf.getData((byte *) &choice);
      }    
      if (choice != flag)
      {   
        if (choice == '3' )
        {
        grasp(pinnode1,pinnode2,pinnode3,0);       
        delay(3000);
        SEmove1(pinnode1,90);
        delay(2000);
        break;
        }
        if (choice == '4')
        {
        grasp(pinnode1,pinnode2,pinnode3,1);       
        delay(3000);
        SEmove1(pinnode1,90);
        delay(2000);
        break;
        }
      }
    }//grasp 4

    hold(pinnode1,pinnode2,pinnode3);
    
    while(1)
    {
      if(Mirf.dataReady()) {  //当接收到程序，便从串口输出接收到的数据
      Mirf.getData((byte *) &choice);
      }  
      if (choice == 'p')
      {

        //kay part
        if (counter == 0)
        {
        initiate();
        for(int i = 0; i < NUMBER_OF_PERIOD; i++)
          {
          foreach(i);
          }
        }
        delay(5000);
        counter = counter + 1;
        Hpudding(pinnode1,pinnode2,pinnode3);
      }
    }

/*kai
   initiate();

  for(int i = 0; i < NUMBER_OF_PERIOD; i++)
  {
    foreach(i + 1);
  }

  finish();

//Kai 's part

    SEmove1(pinnode1,0);
    SEmove1(pinnode1,90);
    SEmove1(pinnode1,180);
    SEmove2(pinnode2,0);
    SEmove2(pinnode2,90);
    SEmove2(pinnode2,180); 
    SEmove3(pinnode3,0);
    SEmove3(pinnode3,90);

//鑸垫満part
   throaway(pinnode4);
   delay(3000);
   throaway(pinnode5);
   delay(3000);
 
//鐢佃矾part
*/
/*swon
    grasp(pinnode1,pinnode2,pinnode3,0);
    delay(3000);
    hold(pinnode1,pinnode2,pinnode3);
    delay(3000);
    grasp(pinnode1,pinnode2,pinnode3,1);
    delay(3000);
    hold(pinnode1,pinnode2,pinnode3);
    delay(3000);
    Lpudding(pinnode1,pinnode2,pinnode3,0);
    hold(pinnode1,pinnode2,pinnode3);
    delay(3000);
    Lpudding(pinnode1,pinnode2,pinnode3,1);
    delay(3000);
    hold(pinnode1,pinnode2,pinnode3);
//Swon 's part
*/
/*previous
  
  switch(choice)
  {
    case 'a':
    {

    }
    
    case 'd':
    {     
      
      delay(1000);

      delay(3000);     //not a problem.
      while(Serial.read() >= 0 ){}
      break;
    }
  
  /*  鍙绋嬪簭锛歝ase 'd':
    {
      grasp(pinnode1,pinnode2,pinnode3,1);
      delay(2000);  
      for (int i = 0; i < 20; i ++)
      {
        Serial.write('6');
        delay(50);
      }
      delay(5000);
      Serial.flush();
      while(Serial.read() => 0 ){}
      break;
    }   
     case 'g':
    {
      initiate();
      for(int i = 0; i < NUMBER_OF_PERIOD; i++)
      {
        foreach(i + 1);
      }
      finish();

      Hpudding(pinnode1,pinnode2,pinnode3);
      hold(pinnode1,pinnode2,pinnode3);
       
      break;
    }// put down the second and then the first object              

   
    case 'l':
    {
      Lpudding(pinnode1,pinnode2,pinnode3,0);
//      delay(1000);
//      for (int i = 0; i < 20; i ++)
//      {
//        Serial.write('6');
//        delay(50);
//      }
//      delay(5000);
  //    while(Serial.read() >= 0 ){}
      break;
    }// put down the first object
    
    case 'r':
    {
      Lpudding(pinnode1,pinnode2,pinnode3,1);
 //     delay(1000);
   //   for (int i = 0; i < 20; i ++)
     // {
      //  Serial.write('6');
       // delay(50);
     // }
//      while(Serial.read() >= 0){}

      break;
    }// put down the second object
    
    default:
      break;
  }
*/
/*
  if (choice == 'a' || choice == 'd' || choice == 'g' || choice == 'l' || choice == 'r')
  {
    choice = 'w';
    counter = 1;
  }
  */
/*  
  if (counter == 0)
  {
    initiate();
    for(int i = 0; i < NUMBER_OF_PERIOD; i++)
    {
      foreach(i);
    }
  }
  delay(500000);
  counter = counter + 1;
*/
}
