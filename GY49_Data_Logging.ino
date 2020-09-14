/*
ATmega328P, Internal 8MHz
GY-49  Address:0x4b
DS3231 Address:0x68
*/
#include <DS3232RTC.h>  //
#include <SD.h>
#include <SPI.h>
#include <i2c_MAX44009.h>
#include <i2c.h>
#include <Wire.h>
#include <LowPower.h>

MAX44009 max44009;
File myFile;
time_t t; //create time object for time and date

byte pinCS=8;  //SD卡的CS引脚
byte Buzzer=6;
byte LedBlink=4;
const byte interruptPin =3;  //中断通道1
const byte AlarmINTPin  =2;   //中断通道0

byte NextMinute=0;  //中断用的分钟变量
byte RecorderMinute=0;  //记录用的分钟变量
byte RecorderHour=0;    //记录用的小时变量
byte RecorderDay=0;     //记录用的天变量，没必要用
byte RecorderMonth=0;

const byte minute_interval =1;   // Sets the wakeup intervall minute

byte i=0;
byte n=60;   //60分钟写一次SD卡，注意与下一行的数组的长度一致，时间再长的话，内存就溢出了
unsigned long Lux[60]={0}; //用数组存储每次记录的光照强度，一定不要超出动态变量的75%，会出各种BUG！！！

#define VoltTestPin  A2
int v;

void setup()
{
  //Serial.begin(9600);
  analogReference(INTERNAL); //an built-in reference, equal to 1.1 volts
  pinMode(Buzzer, OUTPUT);
  pinMode(LedBlink, OUTPUT);
  pinMode(interruptPin, INPUT_PULLUP);  //利用内置的上拉电阻
  pinMode(AlarmINTPin,  INPUT_PULLUP);  

  // initialize the alarms to known values, clear the alarm flags, clear the alarm interrupt flags
  RTC.setAlarm(ALM1_MATCH_DATE, 0, 0, 0, 1);
  RTC.setAlarm(ALM2_MATCH_DATE, 0, 0, 0, 1);
  RTC.alarm(ALARM_1);
  RTC.alarm(ALARM_2);
  RTC.alarmInterrupt(ALARM_1, false);
  RTC.alarmInterrupt(ALARM_2, false);
  RTC.squareWave(SQWAVE_NONE);
    
  //time_t t; //全局变量已经创建过了
  t=RTC.get();//Gets the current time of the RTC
  NextMinute=minute(t)+minute_interval;
  if (NextMinute>59)      //注意，不这么写会有溢出！！比如可能到61分钟
  {
  	NextMinute=NextMinute-60;  //小时和天的进位就不用管了，尽量节省变量空间
  }
  //Serial.print(F("NextMinute="));
  //Serial.println(NextMinute);
  RTC.setAlarm(ALM1_MATCH_MINUTES , 0, NextMinute, 0, 0);
  //RTC.setAlarm(ALM1_MATCH_SECONDS , 10, 0, 0, 0);  // Setting alarm 1 to go off minute_interval minutes from now
  RTC.alarm(ALARM_1);  // clear the alarm flag
  RTC.squareWave(SQWAVE_NONE);  // configure the INT/SQW pin for "interrupt" operation (disable square wave output)
  RTC.alarmInterrupt(ALARM_1, true);    // enable interrupt output for Alarm 1


  SPI.beginTransaction(SPISettings(2000000, MSBFIRST, SPI_MODE0)); //频率的四分之一
  delay(500);
  if(SD.begin(pinCS)){                //SD初始化，不用再加  pinMode(pinCS, OUTPUT);语句了
        //Serial.println(F("SD found"));
    } else {
        //Serial.println(F("SD missing"));
        for (byte k=0; k<2; k++)
        {
         digitalWrite(Buzzer, HIGH);
         delay(500);
         digitalWrite(Buzzer, LOW);
         delay(500);
        }
        return;
    }
  if(max44009.initialize()){          //max44009 Initialization
        //Serial.println(F("Sensor found"));
    } else {
        //Serial.println(F("Sensor missing"));
        for (byte k=0; k<3; k++)
        {
         digitalWrite(Buzzer, HIGH);
         delay(500);
         digitalWrite(Buzzer, LOW);
         delay(500);
        }
        return;
    }
  Wire.begin();
  delay(500);
  //设置两个中断
  attachInterrupt(digitalPinToInterrupt(interruptPin), EndRecorder, LOW);
  attachInterrupt(digitalPinToInterrupt(AlarmINTPin), WakeUp, FALLING);//不能用LOW触发，因为flag之后，一直是LOW

  digitalWrite(LedBlink, HIGH);  //初始化完成的提示音
  delay(2000);
  digitalWrite(LedBlink, LOW);
}

void loop() {
  //检测电源电压
  v=analogRead(VoltTestPin);
  delay(100);
  v=analogRead(VoltTestPin);
  //Serial.println("v="+String(v));
  //电压 = v*0.0033244; 0.0033244=(1.1/1024.0)/475.3*(475.3+995.6);//R1=995.6K  R2=475.3K
  if(v>960){         //v=960时，电源电压约为3.19V
    t=RTC.get();//获取开始记录的时间，只能放在这个地方
	  static unsigned long mLux_value;
    for (i=0; i < n; i++)    //i为全局变量，以便中断函数调取i
    {
    	max44009.getMeasurement(mLux_value);
    	Lux[i]=mLux_value/1000;
    	//Serial.println(String(i)+" = "+String(Lux[i]));
      digitalWrite(LedBlink, HIGH);   //指示灯
  	  delay(200);
  	  digitalWrite(LedBlink, LOW);
  	  //attachInterrupt(digitalPinToInterrupt(AlarmINTPin), WakeUp, FALLING);//放在setup里
      //Serial.println(F("Go to sleep"));
      //Serial.flush();  //如果不加它，会导致串口的数据还没有传输完成就powerDown
  	  LowPower.powerDown(SLEEP_FOREVER, ADC_OFF, BOD_OFF);
      //Serial.println(F("wake up"));

  	  //Set New Alarm
  	  time_t p;// 创建临时时间变量
	    p=RTC.get(); //gets current time from rtc
  	  NextMinute= minute(p)+minute_interval;
  		if (NextMinute>59)      ////注意，不这么写会有溢出！！比如可能到61分钟
  		{
  			NextMinute=NextMinute-60;  //小时和天的进位就不用管了，节省变量空间
  		}
     	//Serial.print(F("NextMinute="));
     	//Serial.println(NextMinute);
  		RTC.setAlarm(ALM1_MATCH_MINUTES , 0, NextMinute, 0, 0);
		  //RTC.setAlarm(ALM1_MATCH_SECONDS , second(p)+minute_interval, 0, 0, 0);
    	// clear the alarm flag
  		RTC.alarm(ALARM_1);
	}
  
    myFile = SD.open("LUX.TXT", FILE_WRITE);
    if(myFile){
      for ( byte j = 0; j < n; j++)
      {
        RecorderMonth=month(t);  //保存的是开始记录时候的时间，不是现在的时间
      	RecorderDay=day(t);
      	RecorderHour=hour(t);
      	RecorderMinute=minute(t)+j;
      	if (RecorderMinute>59)
      	{
      		RecorderMinute=RecorderMinute-60;
      		RecorderHour=RecorderHour+1;
      		if (RecorderHour>23)
      		{
      			RecorderHour=RecorderHour-24;
      			RecorderDay=RecorderDay+1;
      		}
      	}
      	myFile.println(String(RecorderMonth)+"/"+String(RecorderDay)+" "+String(RecorderHour)+":"+String(RecorderMinute)+","+String(Lux[j])+",Lux");
        myFile.println("SD write over.");
      	
      }
      myFile.close();
      //Serial.println(F("SD write over."));
      digitalWrite(Buzzer, HIGH);   //记录完的提示音
      delay(500);
      digitalWrite(Buzzer, LOW);
    } else {
      for (byte k=0; k<5; k++)
      {
        digitalWrite(Buzzer, HIGH);
        delay(500);
        digitalWrite(Buzzer, LOW);
        delay(500);
      }
    }
  } else {
      for (byte k=0; k<5; k++)
      {
        digitalWrite(Buzzer, HIGH);
        delay(500);
        digitalWrite(Buzzer, LOW);
        delay(500);
      }
  }
}

void EndRecorder(){   //注意一定要消抖！！！否则一次按键能触发8次中断！！！消抖的时间（也即中断函数执行的总时间）至少150ms。
  //GetClock();       //中断函数中不能再调用DS3231函数了，会卡死，改用其他的方式
  myFile = SD.open("LUX.TXT", FILE_WRITE);
  if(myFile){
      for (byte j = 0; j <= i; j++)
      {
        RecorderMonth=month(t);
      	RecorderDay=day(t);
      	RecorderHour=hour(t);
      	RecorderMinute=minute(t)+j;
      	if (RecorderMinute>59)
      	{
      		RecorderMinute=RecorderMinute-60;
      		RecorderHour=RecorderHour+1;
      		if (RecorderHour>23)
      		{
      			RecorderHour=RecorderHour-24;
      			RecorderDay=RecorderDay+1;
      		}
      	}
      	myFile.println(String(RecorderMonth)+"/"+String(RecorderDay)+" "+String(RecorderHour)+":"+String(RecorderMinute)+","+String(Lux[j])+",Lux");
        myFile.println("EndRecorder over.");
      }
      //Serial.println(F("EndRecorder over."));
      //Serial.flush();
      myFile.close();
      digitalWrite(LedBlink, HIGH);
      for (byte y=0; y<200; y++)   //消抖的时间（也即中断函数执行的总时间）至少150ms。
      {
        delayMicroseconds(10000);//delayMicroseconds参数不要超过10000（10ms)，否则会溢出，等于没添加该函数，剩下的时间用for循环
      }
      digitalWrite(LedBlink, LOW);
  } else {
      for (byte k=0; k<5; k++)
      {
        digitalWrite(Buzzer, HIGH);
        for (byte y=0; y<50; y++)
        {
          delayMicroseconds(10000);
        }
        digitalWrite(Buzzer, LOW);
        for (byte y=0; y<50; y++)
        {
          delayMicroseconds(10000);
        }
      }
  }
}

void WakeUp(){
	//Serial.println(F("Interrrupt Fired"));
	sleep_disable();//Disable sleep mode
	detachInterrupt(AlarmINTPin); //Removes the interrupt from pin 2;
	//RTC.alarm(ALARM_1);   // 中断函数内无法执行
}
