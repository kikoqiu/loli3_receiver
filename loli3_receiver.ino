#include "nRF24L01.h"
#include <EEPROM.h>

void fatal(int ms){
  long pos=0;
  SPCR = 0;  
  pinMode(LED_BUILTIN,OUTPUT);
  while ( true ) {
     digitalWrite(LED_BUILTIN,pos++&1); 
     delay(ms);
  };
  //SPCR = SPI_io_save;
}

bool CH1,CH2,CH3,CH4,CH5,CH6,CH7,CH8,LED;
/**************************************************************************/

u8 randomskip[100]={4,1,3,2,2,1,0,0,2,2,2,3,4,1,2,1,4,3,3,4,//随机跳频序列
										 2,0,2,2,3,1,2,3,2,2,2,4,2,4,0,3,4,2,3,1,
										 0,3,1,3,3,0,2,0,4,3,3,3,3,3,4,1,1,4,3,0,
	                   1,0,3,2,3,2,3,3,4,4,1,3,0,0,3,1,3,3,3,0,
	                   3,3,4,1,2,4,1,3,0,1,3,4,4,3,2,3,1,2,3,3};

//数据保存
bool Data_change;
u8  TEMP_DATA[50];
											

//无线通信
u8 rx[11];		//接收的11字节数据
u8 tx[11];
u8 m;
u8 hopping_turn,hopping_num,hopping_count;

u8 address[5];
u8 address_0[5]={'L','O','V','E','!'};//使用LOVE作为对频暗语

u8 hopping[5]={10,35,60,85,110};
bool first,restar;
bool connecting,nolose;
bool jump_1,jump_2,jump_mode;

u8 NRF_error;
u8 TX_power=3;



//通道输出
u16  timer1=0,timer2=0,timer3=0,timer4=0,timer5=0;
u8  rx_num=0,receive=0;
u8  Light_LED=0;

u16  out_control_data[8]={511,511,80,511,511,511,511,511};

u16 CH_data[8]={0,1024,0,1024,0,1024};
u16 buff[8];
u16 temp1,temp2;
u8 t_output;



//电压检测
u8 t_adc;
u16 adc1,adc2;
long last_adc1=1,last_adc2=1;
u16 voltage_bec,voltage_total;



//PPM
u8 t_PPMout;
u8 T_h,T_l;
bool PPM_OUT;


bool CH1_SW,CH2_SW,CH3_SW,CH4_SW,CH5_SW,CH6_SW,CH7_SW,CH8_SW;
bool CH1_PWM,CH2_PWM,CH7_PWM;



//SBUS
bool SBUS;
u8 t_sbus;
u8  Sbus_buff[25],Sbus_tx[25];
u16  Sbus_data[8];



/**************************************************************************/
//ROM存储
u8 EEPROM_read(u8 address_H,u16 address_L)	//读数据
{
  return EEPROM[address_L];
}
void EEPROM_write(u8 address_H,u16 address_L,u8 byte)	//写入数据
{
  EEPROM.update(address_L, byte);
}
void EEPROM_clean(u8 address_H)	//擦除数据
{
  //no action
}
u8 EEPROM_test(u8 address_H)
{		
	return 0;
}



void DATA_read()
{
	u8 i;
	u16 sum=0,sum_read;
	
	//IAP_CONTR=0x83;
	for(i=0;i<40;i++)
	{
		TEMP_DATA[i]=EEPROM_read(0,i);	
    Serial.print(TEMP_DATA[i]);
    Serial.print(",");
		sum+=TEMP_DATA[i];
	}
  Serial.println();
	
	sum_read=EEPROM_read(0,40);//读校验和
	sum_read<<=8;
	sum_read+=EEPROM_read(0,41);
	
	if(TEMP_DATA[0]!=0xab||sum!=sum_read)//比较校验和
	{
		
	   first=1;

	}
	else
	{
	  hopping[0]=TEMP_DATA[1];
		hopping[1]=TEMP_DATA[2];
		hopping[2]=TEMP_DATA[3];
		hopping[3]=TEMP_DATA[4];
		hopping[4]=TEMP_DATA[5];
		address[0]=TEMP_DATA[6];
		address[1]=TEMP_DATA[7];
		address[2]=TEMP_DATA[8];
		address[3]=TEMP_DATA[9];
		address[4]=TEMP_DATA[10];
		
		out_control_data[0]=TEMP_DATA[11];
		out_control_data[0]<<=8;
		out_control_data[0]+=TEMP_DATA[12];
		out_control_data[1]=TEMP_DATA[13];
		out_control_data[1]<<=8;
		out_control_data[1]+=TEMP_DATA[14];
		out_control_data[2]=TEMP_DATA[15];
		out_control_data[2]<<=8;
		out_control_data[2]+=TEMP_DATA[16];
		out_control_data[3]=TEMP_DATA[17];
		out_control_data[3]<<=8;
		out_control_data[3]+=TEMP_DATA[18];
		out_control_data[4]=TEMP_DATA[19];
		out_control_data[4]<<=8;
		out_control_data[4]+=TEMP_DATA[20];
		out_control_data[5]=TEMP_DATA[21];
		out_control_data[5]<<=8;
		out_control_data[5]+=TEMP_DATA[22];
		out_control_data[6]=TEMP_DATA[23];
		out_control_data[6]<<=8;
		out_control_data[6]+=TEMP_DATA[24];
		out_control_data[7]=TEMP_DATA[25];
		out_control_data[7]<<=8;
		out_control_data[7]+=TEMP_DATA[26];
		
		PPM_OUT=TEMP_DATA[27];
		CH1_PWM=TEMP_DATA[28];
		CH2_PWM=TEMP_DATA[29];
		CH7_PWM=TEMP_DATA[30];
		CH1_SW=TEMP_DATA[31];
		CH2_SW=TEMP_DATA[32];
		CH3_SW=TEMP_DATA[33];
		CH4_SW=TEMP_DATA[34];
		CH5_SW=TEMP_DATA[35];
		CH6_SW=TEMP_DATA[36];
		CH7_SW=TEMP_DATA[37];
		CH8_SW=TEMP_DATA[38];
		SBUS=TEMP_DATA[39];
		
	} 	
}
void DATA_save()
{
	u8 i;u16 sum;
	
	TEMP_DATA[0]=0xab;

	TEMP_DATA[1]=hopping[0];	//修改目标模型数据
	TEMP_DATA[2]=hopping[1];
	TEMP_DATA[3]=hopping[2];
	TEMP_DATA[4]=hopping[3];
	TEMP_DATA[5]=hopping[4];
	TEMP_DATA[6]=address[0];	//修改目标模型数据
	TEMP_DATA[7]=address[1];
	TEMP_DATA[8]=address[2];
	TEMP_DATA[9]=address[3];
	TEMP_DATA[10]=address[4];
	
	TEMP_DATA[11]=out_control_data[0]>>8;
	TEMP_DATA[12]=out_control_data[0];
	TEMP_DATA[13]=out_control_data[1]>>8;
	TEMP_DATA[14]=out_control_data[1];
	TEMP_DATA[15]=out_control_data[2]>>8;
	TEMP_DATA[16]=out_control_data[2];
	TEMP_DATA[17]=out_control_data[3]>>8;
	TEMP_DATA[18]=out_control_data[3];
	TEMP_DATA[19]=out_control_data[4]>>8;
	TEMP_DATA[20]=out_control_data[4];
	TEMP_DATA[21]=out_control_data[5]>>8;
	TEMP_DATA[22]=out_control_data[5];
	TEMP_DATA[23]=out_control_data[6]>>8;
	TEMP_DATA[24]=out_control_data[6];
	TEMP_DATA[25]=out_control_data[7]>>8;
	TEMP_DATA[26]=out_control_data[7];
	
	TEMP_DATA[27]=PPM_OUT;
	TEMP_DATA[28]=CH1_PWM;
	TEMP_DATA[29]=CH2_PWM;
	TEMP_DATA[30]=CH7_PWM;
	TEMP_DATA[31]=CH1_SW;
	TEMP_DATA[32]=CH2_SW;
	TEMP_DATA[33]=CH3_SW;
	TEMP_DATA[34]=CH4_SW;
	TEMP_DATA[35]=CH5_SW;
	TEMP_DATA[36]=CH6_SW;
	TEMP_DATA[37]=CH7_SW;
	TEMP_DATA[38]=CH8_SW;
	TEMP_DATA[39]=SBUS;
	
	sum=0;
	for(i=0;i<40;i++)
	{
		sum+=TEMP_DATA[i];
	}
	TEMP_DATA[40]=sum>>8;//写入校验和
	TEMP_DATA[41]=sum;
	
	
  EEPROM_clean(0);
        
  Serial.println("before write");
  delay(1);
  for(i=0;i<42;i++)
  {
    EEPROM_write(0,i,TEMP_DATA[i]);
  }
  Serial.println("after write");

}


/**************************************************************************/
//NRF24L01
#define SOFTSPI 0
#if SOFTSPI
#warning buggy
 const int MOSI_pin  = 11;
 const int MISO_pin  = 12;
 const int SCK_pin   = 13;
#else
#include <SPI.h>
#endif
const int ce_pin=7;
const int csn_pin=8;

#if SOFTSPI
void sdelay(){
  for(volatile long tt=0;tt<100;++tt){}
  //delayus(100);
  //delay(1);
}
/*u8 SPI_io(u8 byte)
{  
	u8 i;
	for(i=0;i<8;i++)
	{
		MDI=(byte&0x80);
		SCK_pin=1;
		byte<<=1;
		byte|=MDO;
		SCK_pin=0;
	}
	return byte;
}*/
u8 SPI_io(u8 byte)
{  
	u8 i;
  Serial.println(byte);

	for(i=0;i<8;i++)
	{
    digitalWrite(SCK_pin,LOW);    
    digitalWrite(MOSI_pin,(byte&0x80)?HIGH:LOW);
    byte<<=1;
    sdelay();
    digitalWrite(SCK_pin,HIGH);
    sdelay();
		byte|=digitalRead(MISO_pin)?1:0;
	}
  Serial.println(byte);
	return byte;
}
#else
u8 SPI_io(u8 byte)
{
  //Serial.println(byte);
  u8 ret= SPI.transfer(byte);
  //Serial.println(ret);
  return ret;
}
#endif





u8 REG_write(u8 address,u8 command,bool addressonly=false)
{
	digitalWrite(csn_pin,0);
	u8 status=SPI_io(0x20+address);
  if(!addressonly){
	  SPI_io(command);
  }
	digitalWrite(csn_pin,1);
  return status;
}

#define R_REGISTER    0x00
u8 REG_read(u8 address)
{
	digitalWrite(csn_pin,0);
  SPI_io(R_REGISTER|address);
  u8 result = SPI_io(0xff);
	digitalWrite(csn_pin,1);
  return result;
}



void FIFO_write(u8 DATA_OUT[],u8 lengh)
{
	u8 i;
	digitalWrite(csn_pin,0);
	SPI_io(0xa0);
	for(i=0;i<lengh;i++)
	SPI_io(DATA_OUT[i]);
	digitalWrite(csn_pin,1);
}
void FIFO_read(u8 DATA_IN[],u8 lengh)		//读取接收数据缓冲区
{
	u8 i;
	digitalWrite(csn_pin,0);
	SPI_io(0x61);	//读取命令
	for(i=0;i<lengh;i++)
	DATA_IN[i]=SPI_io(0);	   
	digitalWrite(csn_pin,1);
}

void TX_address(u8 DATA_IN[])
{
	digitalWrite(csn_pin,0);		 
	SPI_io(0x20+0x10);
	SPI_io(DATA_IN[0]);
	SPI_io(DATA_IN[1]);
	SPI_io(DATA_IN[2]);
	SPI_io(DATA_IN[3]);
	SPI_io(DATA_IN[4]);
	digitalWrite(csn_pin,1);  
}  
void RX_address(u8 DATA_IN[])
{
	digitalWrite(csn_pin,0);		 
	SPI_io(0x20+0x0a);
	SPI_io(DATA_IN[0]);
	SPI_io(DATA_IN[1]);
	SPI_io(DATA_IN[2]);
	SPI_io(DATA_IN[3]);
	SPI_io(DATA_IN[4]);
	digitalWrite(csn_pin,1);  
}
void RX_mode()				 
{
	digitalWrite(ce_pin,0);
	REG_write(0x00,0x3b); //CRC,8 bool,Power on,RX
	digitalWrite(ce_pin,1);
} 				   
	
void TX_mode()				 
{
	digitalWrite(ce_pin,0);
	REG_write(0x00,0x0a);
  digitalWrite(ce_pin,1);
}

void  NRF_power(u8 P)				//发射功率设置 250k
{														
	digitalWrite(ce_pin,0);
	if(P==3)REG_write(0x06,0x27);		  //0db 修正之前注释错误
	else if(P==2)REG_write(0x06,0x25);	  //-6db
	else if(P==1)REG_write(0x06,0x23);	  //-12db
	else if(P==0)REG_write(0x06,0x21);    //-18db
	digitalWrite(ce_pin,1);
}

void NRF_size(u8 l)
{
	digitalWrite(ce_pin,0);
	REG_write(0x11,l);  
	digitalWrite(ce_pin,1);
}

void NRF_channel(u8 c)
{
	digitalWrite(ce_pin,0);
	REG_write(0x05,c);  
	digitalWrite(ce_pin,1);
}

uint8_t get_status()
{
    return REG_write(RF24_NOP,RF24_NOP,true);
}

bool available(uint8_t* pipe_num=0)
{
    // get implied RX FIFO empty flag from status byte
    uint8_t pipe = (get_status() >> RX_P_NO) & 0x07;
    if (pipe > 5)
        return 0;

    // If the caller wants the pipe number, include that
    if (pipe_num)
        *pipe_num = pipe;

    return 1;
}

void NRF_init()
{
  Serial.println("NRF_init");
	digitalWrite(ce_pin,0);

  NRF_channel(33);

	//digitalWrite(SCK_pin,0);
	REG_write(0x01,0x00); //禁止 自动应答
	REG_write(0x02,0x01); //允许 P0信道
	REG_write(0x04,0x00); //禁止 自动重发
	RX_mode(); 		//	 REG_write(0x1d,0x01);
	NRF_channel(66);
	NRF_power(TX_power);
	NRF_size(11);
	RX_address(address);
	TX_address(address);
}
/*
void NRF_test()	//无线模块终极测试
{	
	u8 reset_err=0;
	digitalWrite(ce_pin,0);
	//digitalWrite(SCK_pin,0);
	digitalWrite(csn_pin,0);
	
	if(SPI_io(0x20)!=0x0e){reset_err=1;}
	SPI_io(0x0a);

	digitalWrite(csn_pin,1);
  delay(1);
	digitalWrite(csn_pin,0);
	SPI_io(0x00);

	if(SPI_io(0x00)!=0x0a){NRF_error|=0x02;}//MOSI_pin bad
	digitalWrite(csn_pin,1);
  delay(1);

	REG_write(0x01,0x00);
	REG_write(0x04,0x00);
	REG_write(0x11,1);
	
	FIFO_write(tx,1);
	digitalWrite(ce_pin,1);

	delay(2);
	
	digitalWrite(csn_pin,0);

	if(SPI_io(0x00)!=0x2e){NRF_error|=0x04;}//CE bad
	digitalWrite(csn_pin,1);

	if((!available()))NRF_error|=0x18;	//(!available()) bad
	else 
	{
		if(NRF_error&0x04==0)NRF_error|=0x10;		//MISO_pin bad
	}
	digitalWrite(ce_pin,1);
	
	if(reset_err&&NRF_error>1)NRF_error|=0x01;//CSN,CLK bad
	
	REG_write(0x07,0x20);	//清除TX中断信号
}
*/

void receiver_connect()
{
	u8 t;
	NRF_power(0);
	NRF_channel(33);
  
  if(REG_read(SETUP_AW)!=3||REG_read(RF_CH)!=33){
    Serial.println("nrf24chip not connected!");
    fatal(100);
  }

	TX_address(address_0);
	RX_address(address_0);
	while((!available()));
	FIFO_read(rx,11);		//读取接收数据
	digitalWrite(ce_pin,0);
	REG_write(0x07,0x40);	//清除无线模块中断信号
	digitalWrite(ce_pin,1);
	if(rx[0]==0xa0)
	{
		hopping[0]=rx[1];
		hopping[1]=rx[2];
		hopping[2]=rx[3];
		hopping[3]=rx[4];
		hopping[4]=rx[5];
		address[0]=rx[6];
		address[1]=rx[7];
		address[2]=rx[8];
		address[3]=rx[9];
		address[4]=rx[10];
	}
	
	tx[0]='O',tx[1]='K';
	connecting=1;
	while(connecting)
	{
		TX_mode();		
		NRF_channel(33);
		TX_address(address_0);
		RX_address(address_0);
		FIFO_write(tx,11);
		delay(1);
		
		RX_mode();
		NRF_channel(hopping[0]);
		TX_address(address);
		RX_address(address);
		while(1)
		{
			delay(1);
			if((!available())==0)
			{
				FIFO_read(rx,11);		//读取接收数据
				digitalWrite(ce_pin,0);
				REG_write(0x07,0x40);	//清除无线模块中断信号
				digitalWrite(ce_pin,1);	 
				connecting=0;break;
			}
			t++;if(t>100){t=0;break;}
		}			  
	}
	DATA_save();
	RX_address(address);
	TX_address(address);
	NRF_power(3);
}

/**************************************************************************/

void data_check(int x,int max,int min)
{
	
	if(x>max)x=max;
	if(x<min)x=min;

}

void Get_Sbus_data()
{
	Sbus_data[0]=CH_data[0]<<1;
	Sbus_data[1]=CH_data[1]<<1;
	Sbus_data[2]=CH_data[2]<<1;
	Sbus_data[3]=CH_data[3]<<1;
	Sbus_data[4]=CH_data[4]<<1;
	Sbus_data[5]=CH_data[5]<<1;
	Sbus_data[6]=CH_data[6]<<1;
	Sbus_data[7]=CH_data[7]<<1;
  /*for(char i=0;i<8;++i){
    Serial.print(Sbus_data[i]);
    Serial.print(",");
  }
  Serial.println();*/

	

	Sbus_buff[1]=Sbus_data[0];//CH1~CH8
	Sbus_buff[2]=Sbus_data[0]>>8;
	Sbus_buff[2]+=Sbus_data[1]<<3;

	Sbus_buff[3]=Sbus_data[1]>>5;
	Sbus_buff[3]+=Sbus_data[2]<<6;

	Sbus_buff[4]=Sbus_data[2]>>2;
	
	Sbus_buff[5]=Sbus_data[2]>>10;
		Sbus_buff[5]+=Sbus_data[3]<<1;
	
	Sbus_buff[6]=Sbus_data[3]>>7;
		Sbus_buff[6]+=Sbus_data[4]<<4;
	
	Sbus_buff[7]=Sbus_data[4]>>4;
		Sbus_buff[7]+=Sbus_data[5]<<7;
		
	Sbus_buff[8]=Sbus_data[5]>>1;
		
	Sbus_buff[9]=Sbus_data[5]>>9;
		Sbus_buff[9]+=Sbus_data[6]<<2;
		
	Sbus_buff[10]=Sbus_data[6]>>6;
		Sbus_buff[10]+=Sbus_data[7]<<5;
		
	Sbus_buff[11]=Sbus_data[7]>>3;
	
	Sbus_tx[0]=0x0f;//Star				
	Sbus_tx[1]=Sbus_buff[1];//CH1~CH8
	Sbus_tx[2]=Sbus_buff[2];
	Sbus_tx[3]=Sbus_buff[3];
	Sbus_tx[4]=Sbus_buff[4];
	Sbus_tx[5]=Sbus_buff[5];
	Sbus_tx[6]=Sbus_buff[6];
	Sbus_tx[7]=Sbus_buff[7];
	Sbus_tx[8]=Sbus_buff[8];
	Sbus_tx[9]=Sbus_buff[9];
	Sbus_tx[10]=Sbus_buff[10];
	Sbus_tx[11]=Sbus_buff[11];				
	Sbus_tx[12]=0;//CH9~CH16
	Sbus_tx[13]=0;
	Sbus_tx[14]=0;
	Sbus_tx[15]=0;
	Sbus_tx[16]=0;
	Sbus_tx[17]=0;
	Sbus_tx[18]=0;
	Sbus_tx[19]=0;
	Sbus_tx[20]=0;
	Sbus_tx[21]=0;
	Sbus_tx[22]=0;				
	Sbus_tx[23]=0;//flag
	Sbus_tx[24]=0;//End

}

void Cycle()
{
	timer1++;
	if(timer1>333)			//每秒统计接收到的有效信号，小于20个点亮LED示警，表示信号微弱
	{
		timer1=333;
		rx_num=0;
		Light_LED=2;
	}
	
	
	timer2++;
	if(timer2>333)
	{
		timer2=0;
		rx_num=receive;
		receive=0;
	}
	
	timer3++;
	if(timer3>666)//信号丢失2秒触发失控保护
	{
		timer3=666;
		CH_data[0]=out_control_data[0];
		CH_data[1]=out_control_data[1];
		CH_data[2]=out_control_data[2];
		CH_data[3]=out_control_data[3];
		CH_data[4]=out_control_data[4];
		CH_data[5]=out_control_data[5];
		CH_data[6]=out_control_data[6];
		CH_data[7]=out_control_data[7];
		
		Get_Sbus_data();
	}
	
	if(Data_change)
	{
		timer4++;
		if(timer4>333)
		{
			timer4=0;
			Data_change=0;
			DATA_save();
		}	
	}
	/*
	if(Light_LED==1)
	{
		LED=0;
		timer5++;
		if(timer5>10)
		{
			timer5=0;
			Light_LED=0;
			
		}
	}
	else if(Light_LED==2)LED=0;
	else LED=1;
*/
		
	m++;					//两个标志位循环跳频
	if(jump_1==0)
	{
		if(m>7)nolose=0,jump_1=1,jump_2=0;	  //未收到信号，开始第一跳
	}
	else
	{
		if(jump_2==0)
		{
			if(m>14)nolose=0,jump_2=1;		 //第一跳之后仍无信号，开始第二跳
		}
		else
		{
			 if(m>22)m=0,nolose=0,jump_1=0;	 //第二跳之后仍无信号，复位重新跳。
			
		}		
	}	
	
	t_adc++;
  /*
	if(t_adc==1)
	{
		adc1=ADC_RES;
		adc1<<=2,adc1+=ADC_RESL;
		
		last_adc1=(adc1+last_adc1*9/10);
		
		P1ASF=0x02;
		ADC_CONTR=0x89;	
	}
	else if(t_adc==2)
	{
		t_adc=0;
		adc2=ADC_RES;
		adc2<<=2,adc2+=ADC_RESL;
		
		last_adc2=(adc2+last_adc2*9/10);
		
		P1ASF=0x00;
		ADC_CONTR=0x88;	
	}*/
	
}

/*************************************************************/
//串口通信

/*
void UartInit(void)		//100000bps@12.000MHz
{
	SCON = 0xD0;		//9位数据,可变波特率
	AUXR |= 0x01;		//串口1选择定时器2为波特率发生器
	AUXR |= 0x04;		//定时器2时钟为Fosc,即1T
	T2L = 0xE2;		//设定定时初值
	T2H = 0xFF;		//设定定时初值

}

*/


void initial()
{
	/*CH5=0;CH6=1;			//如果CH5与CH6被短接，重新对码
	delay(1);
	if(CH6==0)		  //修正通道6插上舵机不能用
	{
		P3M0=0x02;	  //插上舵机也会使CH6为0，所以将CH5设为推挽
		CH5=1;
		delay(1);
		if(CH6)restar=1;   //如果CH6被拉高，说明5/6通道短接，启动重新对码
	}
	CH5=0;CH6=0;
	 	
	
	P3M0=0xcf;	 //
	P1M1=0x02;	//将通道输出IO口配置为推挽模式，保证正常驱动电调与舵机

	
	while(1)
	{
		if(EEPROM_test(2))break;
		else delay(100);
	}
	*/

	DATA_read();
	data_check(out_control_data[0],1023,0);
	data_check(out_control_data[1],1023,0);
	data_check(out_control_data[2],1023,0);
	data_check(out_control_data[3],1023,0);
	data_check(out_control_data[4],1023,0);
	data_check(out_control_data[5],1023,0);
	data_check(out_control_data[6],1023,0);
	data_check(out_control_data[7],1023,0);
	
  pinMode(ce_pin,OUTPUT);
  pinMode(csn_pin,OUTPUT);
  digitalWrite(ce_pin,0);
  digitalWrite(csn_pin,1);

#if SOFTSPI
  pinMode(SCK_pin,OUTPUT);
  pinMode(MOSI_pin,OUTPUT);
  pinMode(MISO_pin,INPUT);
#else
  SPI.begin();
#endif

	/*.println("nfrtest");
	NRF_test();
  
	if(NRF_error)//模块错误闪灯
	{
		//LED_flash(10);    
    Serial.print("NRF_error:");
    Serial.println(NRF_error);
    fatal(100);
	}*/
	
	NRF_init();

	
  //restar=1;
	if(restar)first=1;
	if(first)
	{
    Serial.println("receiver_connect");
		receiver_connect();
    Serial.println("receiver_connected");
	}
  
}

//process input 
void processInputData(){
  if(rx[0]==0xa2)
  {
    Light_LED=1;
    
    if(rx[1]&0x80)
    {
      PPM_OUT=1;
    }
    else 
    {
      PPM_OUT=0;
    }
    
    if(rx[1]&0x40)
    {
      SBUS=1;
    }
    else 
    {
      SBUS=0;
    }
    
    if(SBUS)
    {
      //UartInit();
    }
    
    if(rx[1]&0x08)
    {
      CH1_PWM=1;
    }
    else 
    {
      CH1_PWM=0;
    }
    
    if(rx[1]&0x04)
    {
      CH2_PWM=1;
    }
    else 
    {
      CH2_PWM=0;
    }
    
    if(rx[1]&0x02)
    {
      CH7_PWM=1;
    }
    else 
    {
      CH7_PWM=0;
    }
    
    if(rx[2]&0x80)CH1_SW=1;
    else CH1_SW=0;
    if(rx[2]&0x40)CH2_SW=1;
    else CH2_SW=0;
    if(rx[2]&0x20)CH3_SW=1;
    else CH3_SW=0;
    if(rx[2]&0x10)CH4_SW=1;
    else CH4_SW=0;
    if(rx[2]&0x08)CH5_SW=1;
    else CH5_SW=0;
    if(rx[2]&0x04)CH6_SW=1;
    else CH6_SW=0;
    if(rx[2]&0x02)CH7_SW=1;
    else CH7_SW=0;
    if(rx[2]&0x01)CH8_SW=1;
    else CH8_SW=0;				
    
    Data_change=1;timer4=0;
  }
  else if(rx[0]==0xa0)
  {
    Light_LED=1;LED=0;
    
    buff[0]=rx[1];
    buff[0]<<=2;
    buff[0]+=rx[2]>>6;
    buff[1]=rx[2]&0x3f;
    buff[1]<<=4;
    buff[1]+=rx[3]>>4;			
    buff[2]=rx[3]&0x0f;
    buff[2]<<=6;
    buff[2]+=rx[4]>>2;
    buff[3]=rx[4]&0x03;
    buff[3]<<=8;
    buff[3]+=rx[5];

    buff[4]=rx[6];
    buff[4]<<=2;
    buff[4]+=rx[7]>>6;
    buff[5]=rx[7]&0x3f;
    buff[5]<<=4;
    buff[5]+=rx[8]>>4;			
    buff[6]=rx[8]&0x0f;
    buff[6]<<=6;
    buff[6]+=rx[9]>>2;
    buff[7]=rx[9]&0x03;
    buff[7]<<=8;
    buff[7]+=rx[10];
    
    
    data_check(buff[0],1023,0);
    data_check(buff[1],1023,0);
    data_check(buff[2],1023,0);
    data_check(buff[3],1023,0);
    data_check(buff[4],1023,0);
    data_check(buff[5],1023,0);
    data_check(buff[6],1023,0);
    data_check(buff[7],1023,0);
    
    out_control_data[0]=buff[0];
    out_control_data[1]=buff[1];
    out_control_data[2]=buff[2];
    out_control_data[3]=buff[3];
    out_control_data[4]=buff[4];
    out_control_data[5]=buff[5];
    out_control_data[6]=buff[6];
    out_control_data[7]=buff[7];
    
    
    Data_change=1;timer4=0;				
    
  }
  else if(rx[0]==0xa1)
  {
    buff[0]=rx[1];
    buff[0]<<=2;
    buff[0]+=rx[2]>>6;
    buff[1]=rx[2]&0x3f;
    buff[1]<<=4;
    buff[1]+=rx[3]>>4;			
    buff[2]=rx[3]&0x0f;
    buff[2]<<=6;
    buff[2]+=rx[4]>>2;
    buff[3]=rx[4]&0x03;
    buff[3]<<=8;
    buff[3]+=rx[5];

    buff[4]=rx[6];
    buff[4]<<=2;
    buff[4]+=rx[7]>>6;
    buff[5]=rx[7]&0x3f;
    buff[5]<<=4;
    buff[5]+=rx[8]>>4;			
    buff[6]=rx[8]&0x0f;
    buff[6]<<=6;
    buff[6]+=rx[9]>>2;
    buff[7]=rx[9]&0x03;
    buff[7]<<=8;
    buff[7]+=rx[10];
    
    
    data_check(buff[0],1023,0);
    data_check(buff[1],1023,0);
    data_check(buff[2],1023,0);
    data_check(buff[3],1023,0);
    data_check(buff[4],1023,0);
    data_check(buff[5],1023,0);
    data_check(buff[6],1023,0);
    data_check(buff[7],1023,0);
    
    CH_data[0]=buff[0];
    CH_data[1]=buff[1];
    CH_data[2]=buff[2];
    CH_data[3]=buff[3];
    CH_data[4]=buff[4];
    CH_data[5]=buff[5];
    CH_data[6]=buff[6];
    CH_data[7]=buff[7];
    
    Get_Sbus_data();
  }
}

int mymain(){
  delay(100);

  CH1=0;
	CH2=0;
	CH3=0;
	CH4=0;
	CH5=0;
	CH6=0;
	CH7=0;
	CH8=0;

  initial();


  NRF_channel(hopping[0]);

	while((!available()));
  Serial.println("ready");
	nolose=1;
	

  unsigned long prevtime=micros();
  long timepased=0;
	while(1)
	{
		do{
      unsigned long current=micros();
      timepased+=(unsigned long)((long) current - (long)prevtime);
      prevtime=current;
      //Serial.println(timepased);
      if(timepased>3000){
        Cycle();
        //Serial.println("cycling");
        //Serial.println(current);
        timepased-=3000;
      }
    }while((!available())&&nolose);


		if(nolose)
		{
      unsigned long prevtime=micros();
			jump_1=0;jump_2=0;hopping_count=0;//收到有效信号后刷新跳频器
			receive++;timer1=0;timer3=0;m=0;	
			if(Light_LED)Light_LED=0;

			FIFO_read(rx,11);		//读取接收数据
			digitalWrite(ce_pin,0);
			REG_write(0x07,0x40);	//清除无线模块中断信号
			digitalWrite(ce_pin,1);

      //process input first
      processInputData();
      //wait for the sender to get ready for rx   !!! 
      //delay(1);
      while(true){
        unsigned long timepassed=(long)micros()-long(prevtime);
        if(timepassed>=500)break;
      }

			tx[0]=rx_num;
      last_adc1=1299210/4;
      last_adc2=1299210/2;
			voltage_bec=1299210/last_adc1;      
			tx[1]=voltage_bec>>8;
			tx[2]=voltage_bec;
			voltage_total=(long)last_adc2*voltage_bec*3/10240;
			tx[3]=voltage_total>>8;
			tx[4]=voltage_total;
			//Serial.println((int)receive);
      //Serial.println((int)rx_num);

			FIFO_write(tx,11);
			delay(1);

			RX_mode();

			if(jump_mode)
			{
				jump_mode=0;
				hopping_turn=randomskip[hopping_num];
			}
			hopping_turn++;
			if(hopping_turn>4)hopping_turn=0;
			NRF_channel(hopping[hopping_turn]);	
		}
		else
		{
      Serial.println("nolose=0");
		 	hopping_count++;
			if(hopping_count>5)jump_mode=1;
			
			if(jump_mode)
			{
				hopping_num++;
				if(hopping_num>99)hopping_num=0;
				NRF_channel(hopping[randomskip[hopping_num]]);
				nolose=1;
			}
			else
			{
				hopping_turn++;
				if(hopping_turn>4)hopping_turn=0;
				NRF_channel(hopping[hopping_turn]);
				nolose=1;
			}
		}		
	}
  
  return 0;
}




#include <printf.h>
/* ArduinoProMini-6通道接收機 
 *     by Bilibili 蔡子CaiZi
 *     
 * PWM輸出 -> 引腳2~6、9
 * 
 * NRF24L01 | Arduino
 * CE    -> 7
 * CSN   -> 8
 * MOSI_pin  -> 11
 * MISO_pin  -> 12
 * SCK_pin   -> 13
 * IRQ   -> 無連接
 * VCC   -> 小於3.6V
 * GND   -> GND
 */
 
#if false
#include <nRF24L01.h>
#include <RF24.h>
RF24 radio(7, 8); 
#endif


void setup()
{
  Serial.begin(115200);
  mymain();
}

void loop(){ 
}



























#if false

#include "ibus.h"


// //////////////////
// Edit here the pins (as for digitalRead()) that you want to read the PWM signals from.
// IMPORTANT! The pinst MUST belong to the same port. Search 'pinout' in google for
// your Arduino model and choose pins from the same port (for example PD, PB, etc.)
// On Arduino Uno/Nano you can safely use pins 2 through 7, as they all use the same port PD.
byte pins[] = {2, 3, 4, 5};


// //////////////////





#define BAUD_RATE 115200

struct {
  uint8_t mask;
  unsigned long time;
  byte state;
} pinData[sizeof(pins)];

struct {
  unsigned long time;
  uint8_t state;
} state_history[2*sizeof(pins)];

#define FOR_PINS for(int i=0; i < sizeof(pins); i++)
#define PIN pinData[i]

IBus ibus(sizeof(pins));
volatile uint8_t *port; // store the address of the port register
uint8_t mask; // mask for the pins of interest

void setup()
{
  Serial.begin(BAUD_RATE);           // setup serial

  // take the port register from the first pin. The other pins MUST use the same port
  port = portInputRegister(digitalPinToPort(pins[0]));
  mask = 0;

  FOR_PINS {
    PIN.mask = digitalPinToboolMask(pins[i]); // mask for the individual pin
    mask |= PIN.mask; // accumulate all masks
  }
}

enum {
  STATE_INIT, STATE_NORMAL
} program_state = STATE_INIT;


unsigned short tt=0;
void loop()
{
    // send the data
  ibus.begin();
  FOR_PINS {
    ibus.write(tt++);
  }
  ibus.end();
  return;

  if (program_state == STATE_INIT) {
    // wait until all pins are down
    while(*port & mask);

    program_state = STATE_NORMAL;
  }

  uint8_t prev_state = 0, new_state;
  int history_index = 0;


  // Arduinos like Uno/Nano don't have interrupts or timers for all pins so we need to poll for changes and 
  // measure the PWM pulse lengths manually. The problem is that we have to be very quick since we're dealing
  // with microsecond timescales and every instruction counts. The method employed here is to have a tight loop
  // which records all the changes to the port and the time of occurence. After all pins are down
  // we have plenty of time to process the list and send the data over IBUS.
  for(;;) {
    while(prev_state == (new_state = *port & mask)); // wait for change in state

    state_history[history_index++] = { micros(), prev_state = new_state };

    if(!new_state || history_index == sizeof(state_history))
      break; // all pins are down (we have captured all pulses) OR we have gone beyond the size of state_history
  }

  if(new_state) {
    program_state = STATE_INIT; // something's wrong. We didn't reach a state where all pins are down. Start over.
    return;
  }

  // Process the states
  for(int ih = 0; ih < history_index; ih++) {
    uint8_t state = state_history[ih].state;
    unsigned long time = state_history[ih].time;

    FOR_PINS {
      int s = (bool)(state & PIN.mask);

      if (!PIN.state && s) { // low to high
        PIN.time = time; // save pulse starting time;
      }
      else if (PIN.state && !s) { // high to low
        PIN.time = time - PIN.time; // calculate pulse width
      }
      PIN.state = s;
    }
  }

  // send the data
  ibus.begin();
  FOR_PINS {
    ibus.write(PIN.time);
  }
  ibus.end();
}



#endif










