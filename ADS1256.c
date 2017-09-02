#include <bcm2835.h>  
#include <stdio.h>
#include <unistd.h>
#include <string.h>
#include <math.h>
#include <errno.h>

/*
             define from bcm2835.h                       define from Board DVK511
                 3.3V | | 5V               ->                 3.3V | | 5V
    RPI_V2_GPIO_P1_03 | | 5V               ->                  SDA | | 5V 
    RPI_V2_GPIO_P1_05 | | GND              ->                  SCL | | GND
       RPI_GPIO_P1_07 | | RPI_GPIO_P1_08   ->                  IO7 | | TX
                  GND | | RPI_GPIO_P1_10   ->                  GND | | RX
       RPI_GPIO_P1_11 | | RPI_GPIO_P1_12   ->                  IO0 | | IO1
    RPI_V2_GPIO_P1_13 | | GND              ->                  IO2 | | GND
       RPI_GPIO_P1_15 | | RPI_GPIO_P1_16   ->                  IO3 | | IO4
                  VCC | | RPI_GPIO_P1_18   ->                  VCC | | IO5
       RPI_GPIO_P1_19 | | GND              ->                 MOSI | | GND
       RPI_GPIO_P1_21 | | RPI_GPIO_P1_22   ->                 MISO | | IO6
       RPI_GPIO_P1_23 | | RPI_GPIO_P1_24   ->                  SCK | | CE0
                  GND | | RPI_GPIO_P1_26   ->                  GND | | CE1

*/

#define     DRDY_PIN    RPI_GPIO_P1_11         
#define     RST_PIN     RPI_GPIO_P1_12     
#define     CS_PIN	    RPI_GPIO_P1_15	

//Register address
#define     REG_STATUS  0x00
#define     REG_MUX     0x01
#define     REG_ADCON   0x02
#define     REGdrate   0x03
#define     REG_IO      0x04
#define     REG_OFC0    0x05
#define     REG_OFC1    0x06
#define     REG_OFC2    0x07
#define     REG_FSC0    0x08
#define     REG_FSC1    0x09
#define     REG_FSC2    0x0A

//Sample rate
#define     DRATE_30000      0b11110000 
#define     DRATE_15000      0b11100000 
#define     DRATE_7500       0b11010000 
#define     DRATE_3750       0b11000000 
#define     DRATE_2000       0b10110000 
#define     DRATE_1000       0b10100001 
#define     DRATE_500        0b10010010 
#define     DRATE_100        0b10000010 
#define     DRATE_60         0b01110010 
#define     DRATE_50         0b01100011 
#define     DRATE_30         0b01010011 
#define     DRATE_25         0b01000011 
#define     DRATE_15         0b00110011 
#define     DRATE_10         0b00100011 
#define     DRATE_5          0b00010011 
#define     DRATE_2_5        0b00000011 

//Commands
#define     CMD_WAKEUP       0x00 
#define     CMD_RDATA        0x01 
#define     CMD_RDATAC       0x03 
#define     CMD_SDATAC       0x0F 
#define     CMD_RREG         0x10 
#define     CMD_WREG         0x50 
#define     CMD_SELFCAL      0xF0 
#define     CMD_SELFOCAL     0xF1 
#define     CMD_SELFGCAL     0xF2 
#define     CMD_SYSOCAL      0xF3 
#define     CMD_SYSGCAL      0xF4 
#define     CMD_SYNC         0xFC 
#define     CMD_STANDBY      0xFD 
#define     CMD_RESET        0xFE

#define    STATUS_BUFFER_ENABLE     0x02
#define    STATUS_AUTOCAL_ENABLE    0x04
#define    STATUS_ORDER_LSB         0x08

//Gain level
#define    AD_GAIN_1      0x00
#define    AD_GAIN_2      0x01
#define    AD_GAIN_4      0x02
#define    AD_GAIN_8      0x03
#define    AD_GAIN_16     0x04
#define    AD_GAIN_32     0x05
#define    AD_GAIN_64     0x06

//Sensor detect current source
#define    AD_SDCS_500pA   0x08
#define    AD_SDCS_2uA     0x10
#define    AD_SDCS_10uA    0x18

//Clock divider
#define    AD_CLK_EQUAL    0x20
#define    AD_CLK_HALF     0x40
#define    AD_CLK_FOURTH   0x60

#define DRDY_IS_LOW()	((bcm2835_gpio_lev(DRDY_PIN)==0))

#define RST_1() 	bcm2835_gpio_write(RST_PIN,HIGH);
#define RST_0() 	bcm2835_gpio_write(RST_PIN,LOW);


typedef enum {FALSE = 0, TRUE = !FALSE} bool;

#define ADS1256_DRAE_COUNT = 15;

typedef struct
{
	int Gain;		                // GAIN  
	int DataRate;	                // DATA output  speed
	int AdcNow[8];			        // ADC  Conversion value 
	unsigned char Channel;			// The current channel
}ADS1256_Class;

ADS1256_Class ADS1256;

void  delay_ms(int micros);
void  chip_select(void);
void  chip_release(void);
static void WaitDRDY(void);
static void SendByte(unsigned char data);
static unsigned char ReadByte(void);
static void DataDelay(void);
static unsigned char ReadReg(unsigned char RegID);
static void WriteReg(unsigned char RegID, unsigned char RegValue);
static int32_t ReadData(void);
unsigned char ReadID(void);
void ADC_Setting(int gain, int drate);
static void Choosechannel(unsigned char ch);
int32_t GetADC(unsigned char ch);
void ADC_ISR(void);
void ADC_StartScan(void);
unsigned char ADC_Scan(void);
static void ADS1256_WriteCmd(unsigned char cmd);
void ADC_Init(void);
void ADC_Stop(void);

void  delay_ms(int micros)
{
	bcm2835_delayMicroseconds (micros);
}

void chip_select()
{
    bcm2835_gpio_write(CS_PIN,LOW);
}

void chip_release()
{
    bcm2835_gpio_write(CS_PIN,HIGH);
}

static void WaitDRDY(void)
{
	unsigned long i;

	for (i = 0; i < 400000; i++)
	{
		if (DRDY_IS_LOW())
		{
			break;
		}
	}
	if (i >= 400000)
	{
		printf("WaitDRDY() Time Out ...\r\n");
	}
}

static void SendByte(unsigned char data)
{

	delay_ms(2);
    //bcm2835_spi_transfer: Transfers one byte to and from currently selectef SPI slave 
	bcm2835_spi_transfer(data);
}

static unsigned char ReadByte(void)
{
	unsigned char read = 0;
    //bcm2835_spi_transfer: Transfers one byte to and from currently selectef SPI slave 
	read = bcm2835_spi_transfer(0xff);
	return read;
}

static void DataDelay(void)
{
	/*
		Delay from last SCLK edge for DIN to first SCLK rising edge for DOUT: RDATA, RDATAC,RREG Commands
		min  50   CLK = 50 * 0.13uS = 6.5uS
	*/
	delay_ms(10);	/* The minimum time delay 6.5us */
}

static void WriteReg(unsigned char RegID, unsigned char RegValue)
{
    /*
        Write to the registers starting with the register specified as part of the command. 
        
        The number of registers that will be written is one plus the value of the second byte in the command.
        
        1st Command Byte: 0101 rrrr where rrrr is the address to the first register to be written.
        
        2nd Command Byte: 0000 nnnn where nnnn is the number of bytes to be written – 1.

    */
    
	chip_select();
   
	SendByte(CMD_WREG | RegID);//1st Command Byte
	SendByte(0x00);//2nd Command Byte

	SendByte(RegValue);
	chip_release();
}

static unsigned char ReadReg(unsigned char RegID)
{
    /*
        Output the data from up to 11 registers starting with the register address specified as part of the command.
        
        The number of registers read will be one plus the second byte of the command. If the count exceeds the remaining registers,the addresses will wrap back to the beginning.
        
        1st Command Byte: 0001 rrrr where rrrr is the address of the first register to read.
        
        2nd Command Byte: 0000 nnnn where nnnn is the number of bytes to read – 1.
    */
	unsigned char read;

	chip_select();
	SendByte(CMD_RREG | RegID);	
	SendByte(0x00);	

	DataDelay();

	read = ReadByte();
	chip_release();

	return read;
}

static int32_t ReadData(void)
{
	unsigned long data = 0;
    static unsigned char temp[3];

	chip_select();

	SendByte(CMD_RDATA);

	DataDelay();

	
    temp[0] = ReadByte();
    temp[1] = ReadByte();
    temp[2] = ReadByte();

    data = ((unsigned long)temp[0] << 16) & 0x00FF0000;
    data |= ((unsigned long)temp[1] << 8);  
    data |= temp[2];

	chip_release();

	
    if (data & 0x800000)
    {
	    data |= 0xFF000000;
    }

	return (int32_t)data;
}

unsigned char ReadID(void)
{
	unsigned char id;

	WaitDRDY();
	id = ReadReg(REG_STATUS);
	return (id >> 4);
}

void ADC_Setting(int gain, int drate)
{
	ADS1256.Gain = gain;
	ADS1256.DataRate = drate;

	WaitDRDY();
	
	unsigned char buf[4];		
    
	/* 
        STATUS REGISTER (ADDRESS 00h):
            Bit7~4:  ID      
            Bit3:    
                Order(Data Output Bit Order)        0=MSB       1=LSB
                Input data is always shifted in most significant byte and bit first. 
                Output data is always shifted out most significant byte first. 
                The ORDER bit only controls the bit order of the output data within the byte.
            Bit2:    
                ACAL(Auto-Calibration)              0=Disabled  1=Enable
            Bit1:    
                BUFEN(Analog Input Buffer Enable)   0=Disabled  1=Enable 
            Bit0:   
                DRDY(Data Ready)                    
	*/
    
    buf[0] = (0 << 3) | (1 << 2) | (0 << 1);//MSB + Auto-Calibration Enable + Buffer Disabled 
        
    /*    
        Input Multiplexer Control Register (Address 01h):
            Bit7~4:
                PSEL3, PSEL2, PSEL1, PSEL0: Positive Input Channel (AINP) Select
                0000 = AIN0 (default)
                0001 = AIN1
                0010 = AIN2 (ADS1256 only)
                0011 = AIN3 (ADS1256 only)
                0100 = AIN4 (ADS1256 only)
                0101 = AIN5 (ADS1256 only)
                0110 = AIN6 (ADS1256 only)
                0111 = AIN7 (ADS1256 only)
                1xxx = AINCOM (when PSEL3 = 1, PSEL2, PSEL1, PSEL0 are “don’t care”)
            
            Bit3~0:
                NSEL3, NSEL2, NSEL1, NSEL0: Negative Input Channel (AINN)Select
                0000 = AIN0
                0001 = AIN1 (default)
                0010 = AIN2 
                0011 = AIN3 
                0100 = AIN4 
                0101 = AIN5 
                0110 = AIN6 
                0111 = AIN7
                1xxx = AINCOM (when NSEL3 = 1, NSEL2, NSEL1, NSEL0 are “don’t care”)
    */
    
	buf[1] = 0x08;//AIN0 Positive + AINCOM Negative
    
    /*
        A/D Control Register (Address 02h):
            Bit7:
                Reserved, always 0 (Read Only)
            Bit6~5:
                CLK1, CLK0: D0/CLKOUT Clock Out Rate Setting
                00 = Clock Out OFF
                01 = Clock Out Frequency = fCLKIN (default)
                10 = Clock Out Frequency = fCLKIN/2
                11 = Clock Out Frequency = fCLKIN/4
                When not using CLKOUT, it is recommended that it be turned off. These bits can only be reset using the RESET pin.
            Bit4~2:
                SDCS1, SCDS0: Sensor Detect Current Sources
                00 = Sensor Detect OFF (default)
                01 = Sensor Detect Current = 0.5μA
                10 = Sensor Detect Current = 2μA
                11 = Sensor Detect Current = 10μA
                The Sensor Detect Current Sources can be activated to verify the integrity of an external sensor supplying a signal to the
                ADS1255/6. A shorted sensor produces a very small signal while an open-circuit sensor produces a very large signal.
            Bit2~0: 
                PGA2, PGA1, PGA0: Programmable Gain Amplifier Setting
                000 = 1 (default)
                001 = 2
                010 = 4
                011 = 8
                100 = 16
                101 = 32
                110 = 64
                111 = 64
    */
    
	buf[2] = (0 << 5) | (0 << 3) | (gain << 0);//Clock Out off + Sensor Detect off + Gain

    /*
        DRATE: A/D Data Rate (Address 03h)
            Bit7~0: Data Rate Setting
                11110000 = 30,000SPS (default)
                11100000 = 15,000SPS
                11010000 = 7,500SPS
                11000000 = 3,750SPS
                10110000 = 2,000SPS
                10100001 = 1,000SPS
                10010010 = 500SPS
                10000010 = 100SPS
                01110010 = 60SPS
                01100011 = 50SPS
                01010011 = 30SPS
                01000011 = 25SPS
                00110011 = 15SPS
                00100011 = 10SPS
                00010011 = 5SPS
                00000011 = 2.5SPS
    */
	
	buf[3] = drate;//Drate

	chip_select();
    
	SendByte(CMD_WREG | 0);
	SendByte(0x03);//Write the last 4 bits
    
    /*
        Tips:
            When ADS1256 get WriteReg command,
            you can keep sending another 8bits data,
            it will put it to the next register
            STATUS->MUX->ADCON->DRATE
    */
    
	SendByte(buf[0]);//STATUS
	SendByte(buf[1]);//MUX
	SendByte(buf[2]);//ADCON
	SendByte(buf[3]);//DRATE

	chip_release();
	

	delay_ms(50);
}

int32_t GetADC(unsigned char ch)
{
	int32_t iTemp;

	if (ch > 7)
	{
		return 0;
	}

	iTemp = ADS1256.AdcNow[ch];

	return iTemp;
}

void ADC_ISR(void)
{

    Choosechannel(ADS1256.Channel);
	delay_ms(5);

    ADS1256_WriteCmd(CMD_SYNC);
	delay_ms(5);

	ADS1256_WriteCmd(CMD_WAKEUP);
	delay_ms(25);

	if (ADS1256.Channel == 0)
	{
		ADS1256.AdcNow[7] = ReadData();
	}
	else
	{
		ADS1256.AdcNow[ADS1256.Channel-1] = ReadData();
	}

	if (++ADS1256.Channel >= 8)
	{
		ADS1256.Channel = 0;
	}
}

void ADC_StartScan()
{
    int i;
    ADS1256.Channel = 0;
    
    for(i = 0; i < 8; i++)
        ADS1256.AdcNow[i] = 0;
    
}


unsigned char ADC_Scan(void)
{
	if (DRDY_IS_LOW())
	{
		ADC_ISR();
		return 1;
	}

	return 0;
}    

static void ADS1256_WriteCmd(unsigned char cmd)
{
	chip_select();
	SendByte(cmd);
	chip_release();
}

static void Choosechannel(unsigned char ch)
{
	if (ch > 7)
	{
		return;
	}
	WriteReg(REG_MUX, (ch << 4) | (1 << 3));	/* Bit3 = 1, AINN connection AINCOM */
}

void ADC_Init()
{
    bcm2835_spi_begin();//Start the SPI0 on raspberry pi
    bcm2835_spi_setBitOrder(BCM2835_SPI_BIT_ORDER_LSBFIRST );//
    bcm2835_spi_setDataMode(BCM2835_SPI_MODE1);//CPOL = 0, CPHA = 1  
    bcm2835_spi_setClockDivider(BCM2835_SPI_CLOCK_DIVIDER_1024);//Set the SPI clock divider and therefore the SPI clock speed.
    bcm2835_gpio_fsel(CS_PIN, BCM2835_GPIO_FSEL_OUTP);
    bcm2835_gpio_write(CS_PIN, HIGH);
    bcm2835_gpio_fsel(RST_PIN, BCM2835_GPIO_FSEL_OUTP);
    bcm2835_gpio_write(RST_PIN, HIGH);
    bcm2835_gpio_fsel(DRDY_PIN, BCM2835_GPIO_FSEL_INPT);
    bcm2835_gpio_set_pud(DRDY_PIN, BCM2835_GPIO_PUD_UP);//Set the DRDY_PIN as pull-up mode
}

void ADC_Stop()
{
    bcm2835_spi_end();
    bcm2835_close();
}

unsigned short Voltage_Convert(float Vref, float voltage)
{
	unsigned short _D_;
	_D_ = (unsigned short)(65536 * voltage / Vref);

	return _D_;
}


int  main()
{
    unsigned char id;
  	int32_t adc[8];
	int32_t volt[8];
	unsigned char i;
	unsigned char ch_num;
	int32_t iTemp;
    FILE *fp;
    int k,j;
    long int value;
    
    if (!bcm2835_init())
        return 1;

    ADC_Init();
    
    id = ReadID();
    
    if (id != 3)
	{
		printf("Error, ASD1256 Chip ID = 0x%d\r\n", (int)id);
	}
    
	else
	{
		printf("Ok, ASD1256 Chip ID = 0x%d\r\n", (int)id);
	}
    /*
  	ADC_Setting(AD_GAIN_1, DRATE_30000);
    ADC_StartScan();
	ch_num = 1;
    */
    for(j=0;j<60000;j++)
	{
	    while((ADC_Scan() == 0));
        
		for (i = 0; i < ch_num; i++)
		{
			adc[i] = GetADC(i);
            volt[i] = (adc[i] * 100) / 167;                 
		}

		for (i = 0; i < ch_num; i++)
		{
	        iTemp = volt[i];	
                    
			if (iTemp < 0)
			{
                iTemp = -iTemp;
       		  	printf("-%ld.%03ld %03ld V \r\n", iTemp /1000000, (iTemp%1000000)/1000, iTemp%1000);
			}
			else
			{
                printf("%ld.%03ld %03ld V \r\n", iTemp /1000000, (iTemp%1000000)/1000, iTemp%1000);                    
			}
		}
		delay_ms(10);
	}
            
    /*
    for(k = 0; k < ch_num; i++)
    {
        for(j = 0; j < 8000; j++)
        {
            if(DRDY_IS_LOW())
            {
                ADC_ISR();
                break;
            }
        }
    }
    
    while(1)
    {
        while((ADC_Scan() == 0));

        adc[0] = GetADC(0);
        buf[0] = ((uint32_t)adc[0] >> 16) & 0xFF;
        buf[1] = ((uint32_t)adc[0] >> 8) & 0xFF;
        buf[2] = ((uint32_t)adc[0] >> 0) & 0xFF;
        value =  (long)adc[0]; 
        printf("%d",value);
        delay_ms(1);
    }
    */
    ADC_Stop();

    return 0;
}

