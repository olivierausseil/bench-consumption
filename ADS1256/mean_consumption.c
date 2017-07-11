#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <bcm2835.h>

// /* Unsigned integer types  */
// #define uint8_t unsigned char
// #define uint16_t unsigned short
// #define uint32_t unsigned long
//
// typedef enum {FALSE = 0, TRUE = !FALSE} bool;
typedef uint8_t ADS1256_DRATE;
typedef uint8_t ADS1256_GAIN;

// datarate
static const ADS1256_DRATE ADS1256_30000SPS = 0xF0;
static const ADS1256_DRATE ADS1256_15000SPS = 0xE0;
static const ADS1256_DRATE ADS1256_7500SPS  = 0xD0;
static const ADS1256_DRATE ADS1256_3750SPS  = 0xC0;
static const ADS1256_DRATE ADS1256_2000SPS  = 0xB0;
static const ADS1256_DRATE ADS1256_1000SPS  = 0xA1;
static const ADS1256_DRATE ADS1256_500SPS   = 0x92;
static const ADS1256_DRATE ADS1256_100SPS   = 0x82;
static const ADS1256_DRATE ADS1256_60SPS    = 0x72;
static const ADS1256_DRATE ADS1256_50SPS    = 0x63;
static const ADS1256_DRATE ADS1256_30SPS    = 0x53;
static const ADS1256_DRATE ADS1256_25SPS    = 0x43;
static const ADS1256_DRATE ADS1256_15SPS    = 0x33;
static const ADS1256_DRATE ADS1256_10SPS    = 0x20;
static const ADS1256_DRATE ADS1256_5SPS     = 0x13;
static const ADS1256_DRATE ADS1256_2d5SPS   = 0x03;

//gain
static const ADS1256_GAIN ADS1256_GAIN_1		= 0x00;
static const ADS1256_GAIN ADS1256_GAIN_2		=	0x01;
static const ADS1256_GAIN ADS1256_GAIN_4		=	0x02;
static const ADS1256_GAIN ADS1256_GAIN_8		=	0x03;
static const ADS1256_GAIN ADS1256_GAIN_16		= 0x04;
static const ADS1256_GAIN ADS1256_GAIN_32		=	0x05;
static const ADS1256_GAIN ADS1256_GAIN_64		= 0x06;

static const uint8_t ADS1256_CHANNEL_1      = 0x01;
static const uint8_t ADS1256_CHANNEL_2      = 0x02;
static const uint8_t ADS1256_CHANNEL_3      = 0x03;
static const uint8_t ADS1256_CHANNEL_4      = 0x04;
static const uint8_t ADS1256_CHANNEL_5      = 0x05;
static const uint8_t ADS1256_CHANNEL_6      = 0x06;
static const uint8_t ADS1256_CHANNEL_7      = 0x07;


// ---------------------  HAL --------------------------------------------
//-                                                                      -
//------------------------------------------------------------------------

#define DRDY  (RPI_GPIO_P1_11)  //P0
#define RST   (RPI_GPIO_P1_12)  //P1
#define	SPICS	(RPI_GPIO_P1_15)  //P3

void hal_init()
{
  bcm2835_init();
}

void hal_init_spi()
{
  bcm2835_spi_begin();
  bcm2835_spi_setBitOrder(BCM2835_SPI_BIT_ORDER_LSBFIRST );      // The default
  bcm2835_spi_setDataMode(BCM2835_SPI_MODE1);                   // The default
  bcm2835_spi_setClockDivider(BCM2835_SPI_CLOCK_DIVIDER_1024); // The default
}

void hal_init_gpio()
{
  bcm2835_gpio_fsel(SPICS, BCM2835_GPIO_FSEL_OUTP);//
  bcm2835_gpio_write(SPICS, HIGH);
  bcm2835_gpio_fsel(DRDY, BCM2835_GPIO_FSEL_INPT);
  bcm2835_gpio_set_pud(DRDY, BCM2835_GPIO_PUD_UP);
}

void hal_wait_us (uint32_t t)
{
  bcm2835_delayMicroseconds(t);
}

void hal_CS (uint8_t valueCS)
{

  if (valueCS == 0)
  {
    bcm2835_gpio_write(SPICS,LOW);
  }
  else
  {
    bcm2835_gpio_write(SPICS,HIGH);
  }
}

void hal_send8bit (uint8_t valueToSend)
{
  //printf ("0x%02X\n", valueToSend);
  hal_wait_us(1);
  bcm2835_spi_transfer(valueToSend);
}

uint8_t hal_receive8bit ()
{
  uint8_t receiveValue;
  receiveValue = bcm2835_spi_transfer(0xff);
  //printf ("0x%02X\n", receiveValue);
  return receiveValue;
}

void hal_data_ready()
{
  while (!bcm2835_gpio_lev(DRDY)==0); // a voir?
}


// ---------------------  ADS1256 acces layer  ---------------------------
//-                                                                      -
//------------------------------------------------------------------------
enum
{
	//Register definition�� Table 23. Register Map --- ADS1256 datasheet Page 30
	REG_STATUS = 0,
	REG_MUX    = 1,
	REG_ADCON  = 2,
	REG_DRATE  = 3,
	REG_IO     = 4,
	REG_OFC0   = 5,
	REG_OFC1   = 6,
	REG_OFC2   = 7,
	REG_FSC0   = 8,
	REG_FSC1   = 9,
	REG_FSC2   = 10,
};
/* Command definition�� TTable 24. Command Definitions --- ADS1256 datasheet Page 34 */
enum
{
	CMD_WAKEUP  = 0x00,	// Completes SYNC and Exits Standby Mode 0000  0000 (00h)
	CMD_RDATA   = 0x01, // Read Data 0000  0001 (01h)
	CMD_RDATAC  = 0x03, // Read Data Continuously 0000   0011 (03h)
	CMD_SDATAC  = 0x0F, // Stop Read Data Continuously 0000   1111 (0Fh)
	CMD_RREG    = 0x10, // Read from REG rrr 0001 rrrr (1xh)
	CMD_WREG    = 0x50, // Write to REG rrr 0101 rrrr (5xh)
	CMD_SELFCAL = 0xF0, // Offset and Gain Self-Calibration 1111    0000 (F0h)
	CMD_SELFOCAL= 0xF1, // Offset Self-Calibration 1111    0001 (F1h)
	CMD_SELFGCAL= 0xF2, // Gain Self-Calibration 1111    0010 (F2h)
	CMD_SYSOCAL = 0xF3, // System Offset Calibration 1111   0011 (F3h)
	CMD_SYSGCAL = 0xF4, // System Gain Calibration 1111    0100 (F4h)
	CMD_SYNC    = 0xFC, // Synchronize the A/D Conversion 1111   1100 (FCh)
	CMD_STANDBY = 0xFD, // Begin Standby Mode 1111   1101 (FDh)
	CMD_RESET   = 0xFE, // Reset to Power-Up Values 1111   1110 (FEh)
};

void ADS1256_init()
{
  hal_init();
  hal_init_spi();
  hal_init_gpio();
}

void ADS1256_wait_us(uint32_t t)
{
  hal_wait_us (t); //Delay from last SCLK edge for DIN to first SCLK rising edge for DOUT: RDATA, RDATAC,RREG Commands min  50   CLK = 50 * 0.13uS = 6.5uS
}

uint8_t ADS1256_read_register(uint8_t registerID)
{
  uint8_t read;
  hal_CS(0);  // SPI  CS = 0

  hal_send8bit(CMD_RREG | registerID); // Choose the resgister to read
  hal_send8bit(0x00); // Indicate the number of byte read

  ADS1256_wait_us(10);

  read = hal_receive8bit();

  hal_CS(1); // SPI  CS = 1
  return read;
}

uint8_t ADS1256_write_register(uint8_t registerID, uint8_t registerValue)
{
  hal_CS(0); // SPI  CS = 0
  int temporary;
  temporary = (CMD_WREG | registerID);
  //printf ("temporary = 0x%02X\n", temporary);
  hal_send8bit(temporary);
  hal_send8bit(0x00);
  //printf ("registerValue = 0x%02X\n", registerValue);
  hal_send8bit(registerValue);

  hal_CS(1); // SPI  CS = 1
}

void ADS1256_write_command(uint8_t cmd)
{
  hal_CS(0);
  hal_send8bit(cmd);
  hal_CS(1);
}

void ADS1256_data_ready()
{
  hal_data_ready();
}

void ADS1256_read_data_continuous (uint32_t sample)
{
uint32_t sample_counter;
uint8_t buf_read[3];
uint32_t valueData;
for (sample_counter = 0 ; sample_counter<sample ; sample_counter++)
  {
    hal_data_ready();
    hal_CS(0);
    buf_read[0] = hal_receive8bit();
    buf_read[1] = hal_receive8bit();
    buf_read[2] = hal_receive8bit();
    hal_CS(1);
    valueData = (buf_read[0] << 16) + (buf_read[1] << 8) + buf_read[2];
    printf("raw value channel = 0x%02X%02X%02X, %08ld\n", (int)buf_read[0],(int)buf_read[1], (int)buf_read[2], (long)valueData);
  }

}

// ---------------------  Driver ADS1256 layer ---------------------------
//-                                                                      -
//------------------------------------------------------------------------
void ADS1256_drv_init()
{
  ADS1256_init();
}

uint8_t ADS1256_drv_readID()
{
  uint8_t readID;
  ADS1256_data_ready();
  //printf ("readID = 0x%02X\n", readID);
  readID = ADS1256_read_register(REG_STATUS);
  return (readID >> 4);
}

void ADS1256_drv_configuration(ADS1256_DRATE sps, uint8_t gain, uint8_t channel)
{
  ADS1256_data_ready();

  // write on register status : ORDER = 0, Auto-Calibration --> ACAL = 1, buffer on --> BUFEN = 1
  uint8_t valueStatusRegister = 0x32;
  //printf ("valueStatusRegister = 0x%02X\n", valueStatusRegister);
  ADS1256_write_register(REG_STATUS, valueStatusRegister);
  valueStatusRegister = ADS1256_read_register(REG_STATUS);
  //printf ("valueStatusRegister = 0x%02X\n", valueStatusRegister);
  //printf("--------------------------------\n");

  // Multiplexer register
  uint8_t valueStatusMultiplexer = 0x08; // disabled negative channel by default
  valueStatusMultiplexer = ((channel << 4 ) | valueStatusMultiplexer);
  //printf ("muxSelectChannel = 0x%02X\n", valueStatusMultiplexer);
  ADS1256_write_register(REG_MUX, valueStatusMultiplexer );
  valueStatusMultiplexer = ADS1256_read_register(REG_MUX);
  //printf ("muxSelectChannel = 0x%02X\n", valueStatusMultiplexer);
  //printf("--------------------------------\n");

  // register ADCON
  uint8_t valueStatusADCON = 0x00; // no CLKOUT and no control current by default
  valueStatusADCON = (valueStatusADCON | gain);
  //printf ("valueStatusADCON = 0x%02X\n", valueStatusADCON);
  ADS1256_write_register(REG_ADCON, valueStatusADCON);
  valueStatusADCON = ADS1256_read_register(REG_ADCON);
  //printf ("valueStatusADCON = 0x%02X\n", valueStatusADCON);
  //printf("--------------------------------\n");

  // register datarate
  uint8_t valueSps = (uint8_t)(sps);
  //printf ("valueSps = 0x%02X\n", valueSps);
  ADS1256_write_register(REG_DRATE, valueSps);
  valueSps = ADS1256_read_register(REG_DRATE);
  //printf ("valueSps = 0x%02X\n", valueSps);
  //printf("--------------------------------\n");
}

void ADS1256_drv_run(uint32_t sample)
{
  ADS1256_write_command(CMD_WAKEUP);
  ADS1256_wait_us(5);
  ADS1256_write_command(CMD_SYNC);
  ADS1256_wait_us(5);
  ADS1256_write_command(CMD_RDATAC);
  ADS1256_read_data_continuous(sample);
  ADS1256_write_command(CMD_SDATAC);
}


// ---------------------  Application layer ------------------------------
//-                                                                      -
//------------------------------------------------------------------------

void mean_consumption_bench_init(){
  //printf ("init\n");
  ADS1256_drv_init();
}

uint8_t mean_consumption_bench_read_ID()
{
  uint8_t chipID;
  chipID = ADS1256_drv_readID();
  printf ("ID = 0x%02X\n", chipID);
  if (!chipID == 0x03)
  {
    return false;
  }
  else
  {
    return true;
  }
}

void mean_consumption_bench_configure( ADS1256_DRATE sps, uint8_t gain, uint8_t channel )
{
  //printf ("configuration\n");
  ADS1256_drv_configuration(sps, gain, channel);
}

void mean_consumption_bench_calibrate()
{
  printf ("calibrate\n");
}

void mean_consumption_bench_run(uint32_t samples_number )
{

  ADS1256_drv_run(samples_number);
  //printf ("run\n");
}

void main(void)
{
  uint8_t channel = ADS1256_CHANNEL_7;
  ADS1256_GAIN gain =  ADS1256_GAIN_1;
  ADS1256_DRATE sps = ADS1256_1000SPS;
  uint32_t samples_number = 5000;

  // initialize hardware
  mean_consumption_bench_init();

  // check communication with ADC
  if ( ! mean_consumption_bench_read_ID() )
  {
    printf("error to communicate with chip\n");
    return;
  }

  // configure acquisition channel, gain and datarate
  mean_consumption_bench_configure( sps, gain, channel );

  // perform calibration process
  mean_consumption_bench_calibrate();

  // run acquisition loop
  mean_consumption_bench_run(samples_number);
}
