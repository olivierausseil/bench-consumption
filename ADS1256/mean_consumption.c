#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <bcm2835.h>

// /* Unsigned integer types  */
// #define uint8_t unsigned char
// #define uint16_t unsigned short
// #define uint32_t unsigned long
//
// typedef enum {FALSE = 0, TRUE = !FALSE} bool;

// ---------------------  HAL -------------------------------------------
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

// ---------------------  ADS1256 acces layer  --------------------------

void ADS_1256_init()
{
  hal_init();
  hal_init_spi();
  hal_init_gpio();
}

// ---------------------  Driver ADS1256 layer --------------------------

void ADS1256_drv_init()
{
  ADS_1256_init();
}

// ---------------------  Application layer -----------------------------

typedef enum
{
	ADS1256_30000SPS = 0,
	ADS1256_15000SPS,
	ADS1256_7500SPS,
	ADS1256_3750SPS,
	ADS1256_2000SPS,
	ADS1256_1000SPS,
	ADS1256_500SPS,
	ADS1256_100SPS,
	ADS1256_60SPS,
	ADS1256_50SPS,
	ADS1256_30SPS,
	ADS1256_25SPS,
	ADS1256_15SPS,
	ADS1256_10SPS,
	ADS1256_5SPS,
	ADS1256_2d5SPS,

	ADS1256_DRATE_MAX
}ADS1256_DRATE_E;

void mean_consumption_bench_init()
{
    //printf ("init\n");
    ADS1256_drv_init();
}

bool mean_consumption_bench_read_ID()
{
    printf ("readID\n");
    return true;
}

void mean_consumption_bench_configure( ADS1256_DRATE_E sps, uint8_t channel )
{
    printf ("configuration\n");
}

void mean_consumption_bench_calibrate()
{
    printf ("calibrate\n");
}

void mean_consumption_bench_run()
{
    printf ("run\n");
}

void main(void)
{
  uint8_t channel = 7;
  ADS1256_DRATE_E sps = ADS1256_100SPS;
  uint32_t samples_number = 500;

  // initialize hardware
  mean_consumption_bench_init();

  // check communication with ADC
  if ( ! mean_consumption_bench_read_ID() )
  {
    printf("error message\n");
    return;
  }

  // configure acquisition channel and datarate
  mean_consumption_bench_configure( sps, channel );

  // perform calibration process
  mean_consumption_bench_calibrate();

  // run acquisition loop
  mean_consumption_bench_run( samples_number );
}
