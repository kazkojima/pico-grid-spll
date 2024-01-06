#include <stdio.h>
#include "pico/stdlib.h"
// For ADC input:
#include "hardware/adc.h"
//#include "hardware/dma.h"
#include "pico/multicore.h"
#include "pico/sync.h"

#include "IQmathLib.h"
#include "SPLL_1ph.h"

// Software PLL for grid power line frequency
SPLL_1ph spll1;
SPLL_LPF_COEFF spll_lpf_coef1;

#define B0_LPF SPLL_Q(166.877556)
#define B1_LPF SPLL_Q(-166.322444)
#define A1_LPF SPLL_Q(-1.0)

#define GRID_FREQ 50
#define ISR_FREQUENCY 50000

#define PI 3.1415927

// ADC
#define CAPTURE_CHANNEL 0

// GPIO
#define NPATH_PIN 16
#define NPATH_MASK (0x1f << NPATH_PIN)
#define SMSP_PIN 23

#define TEST_PIN 15

static critical_section_t spll_critsec;

void spll_irq_handler()
{
  uint16_t val = adc_fifo_get();
  critical_section_enter_blocking(&spll_critsec);
  spll1.AC_input = (long)(val - 0x981) << 9; // to Q21
  //gpio_put(TEST_PIN, (spll1.AC_input > 0 ? 1 : 0));
  SPLL_1ph_run_FUNC(&spll1);
  critical_section_exit(&spll_critsec);
}

void core1_main(void);

int main(void)
{
  stdio_init_all();

  // Initialize the critical sections
  critical_section_init(&spll_critsec);

  // Send core 1 off to start driving the GPIO whilst we configure the ADC/SPLL.
  sleep_ms(5);
  multicore_reset_core1();
  sleep_ms(5);
  multicore_launch_core1(core1_main);

  // SMPS PWM always to reduce noise
  gpio_init(SMSP_PIN);
  gpio_set_dir(SMSP_PIN, GPIO_OUT);
  gpio_put(SMSP_PIN, 1);

  //gpio_init(TEST_PIN);
  //gpio_set_dir(TEST_PIN, GPIO_OUT);
  //gpio_put(TEST_PIN, 1);

  gpio_init_mask(NPATH_MASK);
  gpio_set_dir_out_masked(NPATH_MASK);

  // Init GPIO for analogue use: hi-Z, no pulls, disable digital input buffer.
  adc_gpio_init(26 + CAPTURE_CHANNEL);

  adc_init();
  adc_select_input(CAPTURE_CHANNEL);
  adc_fifo_setup(
    true,    // Write each completed conversion to the sample FIFO
    false,   // Enable DMA data request (DREQ)
    1,       // DREQ (and IRQ) asserted when at least 1 sample present
    false,   // Enable error bit
    false    // Shift each sample to 8 bits when pushing to FIFO
  );

  // Set divisor to 959. 48M/(1+959) = 50ksps
  adc_set_clkdiv(959);

  irq_set_exclusive_handler(ADC_IRQ_FIFO, spll_irq_handler);
  adc_irq_set_enabled(true);
  irq_set_priority (ADC_IRQ_FIFO, PICO_HIGHEST_IRQ_PRIORITY);	
  irq_set_enabled(ADC_IRQ_FIFO, true);

  // Configure SPLL
  spll_lpf_coef1.B0_lf=B0_LPF;
  spll_lpf_coef1.B1_lf=B1_LPF;
  spll_lpf_coef1.A1_lf=A1_LPF;

  SPLL_1ph_init(GRID_FREQ,SPLL_Q((float)(1.0/ISR_FREQUENCY)),&spll1,spll_lpf_coef1);
  float c1=0.1;
  float c2=0.00001;
  SPLL_1ph_notch_coeff_update(((float)(1.0/ISR_FREQUENCY)),(float)(2*PI*GRID_FREQ*2),(float)c2,(float)c1, &spll1);

  printf("Starting capture\n");
  adc_run(true);

  // Everything else from this point is interrupt-driven. The processor has
  // time to sit and think about its early retirement -- maybe open a bakery?
  while (true)
    tight_loop_contents();
}

#define Nmax 16
#define Npath 16

uint32_t npath_tbl[Nmax] = {
  0x10, 0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17,
  0x08, 0x09, 0x0a, 0x0b, 0x0c, 0x0d, 0x0e, 0x0f
};
  
void core1_main(void)
{
  int16_t n;
  int16_t n_old = -1;
  int32_t theta = -1;
  int32_t npath_coeff = SPLL_Q((float)(Npath/(2*PI)));
  while (true) {
    critical_section_enter_blocking(&spll_critsec);
    theta = spll1.theta[0];
    critical_section_exit(&spll_critsec);
    n = SPLL_Qint(SPLL_Qmpy(theta, npath_coeff));
    if (n == n_old)
      continue;
    n_old = n;
    gpio_put_masked(NPATH_MASK, (npath_tbl[n] << NPATH_PIN));
  }
}
