#include <stdio.h>
#include "pico/stdlib.h"
// For ADC input:
#include "hardware/adc.h"
//#include "hardware/dma.h"

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

void spll_irq_handler()
{
  spll1.AC_input = (long)((adc_fifo_get() << 4) - 0x7fff) << 5; // to Q21
  SPLL_1ph_run_FUNC(&spll1);
  //spll1.theta[0]
  //InvSine=spll1.sin[0]<<3; // shift from Q21 to Q24
}

int main(void )
{
  stdio_init_all();

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
  irq_set_priority (ADC_IRQ_FIFO, PICO_HIGHEST_IRQ_PRIORITY);	
  irq_set_enabled(ADC_IRQ_FIFO, true);

  // Configure SPLL
  spll_lpf_coef1.B0_lf=B0_LPF;
  spll_lpf_coef1.B1_lf=B1_LPF;
  spll_lpf_coef1.A1_lf=A1_LPF;

  SPLL_1ph_init(GRID_FREQ,_IQ21((float)(1.0/ISR_FREQUENCY)),&spll1,spll_lpf_coef1);
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
