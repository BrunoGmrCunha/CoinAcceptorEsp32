#include <Arduino.h>

#include "driver/pcnt.h"

#define PCNT_TEST_UNIT PCNT_UNIT_0
#define PCNT_H_LIM_VAL 10
#define PCNT_L_LIM_VAL -10
#define PCNT_THRESH1_VAL 1
#define PCNT_THRESH2_VAL 4
#define PCNT_THRESH0_VAL -1
#define PCNT_INPUT_SIG_IO 13              // Pulse Input GPIO
#define PCNT_INPUT_CTRL_IO 5              // Control GPIO HIGH=count up, LOW=count down
#define LEDC_OUTPUT_IO 18                 // Output GPIO of a sample 1 Hz pulse generator
xQueueHandle pcnt_evt_queue;              // A queue to handle pulse counter events
pcnt_isr_handle_t user_isr_handle = NULL; //user's ISR service handle

/* A sample structure to pass events from the PCNT
 * interrupt handler to the main program.
 */
typedef struct
{
  int unit;        // the PCNT unit that originated an interrupt
  uint32_t status; // information on the event type that caused the interrupt
} pcnt_evt_t;

/* Decode what PCNT's unit originated an interrupt
 * and pass this information together with the event type
 * the main program using a queue.
 */
static void IRAM_ATTR pcnt_example_intr_handler(void *arg)
{
  uint32_t intr_status = PCNT.int_st.val;
  int i;
  pcnt_evt_t evt;
  portBASE_TYPE HPTaskAwoken = pdFALSE;

  for (i = 0; i < PCNT_UNIT_MAX; i++)
  {
    if (intr_status & (BIT(i)))
    {
      evt.unit = i;
      /* Save the PCNT event type that caused an interrupt
               to pass it to the main program */
      evt.status = PCNT.status_unit[i].val;
      PCNT.int_clr.val = BIT(i);
      xQueueSendFromISR(pcnt_evt_queue, &evt, &HPTaskAwoken);
      if (HPTaskAwoken == pdTRUE)
      {
        portYIELD_FROM_ISR();
      }
    }
  }
}
static void pcnt_example_init(void)
{
  /* Prepare configuration for the PCNT unit */
  pcnt_config_t pcnt_config;
  // Set PCNT input signal and control GPIOs
  pcnt_config.pulse_gpio_num = PCNT_INPUT_SIG_IO;
  pcnt_config.ctrl_gpio_num = PCNT_INPUT_CTRL_IO;
  pcnt_config.channel = PCNT_CHANNEL_0;
  pcnt_config.unit = PCNT_TEST_UNIT;
  // What to do on the positive / negative edge of pulse input?
  pcnt_config.pos_mode = PCNT_COUNT_INC; // Count up on the positive edge
  pcnt_config.neg_mode = PCNT_COUNT_DIS; // Keep the counter value on the negative edge
  // What to do when control input is low or high?
  pcnt_config.lctrl_mode = PCNT_MODE_REVERSE; // Reverse counting direction if low
  pcnt_config.hctrl_mode = PCNT_MODE_KEEP;    // Keep the primary counter mode if high
  // Set the maximum and minimum limit values to watch
  pcnt_config.counter_h_lim = PCNT_H_LIM_VAL;
  pcnt_config.counter_l_lim = PCNT_L_LIM_VAL;

  /* Initialize PCNT unit */
  pcnt_unit_config(&pcnt_config);

  /* Configure and enable the input filter */
  pcnt_set_filter_value(PCNT_TEST_UNIT, 100);
  pcnt_filter_enable(PCNT_TEST_UNIT);

  /* Set threshold 0 and 1 values and enable events to watch */
  //pcnt_set_event_value(PCNT_TEST_UNIT, PCNT_EVT_THRES_1, PCNT_THRESH1_VAL);
  //pcnt_event_enable(PCNT_TEST_UNIT, PCNT_EVT_THRES_1);
  // pcnt_set_event_value(PCNT_TEST_UNIT, PCNT_EVT_THRES_0, PCNT_THRESH0_VAL);
  //  pcnt_event_enable(PCNT_TEST_UNIT, PCNT_EVT_THRES_0);
  /* Enable events on zero, maximum and minimum limit values */
  //pcnt_event_enable(PCNT_TEST_UNIT, PCNT_EVT_ZERO);
  pcnt_event_enable(PCNT_TEST_UNIT, PCNT_EVT_H_LIM);
  pcnt_event_enable(PCNT_TEST_UNIT, PCNT_EVT_L_LIM);

  /* Initialize PCNT's counter */
  pcnt_counter_pause(PCNT_TEST_UNIT);
  pcnt_counter_clear(PCNT_TEST_UNIT);

  /* Register ISR handler and enable interrupts for PCNT unit */
  pcnt_isr_register(pcnt_example_intr_handler, NULL, 0, &user_isr_handle);
  pcnt_intr_enable(PCNT_TEST_UNIT);

  /* Everything is set up, now go to counting */
  pcnt_counter_resume(PCNT_TEST_UNIT);
}

void setup()
{
  Serial.begin(115200);
  pcnt_example_init();
}

void loop()
{
  int16_t count = 0;
  pcnt_get_counter_value(PCNT_TEST_UNIT, &count);
  if (count != 0)
  {
    delay(1000);
    pcnt_get_counter_value(PCNT_TEST_UNIT, &count);
    pcnt_counter_clear(PCNT_TEST_UNIT);
    if (count > 0)
    {
      Serial.println(count);
    }
    else
    {
      pcnt_counter_clear(PCNT_TEST_UNIT);
    }
  }
}