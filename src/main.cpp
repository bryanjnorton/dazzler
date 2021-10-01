#include <stdio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <driver/gpio.h>
#include "sdkconfig.h"
#include <Arduino.h>
#include "CmdMessenger.h"
#include <AccelStepper.h>

QueueHandle_t queue;

enum {
  who_are_you,
  my_name_is,
  Add_CMD,
  return_val1,
  set_speed,
  return_val2,
  error,
};

struct moto_cmd {
  int speed;
};

const int BAUD_RATE = 9600;
CmdMessenger c = CmdMessenger(Serial,',',';','/');

void on_who_are_you(void)
{
  c.sendCmd(my_name_is,"Bob");
}

void on_set_speed(void)
{
  struct moto_cmd load_cmd;
  load_cmd.speed = c.readBinArg<int>();
  xQueueSend(queue, &load_cmd, portMAX_DELAY);

  c.sendBinCmd(return_val2,1);
}
// #include "FastAccelStepper.h"

// #define dirPinStepper 18
// #define enablePinStepper 26
// #define stepPinStepper 17

// FastAccelStepperEngine engine = FastAccelStepperEngine();
// FastAccelStepper *stepper = NULL;

// void setup() {
//   // put your setup code here, to run once:

//   gpio_pad_select_gpio( (gpio_num_t) 2 );
//     /* Set the GPIO as a push/pull output */
//   gpio_set_direction( (gpio_num_t) 2, GPIO_MODE_OUTPUT);

//   engine.init();
//   stepper = engine.stepperConnectToPin(stepPinStepper);
//   if (stepper) {
//     stepper->setDirectionPin(dirPinStepper);
//     stepper->setEnablePin(enablePinStepper);
//     stepper->setAutoEnable(true);

//     // If auto enable/disable need delays, just add (one or both):
//     // stepper->setDelayToEnable(50);
//     // stepper->setDelayToDisable(1000);

//     stepper->setSpeedInUs(5);  // the parameter is us/step !!!
//     stepper->setAcceleration(10000000);
//     //stepper->moveTo(1000);
//   }
// }

// void loop() {
//   // put your main code here, to run repeatedly:

//   /* Blink off (output low) */
//   gpio_set_level((gpio_num_t)2, 0);
//   //vTaskDelay(1000 / portTICK_PERIOD_MS);
//   /* Blink on (output high) */
//   gpio_set_level((gpio_num_t)2, 1);
//   vTaskDelay(1000 / portTICK_PERIOD_MS);

//     stepper->moveTo(128*20);
//       vTaskDelay(1000 / portTICK_PERIOD_MS);

//     stepper->moveTo(-128*20);

//    // if (stepper->getCurrentPosition() == 0)
//    //   stepper->moveTo(-stepper->getCurrentPosition());
//    // stepper->keepRunning();
// }

AccelStepper stepper1(AccelStepper::DRIVER, 17, 18);
AccelStepper stepper2(AccelStepper::DRIVER, 15, 2);

void on_unknown_command(void)
{
  c.sendCmd(error,"Command without callback.");
}

void attach_callbacks(void) 
{   
  c.attach(who_are_you,on_who_are_you);
  c.attach(set_speed,on_set_speed);
  c.attach(on_unknown_command);
}

void setup()
{  

  stepper1.setMaxSpeed(5000.0);
  stepper1.setAcceleration(100000.0);
  stepper1.moveTo(3000);

  stepper2.setMaxSpeed(5000.0);
  stepper2.setAcceleration(100000.0);
  stepper2.moveTo(3000);

  struct moto_cmd load_cmd;
  load_cmd.speed = 1;

  // stepper3.setMaxSpeed(300.0);
  // stepper3.setAcceleration(100.0);
  // stepper3.moveTo(1000000); 

  Serial.begin(BAUD_RATE);
  attach_callbacks();    
}

void loop()
{
  struct moto_cmd pull_cmd;
  for(;;)
  {
    if( 1 == xQueueReceive(queue, &pull_cmd, portMAX_DELAY) )
    {
      stepper1.setMaxSpeed(1000);
      stepper1.setSpeed(pull_cmd.speed); 
    } else 
    {
      stepper1.runSpeed();
    }
  }
}