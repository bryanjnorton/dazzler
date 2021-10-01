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
  return_val,
  error,
};

struct moto_cmd {
  int Xspeed;
  int X;
  int Yspeed;
  int Y;
};

const int BAUD_RATE = 9600;
CmdMessenger c = CmdMessenger(Serial,',',';','/');

void on_who_are_you(void)
{
  c.sendCmd(my_name_is,"Bob");
}

void on_Add_CMD(void)
{
  struct moto_cmd load_cmd;
  load_cmd.Xspeed = c.readBinArg<int>();
  load_cmd.X =      c.readBinArg<int>();
  load_cmd.Yspeed = c.readBinArg<int>();
  load_cmd.Y =      c.readBinArg<int>();

  xQueueSend(queue, &load_cmd, portMAX_DELAY);

  c.sendBinCmd(return_val,1);
}

void on_set_speed(void)
{
  struct moto_cmd load_cmd;
  int speed = c.readBinArg<int>();

  stepper.setMaxSpeed(1000);
  stepper.setSpeed(speed); 

  c.sendBinCmd(return_val,1);
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
  // c.attach(sum_two_ints,on_sum_two_ints);
  c.attach(on_unknown_command);
}

void setup()
{  
  queue = xQueueCreate( 100, sizeof( struct moto_cmd ) );

  stepper1.setMaxSpeed(5000.0);
  stepper1.setAcceleration(100000.0);
  stepper1.moveTo(3000);

  stepper2.setMaxSpeed(5000.0);
  stepper2.setAcceleration(100000.0);
  stepper2.moveTo(3000);

  struct moto_cmd load_cmd;
  load_cmd.Xspeed = 1;
  load_cmd.X = 1;
  load_cmd.Yspeed = 20.4;
  load_cmd.Y = 1;

  // stepper3.setMaxSpeed(300.0);
  // stepper3.setAcceleration(100.0);
  // stepper3.moveTo(1000000); 
  xQueueSend(queue, &load_cmd, portMAX_DELAY);

  Serial.begin(BAUD_RATE);
  attach_callbacks();    

}
void loop()
{
  struct moto_cmd pull_cmd;
  for(;;)
  {
    stepper1.runSpeed();
  }
}