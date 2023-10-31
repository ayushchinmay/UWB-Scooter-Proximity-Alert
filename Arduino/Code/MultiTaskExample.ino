/* Basic Multi Threading Arduino Example
   This example code is in the Public Domain (or CC0 licensed, at your option.)
   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
// Please read file README.md in the folder containing this example.

#if CONFIG_FREERTOS_UNICORE
#define ARDUINO_RUNNING_CORE 0
#else
#define ARDUINO_RUNNING_CORE 1
#endif

#define BUZZ_PIN 12
#define LED_BUILTIN 13


// Define two tasks for Blink & AnalogRead.
void TaskBlink( void *pvParameters );
void TaskBuzzer( void *pvParameters );
TaskHandle_t buzz_read_task_handle;

// The setup function runs once when you press reset or power on the board.
void setup() {
  // Initialize serial communication at 115200 bits per second:
  Serial.begin(115200);
  // Set up two tasks to run independently.
  uint32_t blink_delay = 2000; // Delay between changing state on LED pin
  uint32_t buzz_delay = 2000; // Delay between changing state on Buzzer pin
  xTaskCreate(
    TaskBlink
    ,  "Task Blink" // A name just for humans
    ,  2048        // The stack size can be checked by calling `uxHighWaterMark = uxTaskGetStackHighWaterMark(NULL);`
    ,  (void*) &blink_delay // Task parameter which can modify the task behavior. This must be passed as pointer to void.
    ,  1  // Priority
    ,  NULL // Task handle is not used here - simply pass NULL
    );

  // This variant of task creation can also specify on which core it will be run (only relevant for multi-core ESPs)
  xTaskCreate(
    TaskBuzzer
    ,  "Task Buzzer"
    ,  2048  // Stack size
    ,  (void *) &buzz_delay  // When no parameter is used, simply pass NULL
    ,  2  // Priority
    ,  &buzz_read_task_handle // With task handle we will be able to manipulate with this task.
    );

  Serial.printf("Basic Multi Threading Arduino Example\n");
  // Now the task scheduler, which takes over control of scheduling individual tasks, is automatically started.
}

char input;
void loop(){
	if (Serial.available()) {
		input = Serial.read();

		if (input == 'q') {
			vTaskDelete(buzz_read_task_handle);
			buzz_read_task_handle = NULL;
		}
		if (input == 'r') {
			xTaskCreate(TaskBuzzer, "Task Buzzer", 2048, (void *) &buzz_delay, 2,  &buzz_read_task_handle);
			buzz_read_task_handle = !NULL;
		}
	}
}

/*--------------------------------------------------*/
/*---------------------- Tasks ---------------------*/
/*--------------------------------------------------*/

void TaskBlink(void *pvParameters){  // This is a task.
  uint32_t blink_delay = *((uint32_t*)pvParameters);

  // initialize digital LED_BUILTIN on pin 13 as an output.
  pinMode(LED_BUILTIN, OUTPUT);

  for (;;){ // A Task shall never return or exit.
    digitalWrite(LED_BUILTIN, HIGH);   // turn the LED on (HIGH is the voltage level)
    // arduino-esp32 has FreeRTOS configured to have a tick-rate of 1000Hz and portTICK_PERIOD_MS
    // refers to how many milliseconds the period between each ticks is, ie. 1ms.
	Serial.println("BLINK ON");
    delay(blink_delay);
    digitalWrite(LED_BUILTIN, LOW);    // turn the LED off by making the voltage HIGH
	Serial.println("BLINK OFF");
    delay(blink_delay);
  }
}

void TaskBuzzer(void *pvParameters){  // This is a task.
  uint32_t buzz_delay = *((uint32_t*)pvParameters);

  if (!adcAttachPin(BUZZ_PIN)) {
	Serial.printf("TaskAnalogRead cannot work because the given pin %d cannot be used for ADC - the task will delete itself.\n", BUZZ_PIN);
	buzz_read_task_handle = NULL;
	vTaskDelete(NULL);
  }
  pinMode(BUZZ_PIN, INPUT);
//   pinMode(BUZZ_PIN, OUTPUT);

  for (;;) {
	int adcVal = analogRead(BUZZ_PIN);
	Serial.println(adcVal);
	delay(100);
	// digitalWrite(BUZZ_PIN, HIGH);
	// Serial.println("BUZZ ON");
	// delay(buzz_delay);
	// digitalWrite(BUZZ_PIN, LOW);
	// Serial.println("BUZZ OFF");
	// delay(buzz_delay);
  }
}
