# ESP32 Deep Sleep Mode Tutorial


*Woojin Lee ECE 196 SP26*


---


## Brainstorm

Power management is a critical factor for any portable device that runs on battery. Implementing a "deep sleep" state allows the microcontroller to power down components that are not essential while keeping the RTC (Real-Time Clock) module active to monitor for specific triggers that wakes the board up. 


## Abstract

This tutorial demonstrates how to implement deep sleep mode on an ESP32-S3-Mini microcontroller using the Arduino IDE. We will configure a dual-button system: one physical button triggers the deep sleep, while a second button acts as an external hardware interrupt to wake the board up. 


## Introduction

The function of the deep sleep mode is to **save the battery** that would be powering the dev board. Because my final project is making a *portable* *pill* *box* that connects with an *app*, the battery life matters to the users. The longer the battery life, the better. The esp32 wakes up only when it needs to wake up and when it is not needed, it goes into **deep sleep mode** turning all unnecessary functions off to maximize the battery life.

![ESP32 deep sleep](https://i0.wp.com/randomnerdtutorials.com/wp-content/uploads/2019/03/ESP32-Deep-Sleep-and-wake-up-sources.jpg?resize=828%2C466&quality=100&strip=all&ssl=1)


### Tutorial Objectives

* **Configure Wake and Sleep Triggers:** Assign physical buttons to GPIO pins to manually enter and exit deep sleep.

* **Hardware Interfacing:** Correctly utilize RTC GPIO driver functions (`driver/rtc_io.h`) to maintain stable pin states during low-power modes.


### Materials

* The dev board that was made in the beginning of the quarter

* A computer with Arduino downloaded

* USB-C Cable for programming and serial monitoring


### Software/Imports

* **Environment:** Arduino IDE (Ensure "USB CDC On Boot" is enabled in the Tools menu for the ESP32-S3).

* **Libraries:**

  * `<Arduino.h>`: Standard Arduino framework.

  * `"driver/rtc_io.h"`: Required for configuring the RTC GPIO pins to function correctly as wake-up sources while the main CPU is powered off.



## Working Example


Below is the complete Arduino code. Notice the `delay(1500);` at the start of `setup()`. This is a crucial safety buffer that gives your computer time to establish a serial connection and allows you to upload new code before the board potentially goes back to sleep.



###Arduino Code


```cpp

#include <Arduino.h>

#include "driver/rtc_io.h"



#define WAKEUP_PIN GPIO_NUM_12 // SW1 - Used to wake up the board

#define SLEEP_PIN  GPIO_NUM_13 // SW2 - Used to trigger deep sleep



RTC_DATA_ATTR int bootCount = 0;



void setup() {

  Serial.begin(115200);

  delay(1500);



  bootCount++;

  Serial.printf("  BOARD AWAKE | STABLE BOOT COUNT: %d\n", bootCount);

 

  pinMode(SLEEP_PIN, INPUT_PULLDOWN);



  Serial.println("Waiting for you to press SW2 to go to sleep...");



  while (digitalRead(SLEEP_PIN) == LOW) {

    delay(50);

  }



  Serial.println("\n[SW2 Pressed!] Starting sleep sequence...");

  for (int i = 5; i > 0; i--) {

    Serial.printf("Going to deep sleep in %d seconds...\n", i);

    delay(1000);

  }



  esp_sleep_enable_ext0_wakeup(WAKEUP_PIN, 1);



  gpio_pulldown_en(WAKEUP_PIN);

  gpio_pullup_dis(WAKEUP_PIN);



  Serial.println("Going to sleep now. Press SW1 to wake me up !!!");

  Serial.flush();

 

  esp_deep_sleep_start();

}



void loop() {

}

```


###Results

![DeepSleep Result](https://postimg.cc/ZCJs7828)


##Additional Resources

https://www.instructables.com/ESP32-Deep-Sleep-Tutorial/

https://randomnerdtutorials.com/esp32-deep-sleep-arduino-ide-wake-up-sources/ 


##AI-Use Disclosure

Utilized Google Gemini to teach me how to insert all of my code line by line as a code block.