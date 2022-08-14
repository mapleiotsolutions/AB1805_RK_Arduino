# AB1805_RK

*Library for AB1805/AM1805 RTC/Watchdog for Arduino devices*

This was an original library for Particle Devices forked and updated to use on Arduino devices. 
The original Particle Version is avalable here: [browse the generated API documentation](https://rickkas7.github.io/AB1805_RK/index.html).

## Examples

### 01-minimal

This is just the minimal implement of using the RTC and Watchdog Timer (WDT). Here's the complete code:

```cpp
#include "AB1805_RK.h"
#define IRQ 8 

SYSTEM_THREAD(ENABLED);
SYSTEM_MODE(SEMI_AUTOMATIC);

SerialLogHandler logHandler;

AB1805 ab1805(Wire);

//Dummy routine needed for interrupt awake. Could be used to set a flag that we woke up via LoRa Packet recieved nterrupt?
void wakeUp() {
}

void setup() {
    
    //Setup an ionterrupt on pin IRQ
    pinMode(IRQ, INPUT_PULLUP);
    LowPower.attachInterruptWakeup(IRQ, wakeUp, FALLING);
    
    //Setup withFOUT as the interrupt PIN
    ab1805.withFOUT(IRQ).setup();

    // Reset the AB1805 configuration to default values
    ab1805.resetConfig();

    //Set the RTC with an initial time. Will update this again with the first message recieved
    ab1805.setRtcFromTime(1600000000);

    // Enable watchdog
    ab1805.setWDT(ab1805.WATCHDOG_MAX_SECONDS);

}

void loop() {
    // Be sure to call ab1805.loop() on every call to loop() to pet the watchdog
    ab1805.loop();

    delay(1000);
    ab1805.getRtcAsTime(time_cv);
    Log.infoln("Time from RTC is: %l", (unsigned long) time_cv);

    // Set an interrupt for 5 seconds in the future. We should wake up 
    ab1805.interruptCountdownTimer(5, false);
    LowPower.deepSleep(60000);
}

```

Things to note in this code:

Declare an `AB1805` object in your code as a global variable. Only do this once in your main source file. The parameter is the I2C interface the AB1805 is connected to, typically `Wire` (D0/D1).

```cpp
AB1805 ab1805(Wire);
```

In setup(), call the `ab1805.setup()` method.

```cpp
    ab1805.setup();
```

In setup(), when using interrupts you can call this withFOUT indicating the interrupt pin call the `ab1805.withFOUT(IRQ).setup()` method.

```cpp
    ab1805.withFOUT(IRQ).setup();
```

Reset the settings on the AB1805. This isn't strictly necessary since it resets the chip to power-on defaults, but it's not a bad idea to be safe:

```cpp
    ab1805.resetConfig();
```

If you want to use the hardware watchdog, enable it:

```cpp
    ab1805.setWDT(AB1805::WATCHDOG_MAX_SECONDS);
```

And from loop(), make sure you call the loop method:

```cpp
    ab1805.loop();
```

The `ab1805.loop()` method takes care of:
- Serving the watchdog timer.

You can call various functions to set different types of interrupts. For example. to set an interrupt 5 seconds in the future call `ab1805.interruptCountdownTimer(5, false)`

```cpp
    ab1805.interruptCountdownTimer(5, false);
```

