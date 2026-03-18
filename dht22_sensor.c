// ============================================================
//  FILE:    dht22_sensor.c
//  PROJECT: Gearbox Embedded Systems Program
//  AUTHOR:  Kiprop Yego
//  BOARD:   Raspberry Pi Pico (RP2040)
//  SENSOR:  DHT22 Temperature & Humidity Sensor
//
//  WHAT THIS PROGRAM DOES:
//  Reads temperature and humidity from a DHT22 sensor every
//  2 seconds and prints the values over USB serial (UART).
//  You can see the output in VS Code's Serial Monitor.
//
//  WIRING:
//  DHT22 Pin 1 (VCC)  --> Pico 3.3V  (Pin 36)
//  DHT22 Pin 2 (DATA) --> Pico GP15  (Pin 20)  + 10kΩ pullup to 3.3V
//  DHT22 Pin 4 (GND)  --> Pico GND   (Pin 38)
// ============================================================


// ── SECTION 1: INCLUDES ──────────────────────────────────────────
//
// #include tells the PREPROCESSOR (first stage of compilation)
// to copy the contents of that header file into this file
// BEFORE the compiler sees it.
//
// Think of it like: "paste the contents of that file here"
//
// "pico/stdlib.h"  — Pico Standard Library
//                    Gives us: gpio_init(), sleep_ms(), stdio_init_all()
//                    This is NOT the same as the normal C stdlib.
//                    It is Raspberry Pi's version for the RP2040 chip.
//
// "hardware/gpio.h" — Low-level GPIO hardware driver
//                     Gives us direct access to the GPIO registers
//                     inside the RP2040 chip.
//
// "hardware/timer.h" — Hardware timer driver
//                      Gives us: time_us_32() — read the hardware clock
//                      The RP2040 has a 64-bit hardware timer counting
//                      microseconds since boot.
//
// "stdio.h"  — Standard C input/output
//              Gives us: printf() — print text to the serial port
//              On the Pico, printf() sends text over USB.
//
// "stdint.h" — Standard integer types
//              Gives us: uint8_t, uint16_t, uint32_t, int32_t
//              These are EXACT-width integer types.
//              uint8_t = unsigned integer, exactly 8 bits (0 to 255)
//              We use these instead of "int" because on embedded
//              systems, knowing the EXACT size matters.
//              A plain "int" could be 16 or 32 bits depending on
//              the compiler and architecture — dangerous!
//
// "stdbool.h" — Standard boolean type
//               Gives us: bool, true, false
//               In C, there is no built-in bool before C99.
//               This header adds it.

#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/timer.h"
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>


// ── SECTION 2: CONSTANTS (PREPROCESSOR DEFINES) ──────────────────
//
// #define creates a MACRO — a name that the preprocessor replaces
// with its value BEFORE compilation. The compiler never sees the
// name DHT22_PIN — it only ever sees the number 15.
//
// WHY use #define instead of a variable?
// 1. No memory used — a variable would occupy RAM. A #define
//    costs nothing — it is just text substitution.
// 2. Cannot be accidentally changed at runtime.
// 3. Convention: constants are UPPERCASE by tradition.
//
// DHT22_PIN: GP15 — the GPIO pin connected to the sensor DATA line.
//            GP15 is Pin 20 on the physical Pico board.
//            The "GP" means "General Purpose" — it can be INPUT or OUTPUT.
//
// DHT22_MAX_DATA_BITS: The DHT22 sends exactly 40 bits of data per reading.
//            5 bytes x 8 bits = 40 bits:
//            Byte 0: Humidity integer part
//            Byte 1: Humidity decimal part
//            Byte 2: Temperature integer part
//            Byte 3: Temperature decimal part
//            Byte 4: Checksum (to verify data integrity)
//
// TIMEOUT_US: If the sensor does not respond within 1000 microseconds
//             (1 millisecond), something is wrong. We stop waiting
//             to avoid hanging the whole program forever.

#define DHT22_PIN           15
#define DHT22_MAX_DATA_BITS 40
#define TIMEOUT_US          1000


// ── SECTION 3: DATA STRUCTURE ────────────────────────────────────
//
// A STRUCT (structure) groups related data together under one name.
// Think of it like a custom data type you design yourself.
//
// WHY use a struct here instead of separate variables?
// Because temperature and humidity BELONG together — they describe
// one sensor reading at one moment in time. A struct keeps them
// together logically and makes the code readable.
//
// This is the foundation of "data structures" in programming:
// organising data to reflect real-world relationships.
//
// MEMORY LAYOUT of DHT22Data in RAM:
//
//  Address  | Field       | Size    | Range
//  ---------|-------------|---------|------------------
//  +0       | temperature | 4 bytes | -9999 to +9999  (in tenths: 251 = 25.1°C)
//  +4       | humidity    | 4 bytes | 0 to 1000       (in tenths: 653 = 65.3%)
//  +8       | valid       | 1 byte  | 0 (false) or 1 (true)
//  ---------|-------------|---------|------------------
//  TOTAL    |             | 9 bytes | (may be padded to 12 by compiler)
//
// int32_t temperature:
//   Signed 32-bit integer. We store temperature * 10 to avoid floats.
//   25.1°C is stored as 251. This is a common embedded trick —
//   floating point math is expensive on microcontrollers without FPU.
//   The RP2040 HAS a floating point unit, but this teaches good habits.
//
// int32_t humidity:
//   Same trick. 65.3% stored as 653.
//
// bool valid:
//   Did the reading succeed? If the checksum failed or the sensor
//   did not respond, valid = false. We must ALWAYS check this before
//   trusting the data.

typedef struct {
    int32_t temperature;   // Temperature x10 (e.g., 251 means 25.1 degrees C)
    int32_t humidity;      // Humidity x10    (e.g., 653 means 65.3%)
    bool    valid;         // true if reading succeeded, false if error
} DHT22Data;


// ── SECTION 4: HELPER FUNCTION — WAIT FOR PIN STATE ──────────────
//
// FUNCTIONS are named blocks of code that:
// 1. Take INPUT (parameters)
// 2. Do a job
// 3. Return an OUTPUT (return value)
//
// This function waits for a GPIO pin to reach a target state
// (HIGH or LOW) but gives up after TIMEOUT_US microseconds.
//
// RETURN TYPE: int
//   We return the number of microseconds we waited.
//   If we timed out, we return -1 (a sentinel value meaning "error").
//   Returning -1 is a C convention for "something went wrong".
//
// PARAMETERS:
//   uint gpio   — which pin to watch (a number 0-28 on the Pico)
//   bool level  — what state to wait for: true=HIGH, false=LOW
//
// WHAT HAPPENS INSIDE THE CHIP WHEN WE CALL gpio_get():
//   The RP2040 has a register called GPIO_IN at memory address 0xD0000004.
//   Each bit in that 32-bit register represents one GPIO pin.
//   gpio_get(15) reads that register and checks bit 15.
//   If bit 15 is 1 = HIGH (3.3V), if 0 = LOW (0V).
//   This all happens in ONE CPU clock cycle — 7.5 nanoseconds at 133MHz.
//
// WHY uint32_t for start and elapsed?
//   time_us_32() returns an unsigned 32-bit value.
//   It counts microseconds since boot.
//   At 32 bits unsigned, it overflows (wraps to 0) after ~71 minutes.
//   That is fine for our 1ms timeout check.

static int wait_for_level(uint gpio, bool level) {
    uint32_t start   = time_us_32();   // Record the start time (in microseconds)
    uint32_t elapsed = 0;              // How long we have been waiting

    // LOOP: keep checking the pin until it reaches our target level
    // OR until we have waited too long (timeout).
    //
    // This is a POLLING loop — the CPU keeps asking "are we there yet?"
    // over and over. This is simple but uses 100% CPU during the wait.
    // An alternative is INTERRUPTS (we will cover those later).
    //
    // gpio_get() returns 1 (HIGH) or 0 (LOW)
    // We compare to our target level (which is bool: true or false)
    // When they match, we exit the loop.

    while (gpio_get(gpio) != (uint)level) {
        elapsed = time_us_32() - start;   // How many microseconds have passed?

        if (elapsed >= TIMEOUT_US) {      // Have we waited too long?
            return -1;                    // Return -1 = timeout error
        }
    }

    // If we get here, the pin reached the target level.
    // Return how many microseconds it took — the caller may need this.
    return (int)(time_us_32() - start);
}


// ── SECTION 5: CORE FUNCTION — READ THE DHT22 SENSOR ─────────────
//
// This is the most important function. It implements the DHT22
// COMMUNICATION PROTOCOL — the specific sequence of signals the
// sensor expects before it sends data.
//
// RETURN TYPE: DHT22Data
//   We return our custom struct. The entire 9 bytes of the struct
//   are copied onto the call stack and returned to the caller.
//
// HOW THE DHT22 PROTOCOL WORKS (this IS the hardware layer):
//
//  1. HOST SENDS START SIGNAL:
//     Pull DATA line LOW for at least 1ms (we use 1.2ms to be safe)
//     Release DATA line HIGH
//
//  2. SENSOR RESPONDS:
//     Pulls LOW for 80us  ← "I heard you"
//     Pulls HIGH for 80us ← "Get ready"
//
//  3. SENSOR SENDS 40 BITS:
//     Each bit starts with a LOW pulse of ~50us
//     Then a HIGH pulse:
//       - HIGH for ~26-28us = BIT 0
//       - HIGH for ~70us    = BIT 1
//     We measure the HIGH pulse duration to determine 0 or 1.
//
//  4. CHECKSUM VERIFICATION:
//     Byte4 = Byte0 + Byte1 + Byte2 + Byte3 (lowest 8 bits)
//     If it does not match, the data is corrupted — discard it.

DHT22Data read_dht22(void) {

    // Create a result struct and initialise it.
    // Start with valid = false — we will only set it true if
    // everything succeeds. This is DEFENSIVE PROGRAMMING.
    DHT22Data result = {0, 0, false};

    // raw_bits: array to store the 40 bits received from the sensor.
    //
    // ARRAY: a contiguous block of memory holding multiple values
    // of the same type.
    //
    // MEMORY: uint8_t raw_bits[40] allocates 40 bytes on the STACK.
    // The stack is a region of RAM for temporary local variables.
    // When this function returns, these 40 bytes are automatically freed.
    //
    // We use uint8_t (8-bit) even though each value is just 0 or 1.
    // Why? Because C has no "1-bit" type. uint8_t is the smallest
    // available. We could pack 8 bits into one byte for efficiency,
    // but clarity matters more in a teaching context.
    uint8_t raw_bits[DHT22_MAX_DATA_BITS] = {0};

    // ── PHASE 1: SEND START SIGNAL ──────────────────────────────
    //
    // We must DRIVE the pin LOW first.
    // To do this we:
    // 1. Set the pin direction to OUTPUT (we want to control the voltage)
    // 2. Write LOW (0V) to the pin
    // 3. Wait 1.2ms — the sensor needs at least 1ms of LOW
    // 4. Write HIGH (3.3V) — release the line
    // 5. Switch back to INPUT — now we listen for the sensor's response
    //
    // WHAT HAPPENS IN THE CHIP:
    // gpio_set_dir() writes to the GPIO_OE (Output Enable) register.
    // gpio_put() writes to the GPIO_OUT register.
    // These are hardware registers at fixed memory addresses.
    // Writing a 1 to bit 15 of GPIO_OE makes GP15 an output.
    // Writing a 0 to bit 15 of GPIO_OUT pulls GP15 to 0V (LOW).

    gpio_set_dir(DHT22_PIN, GPIO_OUT);   // Set GP15 as OUTPUT
    gpio_put(DHT22_PIN, 0);              // Pull LOW — start signal to sensor
    sleep_ms(1);                         // Hold LOW for 1.2ms
    gpio_put(DHT22_PIN, 1);              // Release HIGH
    sleep_us(30);                        // Wait 30us (datasheet spec: 20-40us)
    gpio_set_dir(DHT22_PIN, GPIO_IN);    // Switch to INPUT — listen for sensor

    // Enable the internal pull-up resistor.
    // WHY? The DHT22 uses an "open-drain" output — it can only pull
    // the line LOW. It cannot drive HIGH. The pull-up resistor (either
    // external 10kΩ or this internal one) pulls the line HIGH when
    // the sensor is not driving it LOW.
    gpio_pull_up(DHT22_PIN);

    // ── PHASE 2: WAIT FOR SENSOR RESPONSE ───────────────────────
    //
    // The sensor should respond by pulling LOW for ~80us
    // then releasing HIGH for ~80us.
    // We use our wait_for_level() function to detect this.
    //
    // If wait_for_level() returns -1, the sensor did not respond.
    // We return our result struct with valid=false.

    // Wait for sensor to pull LOW (start of response)
    if (wait_for_level(DHT22_PIN, false) < 0) {
        return result;   // Sensor did not respond — return invalid result
    }

    // Wait for sensor to release HIGH (end of response low pulse)
    if (wait_for_level(DHT22_PIN, true) < 0) {
        return result;
    }

    // Wait for sensor to pull LOW again (end of response high pulse)
    // This is the transition into the first data bit
    if (wait_for_level(DHT22_PIN, false) < 0) {
        return result;
    }

    // ── PHASE 3: READ 40 DATA BITS ──────────────────────────────
    //
    // FOR LOOP: execute the body exactly DHT22_MAX_DATA_BITS (40) times.
    //
    // LOOP VARIABLE: i is a counter.
    // int i = 0      — start at 0
    // i < 40         — keep going while i is less than 40
    // i++            — add 1 to i after each iteration
    //
    // ITERATIONS: i goes 0, 1, 2, 3, ... 39 → 40 total iterations.

    for (int i = 0; i < DHT22_MAX_DATA_BITS; i++) {

        // Each bit starts with a LOW pulse of ~50us.
        // Wait for this LOW to finish (wait for HIGH to begin).
        if (wait_for_level(DHT22_PIN, true) < 0) {
            return result;   // Timeout — corrupt transmission
        }

        // Now measure how long the HIGH pulse lasts.
        // HIGH < 35us  = BIT 0
        // HIGH > 35us  = BIT 1
        // (Datasheet: 0 = 26-28us,  1 = 70us)
        //
        // We use 35us as our threshold — comfortably between the two.

        int high_duration = wait_for_level(DHT22_PIN, false);

        if (high_duration < 0) {
            return result;   // Timeout — corrupt transmission
        }

        // Store the bit.
        // If HIGH lasted more than 35 microseconds → it is a 1
        // Otherwise → it is a 0
        raw_bits[i] = (high_duration > 35) ? 1 : 0;

        // TERNARY OPERATOR: condition ? value_if_true : value_if_false
        // This is shorthand for an if/else. It is one expression.
    }

    // ── PHASE 4: ASSEMBLE BITS INTO BYTES ───────────────────────
    //
    // We have 40 individual bits in raw_bits[].
    // We need to assemble them into 5 bytes (8 bits each).
    //
    // HOW BIT ASSEMBLY WORKS:
    //
    // For byte 0 (humidity integer): raw_bits[0..7]
    //   raw_bits[0] is the MOST SIGNIFICANT BIT (MSB)
    //   raw_bits[7] is the LEAST SIGNIFICANT BIT (LSB)
    //
    // Example: bits are 0,0,0,0,0,0,1,0 = 0b00000010 = 2
    //
    // To assemble:
    //   byte = 0
    //   For each bit from MSB to LSB:
    //     byte = (byte << 1) | bit
    //
    //   bit-shift LEFT by 1: multiplies by 2, makes room for next bit
    //   OR with bit: sets the lowest bit to the new value
    //
    // This is BITWISE MANIPULATION — working directly with the binary
    // representation of numbers. This is core to embedded programming.
    //
    // DECLARE 5 bytes — one for each chunk of 8 bits:
    uint8_t bytes[5] = {0, 0, 0, 0, 0};

    for (int b = 0; b < 5; b++) {          // For each of the 5 bytes
        for (int bit = 0; bit < 8; bit++) { // For each of the 8 bits in that byte
            bytes[b] = (bytes[b] << 1) | raw_bits[b * 8 + bit];
            //          ^^^^^^^^^^^         shift existing bits left (make room)
            //                        ^^^  OR in the new bit at the lowest position
        }
    }

    // bytes[0] = Humidity    integer part (e.g., 65 for 65.3%)
    // bytes[1] = Humidity    decimal part (e.g.,  3 for 65.3%)
    // bytes[2] = Temperature integer part (e.g., 25 for 25.1°C)
    // bytes[3] = Temperature decimal part (e.g.,  1 for 25.1°C)
    // bytes[4] = Checksum

    // ── PHASE 5: VERIFY CHECKSUM ─────────────────────────────────
    //
    // CHECKSUM: a simple error-detection mechanism.
    // The DHT22 sends the sum of the first 4 bytes as the 5th byte.
    // If our received checksum matches, the data is (probably) correct.
    //
    // We mask with & 0xFF to keep only the lowest 8 bits of the sum,
    // because the 4 bytes might add up to more than 255.
    //
    // 0xFF in binary = 11111111 — the AND operation keeps only
    // the lowest 8 bits, discarding any carry into higher bits.
    //
    // BITWISE AND (&): for each bit position:
    //   1 & 1 = 1
    //   1 & 0 = 0
    //   0 & 0 = 0
    // So X & 0xFF zeroes out all bits except the lowest 8.

    uint8_t checksum = (bytes[0] + bytes[1] + bytes[2] + bytes[3]) & 0xFF;

    if (checksum != bytes[4]) {
        // Checksum mismatch — data was corrupted during transmission.
        // Return invalid result. Do NOT use this data.
        return result;
    }

    // ── PHASE 6: COMBINE BYTES INTO FINAL VALUES ─────────────────
    //
    // Temperature is split across bytes[2] (integer) and bytes[3] (decimal).
    // We combine them: temperature = bytes[2] * 10 + bytes[3]
    // Example: bytes[2]=25, bytes[3]=1 → temperature = 251 (means 25.1°C)
    //
    // BUT: the DHT22 can report NEGATIVE temperatures.
    // The MSB (most significant bit) of bytes[2] is a sign flag:
    //   If bit 7 of bytes[2] is 1 → temperature is negative
    //   We then clear that bit and negate the result.
    //
    // BITWISE CHECK: bytes[2] & 0x80
    //   0x80 = 10000000 in binary
    //   ANDing with 0x80 isolates bit 7.
    //   If result is non-zero, bit 7 was set → negative temperature.
    //
    // BITWISE CLEAR: bytes[2] & 0x7F
    //   0x7F = 01111111 in binary
    //   ANDing clears bit 7, keeping bits 0-6.

    bool temp_negative = (bytes[2] & 0x80) != 0;   // Check sign bit
    bytes[2] &= 0x7F;                               // Clear sign bit

    result.temperature = (int32_t)bytes[2] * 10 + (int32_t)bytes[3];
    result.humidity    = (int32_t)bytes[0] * 10 + (int32_t)bytes[1];

    if (temp_negative) {
        result.temperature = -result.temperature;   // Apply negative sign
    }

    // All checks passed — mark this reading as valid.
    result.valid = true;

    return result;   // Return the populated struct to the caller
}


// ── SECTION 6: MAIN FUNCTION ─────────────────────────────────────
//
// main() is SPECIAL. It is the ENTRY POINT of the program.
// When the Pico boots, the startup code (in crt0.S, written in
// assembly) initialises the stack, zeroes out BSS (uninitialised
// globals), then calls main().
//
// After main(), the program ends. On a microcontroller, we NEVER
// want main() to return — there is nowhere to "go back to".
// That is why we have a while(true) loop at the bottom.
//
// RETURN TYPE: int
// The C standard requires main() to return int.
// On the Pico, this return value is never used, but we follow
// the standard. We put "return 0;" at the bottom even though
// the while loop means we never reach it. This makes the
// compiler happy and is good practice.

int main(void) {

    // Initialise USB serial (UART over USB).
    // This sets up the hardware so that printf() sends text
    // through the USB cable to your computer's Serial Monitor.
    //
    // WHAT HAPPENS INSIDE:
    // The RP2040 has a USB controller peripheral. stdio_init_all()
    // configures it and sets up interrupt handlers so data
    // can flow between the chip and your computer.
    stdio_init_all();

    // Wait 2 seconds for the USB serial connection to establish.
    // When you plug in the Pico, the PC needs a moment to recognise
    // the USB device and open the serial port.
    // Without this delay, the first few printf() messages get lost.
    sleep_ms(2000);

    // Print a header message so we know the program started.
    // printf() works exactly like in standard C:
    // %s = string, %d = integer, \n = newline character
    printf("========================================\n");
    printf("  Gearbox Embedded - DHT22 Sensor Demo  \n");
    printf("  Reading from GP%d every 2 seconds      \n", DHT22_PIN);
    printf("========================================\n\n");

    // Initialise GP15 as a GPIO pin.
    // This tells the RP2040's GPIO hardware to activate the pin.
    // Before this call, the pin is in an undefined state.
    gpio_init(DHT22_PIN);

    // READING COUNTER — tracks how many readings we have taken.
    // uint32_t: unsigned 32-bit integer.
    // At one reading every 2 seconds, this would overflow after
    // 4,294,967,295 * 2 seconds = ~272 years. We are fine.
    uint32_t reading_count = 0;

    // MAIN LOOP — runs forever.
    //
    // while(true): the condition is always true, so this never stops.
    // On a microcontroller this is CORRECT and EXPECTED behaviour.
    // The program should run until the power is removed.
    //
    // Inside the loop:
    // 1. Increment the counter
    // 2. Read the sensor
    // 3. Print the result
    // 4. Wait 2 seconds (DHT22 needs at least 2 seconds between reads)
    // 5. Repeat

    while (true) {

        reading_count++;   // ++ is the increment operator: reading_count = reading_count + 1

        // Call our read_dht22() function.
        // The function returns a DHT22Data struct.
        // We store the returned struct in the variable 'data'.
        // The struct is COPIED from the function's return value
        // into this local variable (value semantics in C).
        DHT22Data data = read_dht22();

        // Print reading number
        printf("Reading #%lu:\n", (unsigned long)reading_count);

        // Check if the reading was valid before printing data.
        // ALWAYS validate before using sensor data in real systems.
        // Using corrupt data could cause wrong decisions!

        if (data.valid) {

            // PRINT TEMPERATURE
            // We stored temperature * 10 as an integer.
            // To print "25.1", we divide: 251 / 10 = 25 (integer part)
            //                             251 % 10 = 1  (decimal part)
            //
            // / is INTEGER DIVISION (truncates toward zero)
            // % is MODULO (remainder after division)
            //
            // For negative temperatures, we need special handling
            // because -251 / 10 = -25 and -251 % 10 = -1 in C.
            // We use the absolute value for the decimal part.

            int32_t temp_int  = data.temperature / 10;    // Integer part
            int32_t temp_frac = data.temperature % 10;    // Decimal part

            // Make fraction positive for display (handles negative temps)
            if (temp_frac < 0) temp_frac = -temp_frac;

            printf("  Temperature: %ld.%ld C\n", (long)temp_int, (long)temp_frac);

            // PRINT HUMIDITY
            printf("  Humidity:    %ld.%ld %%\n",
                   (long)(data.humidity / 10),
                   (long)(data.humidity % 10));

            // Simple comfort indicator — IF/ELSE chain
            // This demonstrates conditional logic and how the CPU
            // branches (jumps to different instruction addresses)
            // based on data comparisons.
            printf("  Status:      ");
            if (data.temperature > 300) {           // > 30.0°C
                printf("TOO HOT!\n");
            } else if (data.temperature < 150) {    // < 15.0°C
                printf("TOO COLD!\n");
            } else if (data.humidity > 700) {       // > 70.0%
                printf("TOO HUMID!\n");
            } else {
                printf("Comfortable\n");
            }

        } else {
            // Reading failed — tell the user why
            printf("  ERROR: Sensor not responding or checksum failed.\n");
            printf("  Check wiring on GP%d.\n", DHT22_PIN);
        }

        printf("\n");   // Blank line between readings for readability

        // Wait 2 seconds before next reading.
        // DHT22 datasheet specifies minimum 2 seconds between readings.
        // If we read faster, the sensor returns stale or invalid data.
        //
        // sleep_ms() works by counting hardware timer ticks.
        // The RP2040's timer runs at 1MHz (1 tick per microsecond).
        // sleep_ms(2000) waits for 2,000,000 timer ticks.
        // During this wait, the CPU is IDLE — it just loops checking
        // the timer. (More advanced: use interrupts to sleep properly.)
        sleep_ms(2000);

    } // End of while(true) — jump back to top of loop

    // This line is UNREACHABLE — the while loop never exits.
    // We include it to satisfy the compiler (it expects main to return int)
    // and to follow the C standard.
    return 0;

} // End of main()


// ============================================================
//  COMPILATION PIPELINE SUMMARY
//  (What happens when you run 'make')
//
//  STAGE 1 — PREPROCESSOR  (arm-none-eabi-gcc -E)
//    Input:  dht22_sensor.c  (text)
//    Action: Expands #include, #define, removes comments
//    Output: dht22_sensor.i  (expanded text, ~50,000 lines)
//
//  STAGE 2 — COMPILER  (arm-none-eabi-gcc -S)
//    Input:  dht22_sensor.i  (C source)
//    Action: Translates C into ARM assembly instructions
//    Output: dht22_sensor.s  (assembly text)
//    Example — the line "gpio_put(DHT22_PIN, 0);" becomes:
//      movs r0, #15      ; load pin number (15) into register r0
//      movs r1, #0       ; load value (0) into register r1
//      bl   gpio_put     ; branch-with-link (call) gpio_put function
//
//  STAGE 3 — ASSEMBLER  (arm-none-eabi-as)
//    Input:  dht22_sensor.s  (assembly text)
//    Action: Translates assembly mnemonics to binary opcodes
//    Output: dht22_sensor.o  (object file — binary)
//    Example — "movs r0, #15" becomes: 0x200F (16-bit Thumb opcode)
//
//  STAGE 4 — LINKER  (arm-none-eabi-ld)
//    Input:  dht22_sensor.o + pico_stdlib.a + hardware_gpio.a + ...
//    Action: Combines all object files, resolves function addresses,
//            places code at correct Flash memory addresses (0x10000000+)
//            places data at correct RAM addresses (0x20000000+)
//    Output: blink_led.elf  (full executable with debug info)
//
//  STAGE 5 — BINARY CONVERSION  (arm-none-eabi-objcopy)
//    Input:  blink_led.elf
//    Action: Strips debug info, extracts raw binary
//    Output: blink_led.bin  then  blink_led.uf2
//
//  STAGE 6 — FLASH TO PICO
//    The .uf2 file is dragged onto the RPI-RP2 USB drive.
//    The Pico's bootrom (permanent ROM code) reads the .uf2
//    and writes each 256-byte page to Flash memory starting
//    at address 0x10000000.
//    Then the Pico resets and your code runs from Flash.
//
//  MEMORY MAP OF RP2040:
//  0x00000000 - 0x00003FFF  ROM (bootrom — 16KB, permanent)
//  0x10000000 - 0x101FFFFF  Flash (2MB — your program lives here)
//  0x20000000 - 0x20041FFF  SRAM (264KB — stack, heap, globals)
//  0xD0000000 - 0xD0000FFF  SIO (single-cycle I/O — GPIO registers)
//  0x40014000 - 0x40014FFF  GPIO hardware registers
//  0x400B0000 - 0x400B0FFF  Timer hardware registers
// ============================================================