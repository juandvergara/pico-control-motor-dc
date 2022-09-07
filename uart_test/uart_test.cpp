#include "pico/stdlib.h"
#include <cstdio>
#include <cstring>
#include "hardware/uart.h"
#include "hardware/irq.h"
#include "pico/util/queue.h"

#define UART_ID uart0
#define BAUD_RATE 100000
#define DATA_BITS 8
#define STOP_BITS 2
#define PARITY    UART_PARITY_EVEN

// We are using pins 0 and 1, but see the GPIO function select table in the
// datasheet for information on which other pins can be used.
#define UART_TX_PIN 0
#define UART_RX_PIN 1

#define INTERVAL 20

typedef struct channels_s
{
    unsigned ch0 : 11;
    unsigned ch1 : 11;
    unsigned ch2 : 11;
    unsigned ch3 : 11;
    unsigned ch4 : 11;
    unsigned ch5 : 11;
    unsigned ch6 : 11;
    unsigned ch7 : 11;
    unsigned ch8 : 11;
    unsigned ch9 : 11;
    unsigned ch10 : 11;
    unsigned ch11 : 11;
    unsigned ch12 : 11;
    unsigned ch13 : 11;
    unsigned ch14 : 11;
    unsigned ch15 : 11;
}  __attribute__((packed)) channels_t;

struct sbusFrame_s{
    uint8_t synchro;
    channels_s rcChannelsData;
    uint8_t flag;
    uint8_t endByte;
};

sbusFrame_s sbus_frame;
queue_t queue;
enum SBUS_STATE {
    NO_SBUS_FRAME = 0,
    RECEIVING_SBUS
};
uint8_t running_frame[24];
char out_buffer[100];

uint32_t last_time_out;
//static int chars_rxed = 0;
static char prev_ch;
// RX interrupt handler
void on_uart_rx() {
    while (uart_is_readable(UART_ID)) {
        uint8_t ch = uart_getc(UART_ID);
        queue_try_add(&queue, &ch);
    }
}

void setup(uart_inst *uart_id, uint baud_rate, uint tx_pin, uint rx_pin, uint data_bits, uint stop_bits, uart_parity_t parity) {
    // Set up our UART with a basic baud rate.
    uint baud = uart_init(uart_id, baud_rate);

    // Set the TX and RX pins by using the function select on the GPIO
    // Set datasheet for more information on function select
    gpio_set_function(tx_pin, GPIO_FUNC_UART);
    gpio_set_function(rx_pin, GPIO_FUNC_UART);
    gpio_set_inover(rx_pin , GPIO_OVERRIDE_INVERT);

    // Set UART flow control CTS/RTS, we don't want these, so turn them off
    uart_set_hw_flow(uart_id, false, false);

    // Set our data format
    uart_set_format(uart_id, data_bits, stop_bits, parity);

    // Turn off FIFO's - we want to do this character by character
    uart_set_fifo_enabled(uart_id, false);

    // Set up a RX interrupt
    // We need to set up the handler first
    // Select correct interrupt for the UART we are using
    int UART_IRQ = uart_id == uart0 ? UART0_IRQ : UART1_IRQ;

    // And set up and enable the interrupt handlers
    irq_set_exclusive_handler(UART_IRQ, on_uart_rx);
    irq_set_enabled(UART_IRQ, true);

    // Now enable the UART to send interrupts - RX only
    uart_set_irq_enables(uart_id, true, false);

    // init queue
    queue_init(&queue, 1, 256);
    uint8_t dummy;
    // empty queue
    while(!queue_is_empty(&queue)) queue_try_remove(&queue, &dummy);
    // OK, all set up.
    // Lets send a basic string out, and then run a loop and wait for RX interrupts
    // The handler will count them, but also reflect the incoming data back with a slight change!
    uart_puts(uart_id, "\nHello, uart interrupts\r\n");
    char buffer[50];
    sprintf(buffer, "Baud: %d\r\n", baud);
    uart_puts(uart_id, buffer);
}


uint32_t millis() {
    return to_ms_since_boot(get_absolute_time());
}

int main() {

    setup(UART_ID, BAUD_RATE, UART_TX_PIN, UART_RX_PIN, DATA_BITS, STOP_BITS, PARITY);

    SBUS_STATE sbus_state = NO_SBUS_FRAME;
    uint8_t sbus_counter = 0;
    uint32_t last_sbus_millis = 0;
    uint8_t c;
    last_time_out = millis();

    while (1) {

        while(!queue_is_empty(&queue)) {
            if((millis() - last_sbus_millis) > 2){
                sbus_state = NO_SBUS_FRAME;
            }
            last_sbus_millis = millis();
            queue_try_remove(&queue, &c);
            switch (sbus_state) {
                case NO_SBUS_FRAME:
                    if (c == 0x0F) {
                        sbus_counter = 1;
                        sbus_state = RECEIVING_SBUS;
                    }
                    break;

                case RECEIVING_SBUS:
                    running_frame[sbus_counter++] = c;
                    if(sbus_counter == 24) {
                        if((c != 0x00) && (c != 0x04) && (c != 0x14) && (c != 0x24) && (c != 0x34))
                            sbus_state = NO_SBUS_FRAME;
                        else {
                            // save
                            memcpy((uint8_t *) &sbus_frame.rcChannelsData, &running_frame[1], 22);
                            sbus_state = NO_SBUS_FRAME;
                        }
                    }
            }
        }

        if ((millis() - last_time_out) > INTERVAL) {
            last_time_out = millis();
            sprintf(
                    out_buffer,
                    "0: %d\t1: %d\t2: %d\t3: %d\t4: %d\t5: %d\t6: %d\t7: %d\t8: %d\t9: %d\t10: %d\r\n",
                    sbus_frame.rcChannelsData.ch0,
                    sbus_frame.rcChannelsData.ch1,
                    sbus_frame.rcChannelsData.ch2,
                    sbus_frame.rcChannelsData.ch3,
                    sbus_frame.rcChannelsData.ch4,
                    sbus_frame.rcChannelsData.ch5,
                    sbus_frame.rcChannelsData.ch6,
                    sbus_frame.rcChannelsData.ch7,
                    sbus_frame.rcChannelsData.ch8,
                    sbus_frame.rcChannelsData.ch9,
                    sbus_frame.rcChannelsData.ch10
                    );
            uart_puts(UART_ID, out_buffer);
        }
    }
}
