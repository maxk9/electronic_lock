/* 
 * File:   main.c
 * Author: Maxk
 *
 * Created on 03.03.23
 * 
 * RFID reader communicate via wiegand 26
 * 
 * keys stored in eeprom:
 * adress 0 - count stored keys
 * add+1    first byte key
 * add+2    second byte key
 * add+3    third byte key
 */

#define _XTAL_FREQ 16000000
#include <htc.h>

__CONFIG(FOSC_INTOSC & WDTE_OFF & BOREN_OFF & FCMEN_OFF & IESO_OFF & MCLRE_OFF); //word 1
__CONFIG(LVP_OFF); //word 2 High-voltage on MCLR must be used for programming

#define EXT_LED_ON { TRISA0 = 0; RA0 = 0; }
#define EXT_LED_OFF { RA0 = 1; TRISA0 = 0; }
#define EXT_ZUM_ON { TRISA1 = 0; RA1 = 0; }
#define EXT_ZUM_OFF { RA1 = 1; TRISA1 = 0; }
#define BTN RA4
#define IS_BTN_PRESSED (BTN == 0)
#define OPEN_LOCK RA5 = 1
#define CLOSE_LOCK RA5 = 0
#define IS_OPEN_LOCK (RA5 == 1)

bit odd_first = 0;
bit even_last = 0;
bit msg_handled = 0;
bit msg_err = 0;
bit pgm_mode = 0;

volatile unsigned char read_data[3] = {0};
volatile unsigned char bit_cnt = 0;
volatile unsigned int tmr0_btn_delay = 0;
volatile unsigned int tmr0_msg_timeout = 0;
volatile unsigned int tmr0_pgm_delay = 0;
volatile unsigned char cnt_d = 0;

//#define DEBUG_ON

void send_usart(unsigned char dat) {
    while (!TRMT) { //wait till send previous byte
        NOP();
    }
    TXREG = dat;
}

void print_data(void) {
    send_usart(0xFF);
    send_usart(0xFF);
    send_usart(read_data[2]);
    send_usart(read_data[1]);
    send_usart(read_data[0]);
    send_usart(odd_first);
    send_usart(even_last);
    send_usart(0xFF);
    send_usart(0xFF);

    __delay_ms(4);
}

//********************************************************************************

void interrupt isr(void) {

    if (msg_handled && IOCIE) {
        if (IOCAF2) { //set 1
            if (bit_cnt == 0) {
                odd_first = 1;
            } else {
                if (bit_cnt < 8) {
                    read_data[2] |= 1;
                    read_data[2] <<= 1;
                } else
                    if (bit_cnt == 8) {
                    read_data[2] |= 1;
                } else
                    if (bit_cnt > 8 && bit_cnt < 16) {
                    read_data[1] |= 1;
                    read_data[1] <<= 1;
                } else
                    if (bit_cnt == 17) {
                    read_data[1] |= 1;
                } else
                    if (bit_cnt > 16 && bit_cnt < 24) {
                    read_data[0] |= 1;
                    read_data[0] <<= 1;
                } else
                    if (bit_cnt == 24) {
                    read_data[0] |= 1;
                }
                if (bit_cnt < 25)
                    ++cnt_d;
            }

            ++bit_cnt;

            if (bit_cnt == 26) { //last
                even_last = 1;
            }
        } else
            if (IOCAF3) { //set 0
            if (bit_cnt > 0 && bit_cnt < 8) {
                read_data[2] <<= 1;
            } else
                if (bit_cnt > 8 && bit_cnt < 16) {
                read_data[1] <<= 1;
            } else
                if (bit_cnt > 16 && bit_cnt < 24) {
                read_data[0] <<= 1;
            }

            ++bit_cnt;
        }
        if (IOCAF2 || IOCAF3) {
            if (bit_cnt == 1) {
                cnt_d = 0;
                msg_err = 0;
            } else
                if (bit_cnt == 13) {//check odd even
                if ((cnt_d & 1) != odd_first)
                    msg_err = 1; //err

                cnt_d = 0;
            } else
                if (bit_cnt > 25) {
                if ((cnt_d & 1) == even_last)
                    msg_err = 1; //err
                msg_handled = 0;
            }
        }
    }
    IOCAF = 0;

    //TIMER0 overflow every 16,384ms
    if (TMR0IE && TMR0IF) {
        TMR0IF = 0;
        if (bit_cnt != 0)
            ++tmr0_msg_timeout;

        if (IS_BTN_PRESSED) {
            ++tmr0_btn_delay;
        }
        if (pgm_mode)
            ++tmr0_pgm_delay;
    }
}


__EEPROM_DATA(0x0, 0x0, 0, 0, 0, 0, 0, 0);

void init_usart() {
    //	TXCKSEL = 1; //TX on RA4
    TXEN = 1; //tx enable
    SYNC = 0;
    BRGH = 1;
    SPBRG = 8; //set 115200 for 16MHz
    SPEN = 1; //Serial port enable
}

void init_timer() {
    /* f_tmr = 16M/4 = 4M 
    1/4M * 256 presc * 256 = 16.384 ms
       overflow every 16,384ms
     */
    PSA = 0; //prescaler is enabled
    OPTION_REGbits.PS = 0x7; //set prescaler 1:256
    TMR0CS = 0; //Internal instruction cycle clock (FOSC/4)
    TMR0IE = 1; //interrupt enable
}

void toggle_lock() {
    if (IS_OPEN_LOCK) {
        CLOSE_LOCK;
    } else {
        OPEN_LOCK;
    }
}

void flush_rx_msg() {
    bit_cnt = 0;
    read_data[2] = 0;
    read_data[1] = 0;
    read_data[0] = 0;
    //TODO check even odd
    even_last = 0;
    odd_first = 0;
    msg_handled = 1;
}

void toggle_led(unsigned char cnt_bit) {
    while (cnt_bit) {
        EXT_LED_ON;
        __delay_ms(100);
        EXT_LED_OFF;
        __delay_ms(100);

        --cnt_bit;
    }
}

void toggle_zumm(unsigned char cnt_bit) {
    while (cnt_bit) {
        EXT_ZUM_ON;
        __delay_ms(100);
        EXT_ZUM_OFF;
        __delay_ms(100);

        --cnt_bit;
    }
}

unsigned char check_key_in_base(void){
   unsigned char dat1, dat2, dat3;
   unsigned char count_keys = eeprom_read(0);
   
   //eeprom erased
   if(count_keys == 0xFF)
       return 0;
   
   //check already presented
     while (count_keys) {
        --count_keys;
        dat1 = eeprom_read(count_keys * 3 + 1);
        dat2 = eeprom_read(count_keys * 3 + 2);
        dat3 = eeprom_read(count_keys * 3 + 3);

        //already stored
        if (((dat1 == read_data[2]) &&
            (dat2 == read_data[1])) &&
            (dat3 == read_data[0]))
            return 1;
    }
    
    return 0;
}

unsigned char save_key_eeprom(unsigned char num_key) {
#ifndef  DEBUG_ON
    if(check_key_in_base()) {
        return 0;
    }
#endif
    
    //restrict max allowed keys is 10
    if(num_key > 10) 
        return 0;
    
    eeprom_write(0, num_key);
    --num_key; //begin from 1 addr
    eeprom_write(num_key * 3 + 1, read_data[2]);
    eeprom_write(num_key * 3 + 2, read_data[1]);
    eeprom_write(num_key * 3 + 3, read_data[0]);
    return 1;
}

void play_long_zum() {
    EXT_ZUM_ON;
    __delay_ms(1000);
    EXT_ZUM_OFF;
}

void main(void) {
    unsigned char cnt_key = 0;
    static unsigned char s_packet[8] = {0};
    static bit btn_released = 0;
    //    unsigned char prev_tmp;

    //setup oscal to 4MHz
    //OSCCON = 0x6A;
    //setup oscal to 16MHz
    OSCCON = 0x78;
    //init
    ANSELA = 0x0; //off analog inputs turn digital IO
    TRISA = 0xff; // all input 
    TRISA5 = 0; //out lock
    TRISA0 = 0; //out led
    TRISA1 = 0; //out zum
    RA0 = 1;
    RA1 = 1;

    nWPUEN = 0;
    WPUA = 0xFF; //WEAK PULL-UP en

    //init_usart();
    //WDT disable
    SWDTEN = 0;
    CLRWDT();
    //init isr
    IOCIE = 1;
    IOCAN3 = 1; //detect a falling edge
    IOCAN2 = 1; //detect a falling edge

    init_timer();
    
    //enable interrupts
    PEIE = 1;
    GIE = 1;
    WREN = 1;

    flush_rx_msg();

    while (1) {

        if (!msg_handled) { //data received 26 bits
#ifdef  DEBUG_ON
        eeprom_write(0, cnt_key);
        eeprom_write(cnt_key * 5 + 1, read_data[2]);
        eeprom_write(cnt_key * 5 + 2, read_data[1]);
        eeprom_write(cnt_key * 5 + 3, read_data[0]);
        eeprom_write(cnt_key * 5 + 4, odd_first);
        eeprom_write(cnt_key * 5 + 5, even_last);
        ++cnt_key;
#endif
            
            //read OK
            if (!msg_err) {
                if (pgm_mode) {
                    tmr0_pgm_delay = 0;
                    ++cnt_key;
                    if (save_key_eeprom(cnt_key)) {
                        toggle_zumm(2);
                    } else {
                       play_long_zum(); 
                    }
                } else {
                   if (check_key_in_base()) {
                       toggle_lock();
                       if (IS_OPEN_LOCK) {
                           //flash green led
                           EXT_LED_ON;
                           __delay_ms(500);
                           EXT_LED_OFF;
                       } else {
                           toggle_led(3);
                       }  
                   }
                }
            } else {
                //read error
                play_long_zum();
            }

            flush_rx_msg();
        }

        //set timeout for receiv msg for ~100ms
        if ((tmr0_msg_timeout > 6) && msg_handled) {
            tmr0_msg_timeout = 0;
            flush_rx_msg();
        }

        //button is push
        if (IS_BTN_PRESSED) {
            //16.3 * 5 = 80 ms
            if (tmr0_btn_delay > 5) {
                if (!btn_released) {
                    btn_released = 1;
                    toggle_lock();
                }
            }
            //check program mode 
            //long press more 5s
            if (tmr0_btn_delay > 312) {
                play_long_zum();
                pgm_mode = 1;
                tmr0_pgm_delay = 0;
                tmr0_btn_delay = 0;
                //erase all cnt
                eeprom_write(0, 0);
                cnt_key == 0x0;
            }
        } else {
            btn_released = 0;
            tmr0_btn_delay = 0;
        }

        if (pgm_mode) {
            toggle_led(1);

            //exit from pgm mode
            if (tmr0_pgm_delay > 600) {
                pgm_mode = 0;
                play_long_zum();
            }
        }

        //SLEEP(); // wait for interrupt
        NOP(); // ensure instruction pre-fetched while sleeping is safe to execute on awakening
    }
}

