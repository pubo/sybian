/* Microchip Technology Inc. and its subsidiaries.  You may use this software 
 * and any derivatives exclusively with Microchip products. 
 * 
 * THIS SOFTWARE IS SUPPLIED BY MICROCHIP "AS IS".  NO WARRANTIES, WHETHER 
 * EXPRESS, IMPLIED OR STATUTORY, APPLY TO THIS SOFTWARE, INCLUDING ANY IMPLIED 
 * WARRANTIES OF NON-INFRINGEMENT, MERCHANTABILITY, AND FITNESS FOR A 
 * PARTICULAR PURPOSE, OR ITS INTERACTION WITH MICROCHIP PRODUCTS, COMBINATION 
 * WITH ANY OTHER PRODUCTS, OR USE IN ANY APPLICATION. 
 *
 * IN NO EVENT WILL MICROCHIP BE LIABLE FOR ANY INDIRECT, SPECIAL, PUNITIVE, 
 * INCIDENTAL OR CONSEQUENTIAL LOSS, DAMAGE, COST OR EXPENSE OF ANY KIND 
 * WHATSOEVER RELATED TO THE SOFTWARE, HOWEVER CAUSED, EVEN IF MICROCHIP HAS 
 * BEEN ADVISED OF THE POSSIBILITY OR THE DAMAGES ARE FORESEEABLE.  TO THE 
 * FULLEST EXTENT ALLOWED BY LAW, MICROCHIP'S TOTAL LIABILITY ON ALL CLAIMS 
 * IN ANY WAY RELATED TO THIS SOFTWARE WILL NOT EXCEED THE AMOUNT OF FEES, IF 
 * ANY, THAT YOU HAVE PAID DIRECTLY TO MICROCHIP FOR THIS SOFTWARE.
 *
 * MICROCHIP PROVIDES THIS SOFTWARE CONDITIONALLY UPON YOUR ACCEPTANCE OF THESE 
 * TERMS. 
 */

/* 
 * File:   
 * Author: 
 * Comments:
 * Revision history: 
 */

// This is a guard condition so that contents of this file are not included
// more than once.  
#ifndef DEFINITION_H
#define	DEFINITION_H

// TODO Insert appropriate #include <>

// TODO Insert C++ class definitions if appropriate
#define ON                      1
#define OFF                     0
#define SET                     1
#define CLEAR                   0
#define Hi                      1
#define Lo                      0

#define pwr_state_max           5
#define pwr_timeout             88
#define sample_delay            88
#define debounce_horn_max       800
#define debounce_idle_max       5
#define dac_delay               8
#define debounce_dac_max        28
#define alarm_delay             18
#define debounce_alarm_max      28
#define io_period               100

/*----- Threshold definition--------*/
#define threshold_heat          8
#define threshold_smoke         8
#define threshold_missing       8

#define adc_heat                969
#define adc_smoke               598
#define adc_mhead               306
#define adc_normal              128
#define adc_led                 88
#define adc_det                 28
#define adc_meol                20
#define adc_gap                 50

#define dac_heat                223
#define dac_smoke               141
#define dac_mhead               76
#define dac_normal              0
#define dac_null                0

/*----- PORT Pin Definition---------*/
#define out_pwr                 LATB4
#define out_ict                 LATC0
#define out_headen              LATC1
#define out_led                 LATC3
#define out_ioa                 LATB3                
#define out_iob                 LATA4
#define out_ioc                 LATA7
#define out_iod                 LATA6
#define out_ioe                 LATB0

#define in_sw1                  RC7
#define in_sw2                  RC6
#define in_sw3                  RC5
#define in_sw4                  RC4
#define in_horn                 RA5
#define in_test                 RC2

#define an_det                  channel_AN0
#define an_ict                  channel_AN1
#define an_led                  channel_AN3
#define an_horn                 channel_AN4
#define an_ledcomm              channel_AN13
#define an_test                 channel_AN14

/*----- Device Type Definition-------*/
#define type_null               0x00
#define type_smoke              0x01
#define type_heat               0x02
#define type_mcp                0x03
#define type_io                 0x04

/*----- App State Definition-------*/
#define app_state_null          0x00
#define app_state_init          0x01
#define app_state_runtime       0x02
#define app_state_sample        0x03
#define app_state_io            0x04

/*----- Device State Definition-------*/
#define dev_state_null          0x00
#define dev_state_init          0x01
#define dev_state_idle          0x02
#define dev_state_fault         0x03
#define dev_state_test          0x04  
#define dev_state_extern        0x05
#define dev_state_alarm         0x06

/*----- UART Character Set ----------*/
#define NUL                     0x00
#define SOH                     0x01 
#define STX                     0x02
#define ETX                     0x03
#define EOT                     0x04
#define ENQ                     0x05
#define ACK                     0x06
#define BEL                     0x07

#define REQ                     0x08
#define INT                     0x09
#define ASG                     0x0A
#define ACT1                    0x0C
#define ACT2                    0x0D

#define NAK                     0x15
#define CAN                     0x18
#define ESC                     0x1B

#define RING_BUFFER_SIZE        32

#define mask_index              0xE0
#define mask_data               0x1F

#define offset_master_cmd       0x00
#define offset_master_addr      0x20
#define offset_master_type      0x40
#define offset_reserve          0x60
#define offset_slave_stat       0x80
#define offset_slave_type       0xA0
#define offset_slave_cmd        0xC0
#define offset_slave_addr       0xE0

// TODO Insert declarations
unsigned char app_state         = 0x00;                                         // 0: default state, 1: init routine, 2: runtime
unsigned char dev_address       = 0x00;                                         // initialise as zero
unsigned char dev_state         = 0x00;                                         // 0: Un-init state, 1: Init Routine, 2: Idle, 3: fault, 4: test, 5: alarm, 6: External alarm
unsigned char dev_sstate        = 0x00;                                         // shadow register for device state
unsigned char dev_type          = 0x00;                                         // 0: Un-init state, 1: 3V Ei650 unit (default), 2: 9V Ei605
unsigned char dev_priority      = 0x00;                                         // 0: priority 2 changed via dip switch, 1: default priority 1 

unsigned char uart_state        = 0x00;                                         // 0: Idle/listening rx, 1: Sent/ACK, waiting for reply tx
unsigned char uart_rxmode       = 0x00;                                         // 0: Idle, 1: write data
unsigned char uart_txmode       = 0x00;
unsigned char uart_rx_cmd       = 0x00;

unsigned int value_led          = 0x0000;                                       // led check
unsigned int value_det          = 0x0000;                                       // head check
unsigned int value_ict          = 0x0000;                                       // eol check
//unsigned int value_test     = 0x0000;                                           // test status check
//unsigned int value_horn     = 0x0000;                                           // horn check
unsigned int value_uart         = 0x0000;
unsigned char value_dac         = 0x00;

unsigned char flag_horn         = 0x00;
unsigned char flag_test         = 0x00;
unsigned char flag_ict          = 0x00;
unsigned char flag_dummy        = 0x00;
unsigned char flag_inh_led      = 0x00;
unsigned char flag_inh_ict      = 0x00;

unsigned char RBUFFER[RING_BUFFER_SIZE + 1];
unsigned char uart_tx_buffer [64];
unsigned char uart_rx_buffer [64];
char *s_rx_ptr                  = 0x00;
char *s_tx_ptr                  = 0x00;
unsigned char tx_byte           = 0x00;
unsigned char rx_byte           = 0x00;
unsigned char gets_flag         = 0x00;
unsigned char print_flag        = 0x00;
unsigned char rx_ptr            = 0x00;

unsigned char pwr_chk           = 0x00;                                         // routine init/re-trigger flag
unsigned char pwr_tick          = 0x00;                                         // 1ms counter triggered in tmr0 ISR
//unsigned char pwr_timeout       = 0x58;                                         // 100 count timeout
unsigned char pwr_flag          = 0x00;                                         // counter for pin sampling. 9V++, 3V--
unsigned char pwr_state         = 0x00;                                         // min cycles of check done before defining device type

unsigned char debounce_pwr      = 0x00;                                             // set to 1 when power init is done.
unsigned char debounce_act1     = 0x00;
unsigned char debounce_act2     = 0x00;
unsigned char debounce_can      = 0x00;
unsigned int debounce_horn      = 0x0000;
unsigned int debounce_io        = 0x0000;
unsigned int debounce_idle      = 0x0000;
unsigned int debounce_sample    = 0x0000;
unsigned int debounce_dac       = 0x0000;
unsigned int debounce_alarm     = 0x0000;

// Comment a function and leverage automatic documentation with slash star star
/**
    <p><b>Function prototype:</b></p>
  
    <p><b>Summary:</b></p>

    <p><b>Description:</b></p>

    <p><b>Precondition:</b></p>

    <p><b>Parameters:</b></p>

    <p><b>Returns:</b></p>

    <p><b>Example:</b></p>
    <code>
 
    </code>

    <p><b>Remarks:</b></p>
 */
// TODO Insert declarations or function prototypes (right here) to leverage 
// live documentation

#ifdef	__cplusplus
extern "C" {
#endif /* __cplusplus */

    // TODO If C++ is being used, regular C code needs function names to have C 
    // linkage so the functions can be used by the c code. 

#ifdef	__cplusplus
}
#endif /* __cplusplus */

#endif	/* DEFINITION_H */

