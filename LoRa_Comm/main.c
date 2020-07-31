//###########################################################################
//
// FILE:   Example_2837xDSpi_FFDLB.c
//
// TITLE:  SPI Digital Loop Back program.
//
//! \addtogroup cpu01_example_list
//! <h1>SPI Digital Loop Back (spi_loopback)</h1>
//!
//!  This program uses the internal loop back test mode of the peripheral.
//!  Other then boot mode pin configuration, no other hardware configuration
//!  is required. Interrupts are not used.
//!
//!  A stream of data is sent and then compared to the received stream.
//!  The sent data looks like this: \n
//!  0000 0001 0002 0003 0004 0005 0006 0007 .... FFFE FFFF \n
//!  This pattern is repeated forever.
//!
//!  \b Watch \b Variables \n
//!  - \b sdata - sent data
//!  - \b rdata - received data
//!
//
//###########################################################################
// $TI Release: F2837xD Support Library v3.09.00.00 $
// $Release Date: Thu Mar 19 07:35:24 IST 2020 $
// $Copyright:
// Copyright (C) 2013-2020 Texas Instruments Incorporated - http://www.ti.com/
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions
// are met:
//
//   Redistributions of source code must retain the above copyright
//   notice, this list of conditions and the following disclaimer.
//
//   Redistributions in binary form must reproduce the above copyright
//   notice, this list of conditions and the following disclaimer in the
//   documentation and/or other materials provided with the
//   distribution.
//
//   Neither the name of Texas Instruments Incorporated nor the names of
//   its contributors may be used to endorse or promote products derived
//   from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
// OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
// LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
// DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
// THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
// $
//###########################################################################

//Registers and functions are based of the LoRa.h library for Arduino
// https://github.com/sandeepmistry/arduino-LoRa
// Registers for the card
#define REG_FIFO                 0x00
#define REG_OP_MODE              0x01
#define REG_FRF_MSB              0x06
#define REG_FRF_MID              0x07
#define REG_FRF_LSB              0x08
#define REG_PA_CONFIG            0x09
#define REG_OCP                  0x0b
#define REG_LNA                  0x0c
#define REG_FIFO_ADDR_PTR        0x0d
#define REG_FIFO_TX_BASE_ADDR    0x0e
#define REG_FIFO_RX_BASE_ADDR    0x0f
#define REG_FIFO_RX_CURRENT_ADDR 0x10
#define REG_IRQ_FLAGS            0x12
#define REG_RX_NB_BYTES          0x13
#define REG_PKT_SNR_VALUE        0x19
#define REG_PKT_RSSI_VALUE       0x1a
#define REG_MODEM_CONFIG_1       0x1d
#define REG_MODEM_CONFIG_2       0x1e
#define REG_PREAMBLE_MSB         0x20
#define REG_PREAMBLE_LSB         0x21
#define REG_PAYLOAD_LENGTH       0x22
#define REG_MODEM_CONFIG_3       0x26
#define REG_FREQ_ERROR_MSB       0x28
#define REG_FREQ_ERROR_MID       0x29
#define REG_FREQ_ERROR_LSB       0x2a
#define REG_RSSI_WIDEBAND        0x2c
#define REG_DETECTION_OPTIMIZE   0x31
#define REG_INVERTIQ             0x33
#define REG_DETECTION_THRESHOLD  0x37
#define REG_SYNC_WORD            0x39
#define REG_INVERTIQ2            0x3b
#define REG_DIO_MAPPING_1        0x40
#define REG_VERSION              0x42
#define REG_PA_DAC               0x4d

// Modes
#define MODE_LONG_RANGE_MODE     0x80
#define MODE_SLEEP               0x00
#define MODE_STDBY               0x01
#define MODE_TX                  0x03
#define MODE_RX_CONTINUOUS       0x05
#define MODE_RX_SINGLE           0x06

// Power antenna boost
#define PA_BOOST                   0x80

// IRQ masks
#define IRQ_TX_DONE_MASK           0x08
#define IRQ_PAYLOAD_CRC_ERROR_MASK 0x20
#define IRQ_RX_DONE_MASK           0x40

#define MAX_PKT_LENGTH             255
#define WORDLENGTH 0x7

//
// Included Files
//
#include "C:\ti\C2000Ware_3_02_00_00_Software\device_support\f2837xd\common\include\F28x_Project.h"
#include "C:\ti\C2000Ware_3_02_00_00_Software\device_support\f2837xd\common\include\driverlib.h"
#include "C:\ti\C2000Ware_3_02_00_00_Software\device_support\f2837xd\common\include\device.h"
//#include "board.h"
#include <stdint.h>

//
// Function Prototypes
//
void delay_loop(void);
void spi_xmit(Uint16 a);
void spi_fifo_init(void);
void spi_init(void);
void error(void);

//LoRa prototypes
void resetCard(void);
void write(Uint16 payload);
Uint16 readRegister(Uint16 address);
void writeRegister(Uint16 address, Uint16 value);
Uint16 singleTransfer(Uint16 addres, Uint16 value);
void setSpreadingFactor(int sf);
void setBandwidth(long sbw);
void sleep();
void idle();
long getSignalBandwidth();
int  getSpreadingFactor();
void setFrequency(long frequency);
void loraBegin(long frequency);


void main(void)
{


//
// Step 1. Initialize System Control:
// PLL, WatchDog, enable Peripheral Clocks
// This example function is found in the F2837xD_SysCtrl.c file.
//
   InitSysCtrl();

//
// Step 2. Initialize GPIO:
// This example function is found in the F2837xD_Gpio.c file and
// illustrates how to set the GPIO to it's default state.
// Setup only the GP I/O only for SPI-A functionality
// This function is found in F2837xD_Spi.c
//
   InitSpiaGpio();

//
// Step 3. Clear all interrupts:
//
   DINT;

//
// Initialize PIE control registers to their default state.
// The default state is all PIE __interrupts disabled and flags
// are cleared.
// This function is found in the F2837xD_PieCtrl.c file.
//
   InitPieCtrl();

//
// Disable CPU __interrupts and clear all CPU __interrupt flags:
//
   IER = 0x0000;
   IFR = 0x0000;

//
// Initialize the PIE vector table with pointers to the shell Interrupt
// Service Routines (ISR).
// This will populate the entire table, even if the __interrupt
// is not used in this example.  This is useful for debug purposes.
// The shell ISR routines are found in F2837xD_DefaultIsr.c.
// This function is found in F2837xD_PieVect.c.
//
   InitPieVectTable();

//
// Step 4. Initialize the Device Peripherals:
//
   //spi_fifo_init();     // Initialize the SPI FIFO

   spi_init();
//
// Step 5. User specific code:
//
   resetCard();
   GpioDataRegs.GPADAT.bit.GPIO19 = 0; //GPIO 19 set to LOW (chip select)
   spi_xmit(REG_MODEM_CONFIG_2& 0x7f); //transmit the address

   spi_xmit(0x00); //transmit the value to receive the current value of the register
   GpioDataRegs.GPADAT.bit.GPIO19 = 1; //GPIO 19 set to HIGH




    // Wait until data is received
    //
    //    while(SpiaRegs.SPIFFRX.bit.RXFFST !=1) { }

    //
    // Check against sent data
    //
    // rdata = SpiaRegs.SPIRXBUF;

}

///////////////////////////////////////////////////////////////
//Functions
//
//////////////////////////////////////////////////////////////

//Function that begins transmission through the card
void loraBegin(long frequency){
    resetCard();
    sleep();
    setFrequency(frequency);
    writeRegister(REG_FIFO_TX_BASE_ADDR, 0);
    writeRegister(REG_FIFO_RX_BASE_ADDR, 0);


    idle();
}


//Function that sets the frequency at witch the card works (channel)
void setFrequency(long frequency){
    unsigned long frf = (frequency << 19) / 32000000;
    writeRegister(REG_FRF_MSB, (frf >> 16));
    writeRegister(REG_FRF_MID, (frf >> 8));
    writeRegister(REG_FRF_LSB, (frf >> 0));
}


//Function that gets the Spreading Factor of the signal
int getSpreadingFactor(){
    return readRegister(REG_MODEM_CONFIG_2) >> 4;
}


//Function that gets the signal's bandwidth
long getSignalBandwidth(){
    Uint16 bw = (readRegister(REG_MODEM_CONFIG_1)) >> 4;
    switch (bw) {
      case 0: return 7.8E3;
      case 1: return 10.4E3;
      case 2: return 15.6E3;
      case 3: return 20.8E3;
      case 4: return 31.25E3;
      case 5: return 41.7E3;
      case 6: return 62.5E3;
      case 7: return 125E3;
      case 8: return 250E3;
      case 9: return 500E3;
    }
    return -1;
}


//Function that puts the radio in idle mode
void idle()
{
  writeRegister(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_STDBY);
}



//Function that puts the radio in sleep mode
void sleep()
{
  writeRegister(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_SLEEP);
}



//Function that sets the signal's bandwidth
void setBandwidth(long sbw){
    int bw;

    if (sbw <= 7.8E3) {
      bw = 0;
    }

    else if (sbw <= 10.4E3) {
      bw = 1;
    }

    else if (sbw <= 15.6E3) {
      bw = 2;
    }

    else if (sbw <= 20.8E3) {
      bw = 3;
    }

    else if (sbw <= 31.25E3) {
      bw = 4;
    }

    else if (sbw <= 41.7E3) {
      bw = 5;
    }

    else if (sbw <= 62.5E3) {
      bw = 6;
    }

    else if (sbw <= 125E3) {
      bw = 7;
    }

    else if (sbw <= 250E3) {
      bw = 8;
    }

    else{
      bw = 9;
    }

    writeRegister(REG_MODEM_CONFIG_1, (readRegister(REG_MODEM_CONFIG_1) & 0x0f) | (bw << 4));
}



//Function that sets the spreading factor
void setSpreadingFactor(int sf){
    if (sf < 6){
        sf = 6;         //minimum spreading factor is 6
    }
    else if (sf > 12){
       sf = 12;        //maximum spreading factor is 12
    }

    if (sf == 6){
        writeRegister(REG_DETECTION_OPTIMIZE, 0xc5);
        writeRegister(REG_DETECTION_THRESHOLD, 0x0c);
    }
    else{
        writeRegister(REG_DETECTION_OPTIMIZE, 0xc3);
        writeRegister(REG_DETECTION_THRESHOLD, 0x0a);
    }

    writeRegister(REG_MODEM_CONFIG_2, (readRegister(REG_MODEM_CONFIG_2) & 0x0f) | ((sf << 4) & 0xf0));
}



//Function that writes the pay-load
void write(Uint16 payload){
    int currentLength = readRegister(REG_PAYLOAD_LENGTH);
    writeRegister(REG_FIFO, payload);
}


//Function that reads registers of the card

Uint16 readRegister(Uint16 address){
    Uint16 read = singleTransfer(address & 0x7f, 0x00);
    return read;
}


//Function that writes to the registers
void writeRegister(Uint16 address, Uint16 value){
    singleTransfer(address | 0x80, value);
}



// resetCard resets the LoRa card

void resetCard(void){
    GpioDataRegs.GPADAT.bit.GPIO24 = 0; //GPIO 24 set to LOW
    delay_loop();
    GpioDataRegs.GPADAT.bit.GPIO24 = 1; //GPIO 24 set to HIGH
}

// Function that sends information to the Card either to read or to write

Uint16 singleTransfer(Uint16 address, Uint16 value){
    Uint16 response;
    GpioDataRegs.GPADAT.bit.GPIO19 = 0; //GPIO 19 set to LOW
    spi_xmit(address); //transmit the address
    response = SpiaRegs.SPIRXBUF;
    spi_xmit(value); //transmit the value to receive the current value of the register
    GpioDataRegs.GPADAT.bit.GPIO19 = 1; //GPIO 19 set to HIGH

    return response;
}

//
// delay_loop - Loop for a brief delay
//
void delay_loop()
{
    long i;
    for (i = 0; i < 1000000; i++) {}
}

//
// error - Error function that halts the debugger
//
void error(void)
{
    asm("     ESTOP0");     // Test failed!! Stop!
    for (;;);
}

//
// spi_xmit - Transmit value via SPI
//
void spi_xmit(Uint16 a)
{
    Uint16 data;
    data = a<<8;        // left shift tx data 8 bits.
    SpiaRegs.SPITXBUF = data;
}

//
// spi_fifo_init - Initialize SPIA FIFO
//
void spi_fifo_init()
{
    //
    // Initialize SPI FIFO registers
    //
    SpiaRegs.SPIFFTX.all = 0xE040;
    SpiaRegs.SPIFFRX.all = 0x2044;
    SpiaRegs.SPIFFCT.all = 0x0;

    //
    // Initialize core SPI registers
    //
    InitSpi();
}


void spi_init() //Taken from a modified example found at https://e2e.ti.com/support/microcontrollers/c2000/f/171/t/343554
{
    SpiaRegs.SPICCR.all =0x0000|WORDLENGTH;     // Reset on, rising edge, WORDLENGTH char bits
    SpiaRegs.SPICTL.all =0x0006;                // Enable master mode, normal phase,
                                                // enable talk, and SPI int disabled.
   // SpiaRegs.SPIBRR =0x007f;                  // for some reason this value fails and its not supposed to
//    SpiaRegs.SPICCR.all =0x0090|WORDLENGTH    // Relinquish SPI from Reset (LB mode)
    SpiaRegs.SPICCR.all =0x0080|WORDLENGTH;     // Relinquish SPI from Reset (external loopback mode)
    SpiaRegs.SPIPRI.bit.FREE = 1;               // Set so breakpoints don't disturb xmission
}

//
// End of file
//
