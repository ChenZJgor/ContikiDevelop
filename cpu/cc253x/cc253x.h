/*
 * Copyright (c) 2011, YoCiHou - < www.iotdev.net >
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the Institute nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE INSTITUTE AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE INSTITUTE OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 * This file is part of the Contiki operating system.
 */

/**
 * \file
 *   Definitions for TI/Chipcon cc2530, cc2531 and cc2533 SFR registers.
 *
 *   Based on information in:
 *   "CC253x System-on-Chip Solution for 2.4-GHz IEEE 802.15.4 and ZigBeeÂ®
 *    Applications"
 *   Literature Number: SWRU191B. April 2009. Revised September 2010
 *
 * \author
 *           YoCiHou - < www.iotdev.net >
 */

#ifndef __CC253X_H__
#define __CC253X_H__

/*---------------------------------------------------------------------------*/
/* Compiler Abstraction */
#include <compiler.h>
/*---------------------------------------------------------------------------
 * Interrupt Vectors
 * (Table 2.5, page 40)
 *---------------------------------------------------------------------------*/
#define RFERR_VECTOR     VECT( 0, 0x03 )  /* RF TXFIFO underflow and RXFIFO overflow. */
#define ADC_VECTOR       VECT( 1, 0x0B )  /* ADC end of conversion */
#define URX0_VECTOR      VECT( 2, 0x13 )  /* USART 0 RX complete */
#define URX1_VECTOR      VECT( 3, 0x1B )  /* USART 1 RX complete */
#define ENC_VECTOR       VECT( 4, 0x23 )  /* AES encryption/decryption complete */
#define ST_VECTOR        VECT( 5, 0x2B )  /* Sleep Timer compare */
#define P2INT_VECTOR     VECT( 6, 0x33 )  /* Port 2 inputs/USB/I2C */
#define UTX0_VECTOR      VECT( 7, 0x3B )  /* USART 0 TX complete */
#define DMA_VECTOR       VECT( 8, 0x43 )  /* DMA transfer complete */
#define T1_VECTOR        VECT( 9, 0x4B )  /* Timer 1 (16-bit) capture/compare/overflow */
#define T2_VECTOR        VECT( 10, 0x53 )  /* Timer 2 (MAC Timer) */
#define T3_VECTOR        VECT( 11, 0x5B )  /* Timer 3 (8-bit) compare/overflow */
#define T4_VECTOR        VECT( 12, 0x63 )  /* Timer 4 (8-bit) compare/overflow */
#define P0INT_VECTOR     VECT( 13, 0x6B )  /* Port 0 inputs */
#define UTX1_VECTOR      VECT( 14, 0x73 )  /* USART1 TX complete */
#define P1INT_VECTOR     VECT( 15, 0x7B )  /* Port 1 inputs */
#define RF_VECTOR        VECT( 16, 0x83 )  /* RF general interrupts */
#define WDT_VECTOR       VECT( 17, 0x8B )  /* Watchdog overflow in timer mode */
/*---------------------------------------------------------------------------
 * Special Function Registers and BITs
 * (Table 2.1, page 27)
 *---------------------------------------------------------------------------*/
/* 8051 Internals */
SFRBIT(P0,0x80,P0_7,P0_6,P0_5,P0_4,P0_3,P0_2,P0_1,P0_0); /* Port 0 */
SFR(SP,   0x81); /* Stack pointer */
SFR(DPL0, 0x82); /* Data pointer 0 low byte */
SFR(DPH0, 0x83); /* Data pointer 0 high byte */
SFR(DPL1, 0x84); /* Data pointer 1 low byte */
SFR(DPH1, 0x85); /* Data pointer 1 high byte */
SFR(PCON, 0x87); /* Power mode control */
SFRBIT(TCON,0x88,URX1IF,TCON_b6,ADCIF,TCON_b4,URX0IF,IT1,RFERRIF,IT0); /* Interrupt flags */
/* URX1IF: USART1 RX interrupt flag 
   ADCIF: ADC interrupt flag 
   URX0IF: USART0 RX interrupt flag 
   IT1: Reserved. Must always be set to 1 
   RFERRIF: RF TXFIFO/RXFIFO interrupt flag 
   IT0: Reserved. Must always be set to 1 */
SFRBIT(P1,0x90,P1_7,P1_6,P1_5,P1_4,P1_3,P1_2,P1_1,P1_0); /* Port 1 */
SFR(DPS, 0x92); /* Data pointer select */
SFRBIT(S0CON,0x98,S0CON_b7,S0CON_b6,S0CON_b5,S0CON_b4,S0CON_b3,S0CON_b2, ENCIF_1, ENCIF_0); /* Interrupt flags 2 */
/* ENCIF_1: AES Interrupt flag 1 */
/* ENCIF_0: AES Interrupt flag 0 */
SFR(IEN2,  0x9A); /* Interrupt enable 2 */
SFR(S1CON, 0x9B); /* Interrupt flags 3 */
SFRBIT(P2, 0xA0, P2_7,P2_6,P2_5,P2_4,P2_3,P2_2,P2_1,P2_0); /* Port 2 */
SFRBIT(IEN0,0xA8,EA,IEN0_b6,STIE,ENCIE,URX1IE,URX0IE,ADCIE,RFERRIE); /* Interrupt enable 0 */
/* EA: All interrupts - enable/disable
   STIE: Sleep Timer interrupt enable
   ENCIE: AES encryption/decryption interrupt enable
   URX1IE: USART1 RX interrupt enable
   URX0IE: USART0 RX interrupt enable
   ADCIE: ADC interrupt enable
   RFERRIE: RF TXFIFO/RXFIFO interrupt enable */
SFR(IP0, 0xA9); /* Interrupt priority 0 */
SFRBIT(IEN1,0xB8,IEN1_b7,IEN1_b6,P0IE,T4IE,T3IE,T2IE,T1IE,DMAIE); /* Interrupt enable 1 */
/* P0IE: Port 0 interrupt enable 
   T4IE: Timer 4 interrupt enable
   T3IE: Timer 3 interrupt enable
   T2IE: Timer 2 interrupt enable
   T1IE: Timer 1 interrupt enable
   DMAIE: DMA Transfer interrupt enable */
SFR(IP1, 0xB9); /* Interrupt priority 1 */
SFRBIT(IRCON,0xC0,STIF,IRCON_b6,P0IF,T4IF,T3IF,T2IF,T1IF,DMAIF);/* Interrupt flags 4 */
/* STIF: Sleep Timer interrupt flag
   P0IF: Port 0 interrupt flag
   T4IF: Timer 4 interrupt flag
   T3IF: Timer 3 interrupt flag
   T2IF: Timer 2 interrupt flag
   T1IF: Timer 1 interrupt flag
   DMAIF: DMA-complete interrupt flag */
SFRBIT(PSW,0xD0,CY,AC,F0,RS1,RS0,OV,F1,P); /* Program status word */
/* CY: Carry flag 
   AC: Auxiliary carry flag
   F0: User-defined flag 1, bit addressable
   RS1: Register bank select, bit 1
   RS0: Register bank select, bit 0
   OV: Overflow flag
   F1: User-defined flag 0, bit addressable
   P: Parity flag */
SFRBIT(ACC,0xE0,ACC_7,ACC_6,ACC_5,ACC_4,ACC_3,ACC_2,ACC_1,ACC_0); /* Accumulator */
SFRBIT(IRCON2,0xE8,IRCON2_b7,IRCON2_b6,IRCON2_b5,WDTIF,P1IF,UTX1IF,UTX0IF,P2IF); /* Interrupt flags 5 */
/* WDTIF: Watchdog Timer interrupt flag 
   P1IF: Port 1 Interrupt flag
   UTX1IF: USART1 TX interrupt flag
   UTX0IF: USART0 TX interrupt flag
   P2IF: Port 2 interrupt flag */
SFRBIT(B,0xF0,B_7,B_6,B_5,B_4,B_3,B_2,B_1,B_0); /* B Register */

/* ADC */
SFR(ADCCON1,   0xB4); /* ADC control 1 */
SFR(ADCCON2,   0xB5); /* ADC control 2 */
SFR(ADCCON3,   0xB6); /* ADC control 3 */
SFR(ADCL,      0xBA); /* ADC data low */
SFR(ADCH,      0xBB); /* ADC data high */
SFR(RNDL,      0xBC); /* Random number generator data low */
SFR(RNDH,      0xBD); /* Random number generator data high */

/* AES Coprocessor */
SFR(ENCDI,     0xB1); /* Encryption/decryption input data */
SFR(ENCDO,     0xB2); /* Encryption/decryption output data */
SFR(ENCCS,     0xB3); /* Encryption/decryption control and status */

/* DMA Controller */
SFR(DMAIRQ,    0xD1); /* DMA interrupt flag */
SFR(DMA1CFGL,  0xD2); /* DMA channel 1â€? configuration address low */
SFR(DMA1CFGH,  0xD3); /* DMA channel 1â€? configuration address high */
SFR(DMA0CFGL,  0xD4); /* DMA channel 0 configuration address low */
SFR(DMA0CFGH,  0xD5); /* DMA channel 0 configuration address high */
SFR(DMAARM,    0xD6); /* DMA channel armed */
SFR(DMAREQ,    0xD7); /* DMA channel start request and status */

/* I/O */
SFR(P0IFG,     0x89); /* Port 0 interrupt status flag */
SFR(P1IFG,     0x8A); /* Port 1 interrupt status flag */
SFR(P2IFG,     0x8B); /* Port 2 interrupt status flag */
SFR(PICTL,     0x8C); /* Port pins interrupt mask and edge */
SFR(P0IEN,     0xAB); /* Port 0 interrupt mask */
SFR(P1IEN,     0x8D); /* Port 1 interrupt mask */
SFR(P2IEN,     0xAC); /* Port 2 interrupt mask */
SFR(P0INP,     0x8F); /* Port 0 input Mode */
SFR(PERCFG,    0xF1); /* Peripheral I/O control */
SFR(APCFG,     0xF2); /* Analog peripheral I/O configuration */
SFR(P0SEL,     0xF3); /* Port 0 function select */
SFR(P1SEL,     0xF4); /* Port 1 function select */
SFR(P2SEL,     0xF5); /* Port 2 function select */
SFR(P1INP,     0xF6); /* Port 1 input mode */
SFR(P2INP,     0xF7); /* Port 2 input mode */
SFR(P0DIR,     0xFD); /* Port 0 direction */
SFR(P1DIR,     0xFE); /* Port 1 direction */
SFR(P2DIR,     0xFF); /* Port 2 direction */
SFR(PMUX,      0xAE); /* Power-down signal mux */

/* Memory */
SFR(MPAGE,     0x93); /* Memory page select */
SFR(_XPAGE,    0x93); /* Memory page select - SDCC name */
SFR(MEMCTR,    0xC7); /* Memory system control */
SFR(FMAP,      0x9F); /* Flash-memory bank mapping */
SFR(PSBANK,    0x9F); /* Flash-memory bank mapping - SDCC name */

/* RF */
SFR(RFIRQF1,   0x91); /* RF interrupt flags MSB */
SFR(RFD,       0xD9); /* RF data */
SFR(RFST,      0xE1); /* RF command strobe */
SFR(RFIRQF0,   0xE9); /* RF interrupt flags LSB */
SFR(RFERRF,    0xBF); /* RF error interrupt flags */

/* Sleep Timer */
SFR(ST0,       0x95); /* Sleep Timer 0 */
SFR(ST1,       0x96); /* Sleep Timer 1 */
SFR(ST2,       0x97); /* Sleep Timer 2 */
SFR(STLOAD,    0xAD); /* Sleep-timer load status */
SFR(SLEEPCMD,  0xBE); /* Sleep-mode control command */
SFR(SLEEPSTA,  0x9D); /* Sleep-mode control status */

/* Power Management and Clocks */
SFR(CLKCONCMD, 0xC6); /* Clock control command */
SFR(CLKCONSTA, 0x9E); /* Clock control status */

/* Timer 1 */
SFR(T1CC0L,    0xDA); /* Timer 1 channel 0 capture/compare value low */
SFR(T1CC0H,    0xDB); /* Timer 1 channel 0 capture/compare value high */
SFR(T1CC1L,    0xDC); /* Timer 1 channel 1 capture/compare value low */
SFR(T1CC1H,    0xDD); /* Timer 1 channel 1 capture/compare value high */
SFR(T1CC2L,    0xDE); /* Timer 1 channel 2 capture/compare value low */
SFR(T1CC2H,    0xDF); /* Timer 1 channel 2 capture/compare value high */
SFR(T1CNTL,    0xE2); /* Timer 1 counter low */
SFR(T1CNTH,    0xE3); /* Timer 1 counter high */
SFR(T1CTL,     0xE4); /* Timer 1 control and status */
SFR(T1CCTL0,   0xE5); /* Timer 1 channel 0 capture/compare control */
SFR(T1CCTL1,   0xE6); /* Timer 1 channel 1 capture/compare control */
SFR(T1CCTL2,   0xE7); /* Timer 1 channel 2 capture/compare control */
SFR(T1STAT,    0xAF); /* Timer 1 status */

/* Timer 2 (MAC Timer) */
SFR(T2CTRL,    0x94); /* Timer 2 control */
SFR(T2EVTCFG,  0x9C); /* Timer 2 event configuration */
SFR(T2IRQF,    0xA1); /* Timer 2 interrupt flags */
SFR(T2M0,      0xA2); /* Timer 2 multiplexed register 0 */
SFR(T2M1,      0xA3); /* Timer 2 multiplexed register 1 */
SFR(T2MOVF0,   0xA4); /* Timer 2 multiplexed overflow register 0 */
SFR(T2MOVF1,   0xA5); /* Timer 2 multiplexed overflow register 1 */
SFR(T2MOVF2,   0xA6); /* Timer 2 multiplexed overflow register 2 */
SFR(T2IRQM,    0xA7); /* Timer 2 interrupt mask */
SFR(T2MSEL,    0xC3); /* Timer 2 multiplex select */

/* Timer 3 */
SFR(T3CNT,     0xCA); /* Timer 3 counter */
SFR(T3CTL,     0xCB); /* Timer 3 control */
SFR(T3CCTL0,   0xCC); /* Timer 3 channel 0 compare control */
SFR(T3CC0,     0xCD); /* Timer 3 channel 0 compare value */
SFR(T3CCTL1,   0xCE); /* Timer 3 channel 1 compare control */
SFR(T3CC1,     0xCF); /* Timer 3 channel 1 compare value */

/* Timer 4 */
SFR(T4CNT,     0xEA); /* Timer 4 counter */
SFR(T4CTL,     0xEB); /* Timer 4 control */
SFR(T4CCTL0,   0xEC); /* Timer 4 channel 0 compare control */
SFR(T4CC0,     0xED); /* Timer 4 channel 0 compare value */
SFR(T4CCTL1,   0xEE); /* Timer 4 channel 1 compare control */
SFR(T4CC1,     0xEF); /* Timer 4 channel 1 compare value */

/* Timer 1, 3, 4 join Interrupts */
SFRBIT(TIMIF,0xD8,TIMIF_b7,OVFIM,T4CH1IF,T4CH0IF,T4OVFIF,T3CH1IF,T3CH0IF,T3OVFIF); /* Timers 1/3/4 joint interrupt mask/flags */
/* OVFIM: Timer 1 overflow interrupt mask */
/* T4CH1IF: Timer 4 channel 1 interrupt flag */
/* T4CH0IF: Timer 4 channel 0 interrupt flag */
/* T4OVFIF: Timer 4 overflow interrupt flag */
/* T3CH1IF: Timer 3 channel 1 interrupt flag */
/* T3CH0IF: Timer 3 channel 0 interrupt flag */
/* T3OVFIF: Timer 3 overflow interrupt flag */

/* USART 0 */
SFR(U0CSR,     0x86); /* USART 0 control and status */
SFR(U0DBUF,    0xC1); /* USART 0 receive/transmit data buffer */
SFR(U0BAUD,    0xC2); /* USART 0 baud-rate control */
SFR(U0UCR,     0xC4); /* USART 0 UART control */
SFR(U0GCR,     0xC5); /* USART 0 generic control */

/* USART 1 */
SFRBIT(U1CSR,0xF8,MODE,RE,SLAVE,FE,ERR,RX_BYTE,TX_BYTE,ACTIVE); /* USART 1 control and status */
/* MODE: USART mode select */
/* RE: UART receiver enable */
/* SLAVE: SPI master- or slave mode select */
/* FE: UART framing error status */
/* ERR: UART parity error status */
/* RX_BYTE: Receive byte status */
/* TX_BYTE: Transmit byte status */
/* ACTIVE: USART transmit/receive active status */
SFR(U1DBUF,    0xF9); /* USART 1 receive/transmit data buffer */
SFR(U1BAUD,    0xFA); /* USART 1 baud-rate control */
SFR(U1UCR,     0xFB); /* USART 1 UART control */
SFR(U1GCR,     0xFC); /* USART 1 Generic control */

/* Watchdog Timer */
SFR(WDCTL,     0xC9); /* Watchdog Timer Control */
/*---------------------------------------------------------------------------
 * XREG Registers (0x6000â€?x63FF), excluding RF and USB registers
 * (Table 2.2, page 31)
 *---------------------------------------------------------------------------*/
#define  MONMUX      XREG(0x61A6) /* Operational amplifier mode control (cc2530/31) */
#define  OPAMPMC     XREG(0x61A6) /* Battery monitor MUX (cc2533) */
/* I2C registers - cc2533 only */
#define  I2CCFG      XREG(0x6230)  /* I2C control */
#define  I2CSTAT     XREG(0x6231)  /* I2C status */
#define  I2CDATA     XREG(0x6232)  /* I2C data */
#define  I2CADDR     XREG(0x6233)  /* I2C own slave address */
#define  I2CWC       XREG(0x6234)  /* Wrapper Control */
#define  I2CIO       XREG(0x6235)  /* GPIO */
/* End I2C registers */
#define  OBSSEL0     XREG(0x6243)  /* Observation output control - register 0 */
#define  OBSSEL1     XREG(0x6244)  /* Observation output control - register 1 */
#define  OBSSEL2     XREG(0x6245)  /* Observation output control - register 2 */
#define  OBSSEL3     XREG(0x6246)  /* Observation output control - register 3 */
#define  OBSSEL4     XREG(0x6247)  /* Observation output control - register 4 */
#define  OBSSEL5     XREG(0x6248)  /* Observation output control - register 5 */
#define  CHVER       XREG(0x6249)  /* Chip version */
#define  CHIPID      XREG(0x624A)  /* Chip identification */

/* TR0 below is renamed to TESTREG0 to avoid namespace conflicts with the
 * bit-addressable TCON.TR0 on the default 8051. See SDCC bug 3513300 */
#define  TESTREG0    XREG(0x624B)  /* Test register 0 */

#define  DBGDATA     XREG(0x6260)  /* Debug interface write data */
#define  SRCRC       XREG(0x6262)  /* Sleep reset CRC */
#define  BATTMON     XREG(0x6264)  /* Battery monitor */
#define  IVCTRL      XREG(0x6265)  /* Analog control register */
#define  FCTL        XREG(0x6270)  /* Flash control */
#define  FADDRL      XREG(0x6271)  /* Flash address low */
#define  FADDRH      XREG(0x6272)  /* Flash address high */
#define  FWDATA      XREG(0x6273)  /* Flash write data */
#define  CHIPINFO0   XREG(0x6276)  /* Chip information byte 0 */
#define  CHIPINFO1   XREG(0x6277)  /* Chip information byte 1 */
#define  IRCTL       XREG(0x6281)  /* Timer 1 IR generation control */
#define  CLD         XREG(0x6290)  /* Clock-loss detection */
#define  XX_T1CCTL0  XREG(0x62A0)  /* Timer 1 channel 0 capture/compare control (additional XREG mapping of SFR) */
#define  XX_T1CCTL1  XREG(0x62A1)  /* Timer 1 channel 1 capture/compare control (additional XREG mapping of SFR register) */
#define  XX_T1CCTL2  XREG(0x62A2)  /* Timer 1 channel 2 capture/compare control (additional XREG mapping of SFR register) */
#define  T1CCTL3     XREG(0x62A3)  /* Timer 1 channel 3 capture/compare control */
#define  T1CCTL4     XREG(0x62A4)  /* Timer 1 channel 4 capture/compare control */
#define  XX_T1CC0L   XREG(0x62A6)  /* Timer 1 channel 0 capture/compare value low (additional XREG mapping of SFR register) */
#define  XX_T1CC0H   XREG(0x62A7)  /* Timer 1 channel 0 capture/compare value high (additional XREG mapping of SFR register) */
#define  XX_T1CC1L   XREG(0x62A8)  /* Timer 1 channel 1 capture/compare value low (additional XREG mapping of SFR register) */
#define  XX_T1CC1H   XREG(0x62A9)  /* Timer 1 channel 1 capture/compare value high (additional XREG mapping of SFR register) */
#define  XX_T1CC2L   XREG(0x62AA)  /* Timer 1 channel 2 capture/compare value low (additional XREG mapping of SFR register) */
#define  XX_T1CC2H   XREG(0x62AB)  /* Timer 1 channel 2 capture/compare value high (additional XREG mapping of SFR register) */
#define  T1CC3L      XREG(0x62AC)  /* Timer 1 channel 3 capture/compare value low */
#define  T1CC3H      XREG(0x62AD)  /* Timer 1 channel 3 capture/compare value high */
#define  T1CC4L      XREG(0x62AE)  /* Timer 1 channel 4 capture/compare value low */
#define  T1CC4H      XREG(0x62AF)  /* Timer 1 channel 4 capture/compare value high */
#define  STCC        XREG(0x62B0)  /* Sleep Timer capture control */
#define  STCS        XREG(0x62B1)  /* Sleep Timer capture status */
#define  STCV0       XREG(0x62B2)  /* Sleep Timer capture value byte 0 */
#define  STCV1       XREG(0x62B3)  /* Sleep Timer capture value byte 1 */
#define  STCV2       XREG(0x62B4)  /* Sleep Timer capture value byte 2 */
#define  OPAMPC      XREG(0x62C0)  /* Operational amplifier control */
#define  OPAMPS      XREG(0x62C1)  /* Operational amplifier status */
#define  CMPCTL      XREG(0x62D0)  /* Analog comparator control and status */
/*---------------------------------------------------------------------------
 * Radio Registers
 * (Sec. 23, page 211)
 *---------------------------------------------------------------------------*/
#define  RFCORE_RAM     XREG(0x6000)  /* RF Core Memory Map (0x6000 to 0x0617F) */
#define  RXFIFO         XREG(0x6000)  /* TXFIFO Direct Access (0x6000 to 0x607F) */
#define  TXFIFO         XREG(0x6080)  /* TXFIFO Direct Access (0x6080 to 0x60FF) */

#define  SRC_ADDR_TABLE XREG(0x6100)  /* Source Address Table Start */

/* Source Address Matching Result */
#define  SRCRESMASK0    XREG(0x6160)  /* Extended address matching */
#define  SRCRESMASK1    XREG(0x6161)  /* Short address matching */
#define  SRCRESMASK2    XREG(0x6162)  /* Source address match - 24-bit mask */
#define  SRCRESINDEX    XREG(0x6163)  /* Bit index of least-significant 1 in SRCRESMASK */

/* Source Address Matching Control */
#define  SRCEXTPENDEN0  XREG(0x6164)  /* Ext. Address bit-mask 0 (LSB) */
#define  SRCEXTPENDEN1  XREG(0x6165)  /* Ext. Address bit-mask 1 */
#define  SRCEXTPENDEN2  XREG(0x6166)  /* Ext. Address bit-mask 2 (MSB) */
#define  SRCSHORTPENDEN0  XREG(0x6167)  /* Short Address bit-mask 0 (LSB) */
#define  SRCSHORTPENDEN1  XREG(0x6168)  /* Short Address bit-mask 1 */
#define  SRCSHORTPENDEN2  XREG(0x6169)  /* Short Address bit-mask 2 (MSB) */

/* Local Address Information (used during destination address filtering) */
#define  EXT_ADDR0        XREG(0x616A)  /* IEEE extended address 0 */
#define  EXT_ADDR1        XREG(0x616B)  /* IEEE extended address 1 */
#define  EXT_ADDR2        XREG(0x616C)  /* IEEE extended address 2 */
#define  EXT_ADDR3        XREG(0x616D)  /* IEEE extended address 3 */
#define  EXT_ADDR4        XREG(0x616E)  /* IEEE extended address 4 */
#define  EXT_ADDR5        XREG(0x616F)  /* IEEE extended address 5 */
#define  EXT_ADDR6        XREG(0x6170)  /* IEEE extended address 6 */
#define  EXT_ADDR7        XREG(0x6171)  /* IEEE extended address 7 */
#define  PAN_ID0          XREG(0x6172)  /* PAN ID 0 */
#define  PAN_ID1          XREG(0x6173)  /* PAN ID 1 */
#define  SHORT_ADDR0      XREG(0x6174)  /* Short Address 0 */
#define  SHORT_ADDR1      XREG(0x6175)  /* Short Address 1 */

#define  FRMFILT0         XREG(0x6180)  /* Frame Filtering 0 */
#define  FRMFILT1         XREG(0x6181)  /* Frame Filtering 1 */
#define  SRCMATCH         XREG(0x6182)  /* Source Address Matching and Pending Bits */
#define  SRCSHORTEN0      XREG(0x6183)  /* Short Address Matching 0 */
#define  SRCSHORTEN1      XREG(0x6184)  /* Short Address Matching 1 */
#define  SRCSHORTEN2      XREG(0x6185)  /* Short Address Matching 2 */
#define  SRCEXTEN0        XREG(0x6186)  /* Extended Address Matching 0 */
#define  SRCEXTEN1        XREG(0x6187)  /* Extended Address Matching 1 */
#define  SRCEXTEN2        XREG(0x6188)  /* Extended Address Matching 2 */
#define  FRMCTRL0         XREG(0x6189)  /* Frame Handling */
#define  FRMCTRL1         XREG(0x618A)  /* Frame Handling */
#define  RXENABLE         XREG(0x618B)  /* RX Enabling */
#define  RXMASKSET        XREG(0x618C)  /* RX Enabling */
#define  RXMASKCLR        XREG(0x618D)  /* RX Disabling */
#define  FREQTUNE         XREG(0x618E)  /* Crystal Oscillator Frequency Tuning */
#define  FREQCTRL         XREG(0x618F)  /* RF Frequency Control */
#define  TXPOWER          XREG(0x6190)  /* Controls the Output Power */
#define  TXCTRL           XREG(0x6191)  /* Controls the TX Settings */
#define  FSMSTAT0         XREG(0x6192)  /* Radio Status Register */
#define  FSMSTAT1         XREG(0x6193)  /* Radio Status Register */
#define  FIFOPCTRL        XREG(0x6194)  /* FIFOP Threshold */
#define  FSMCTRL          XREG(0x6195)  /* FSM Options */
#define  CCACTRL0         XREG(0x6196)  /* CCA Threshold */
#define  CCACTRL1         XREG(0x6197)  /* Other CCA Options */
#define  RSSI             XREG(0x6198)  /* RSSI Status Register */
#define  RSSISTAT         XREG(0x6199)  /* RSSI Valid Status Register */
#define  RXFIRST          XREG(0x619A)  /* First Byte in RXFIFO */
#define  RXFIFOCNT        XREG(0x619B)  /* Number of Bytes in RXFIFO */
#define  TXFIFOCNT        XREG(0x619C)  /* Number of Bytes in TXFIFO */
#define  RXFIRST_PTR      XREG(0x619D)  /* RXFIFO Pointer */
#define  RXLAST_PTR       XREG(0x619E)  /* RXFIFO Pointer */
#define  RXP1_PTR         XREG(0x619F)  /* RXFIFO Pointer */
#define  TXFIRST_PTR      XREG(0x61A1)  /* TXFIFO Pointer */
#define  TXLAST_PTR       XREG(0x61A2)  /* TXFIFO Pointer */
#define  RFIRQM0          XREG(0x61A3)  /* RF Interrupt Masks 0 */
#define  RFIRQM1          XREG(0x61A4)  /* RF Interrupt Masks 1 */
#define  RFERRM           XREG(0x61A5)  /* RF Error Interrupt Mask */
#define  RFRND            XREG(0x61A7)  /* Random Data */
#define  MDMCTRL0         XREG(0x61A8)  /* Controls Modem 0 */
#define  MDMCTRL1         XREG(0x61A9)  /* Controls Modem 1 */
#define  FREQEST          XREG(0x61AA)  /* Estimated RF Frequency Offset */
#define  RXCTRL           XREG(0x61AB)  /* Tune Receive Section */
#define  FSCTRL           XREG(0x61AC)  /* Tune Frequency Synthesizer */
#define  FSCAL0           XREG(0x61AD)  /* Tune Frequency Calibration 0 */
#define  FSCAL1           XREG(0x61AE)  /* Tune Frequency Calibration 1 */
#define  FSCAL2           XREG(0x61AF)  /* Tune Frequency Calibration 2 */
#define  FSCAL3           XREG(0x61B0)  /* Tune Frequency Calibration 3 */
#define  AGCCTRL0         XREG(0x61B1)  /* AGC Dynamic Range Control */
#define  AGCCTRL1         XREG(0x61B2)  /* AGC Reference Level */
#define  AGCCTRL2         XREG(0x61B3)  /* AGC Gain Override */
#define  AGCCTRL3         XREG(0x61B4)  /* AGC Control */
#define  ADCTEST0         XREG(0x61B5)  /* ADC Tuning 0 */
#define  ADCTEST1         XREG(0x61B6)  /* ADC Tuning 1 */
#define  ADCTEST2         XREG(0x61B7)  /* ADC Tuning 2 */
#define  MDMTEST0         XREG(0x61B8)  /* Test Register for Modem 0 */
#define  MDMTEST1         XREG(0x61B9)  /* Test Register for Modem 1 */
#define  DACTEST0         XREG(0x61BA)  /* DAC Override Value */
#define  DACTEST1         XREG(0x61BB)  /* DAC Override Value */
#define  DACTEST2         XREG(0x61BC)  /* DAC Test Setting */
#define  ATEST            XREG(0x61BD)  /* Analog Test Control */
#define  PTEST0           XREG(0x61BE)  /* Override Power-Down Register 0 */
#define  PTEST1           XREG(0x61BF)  /* Override Power-Down Register 1 */
#define  TXFILTCFG        XREG(0x61FA)  /*  TX Filter Configuration */
#define  RFC_OBS_CTRL0    XREG(0x61EB)  /* RF Observation Mux Control 0 */
#define  RFC_OBS_CTRL1    XREG(0x61EC)  /* RF Observation Mux Control 1 */
#define  RFC_OBS_CTRL2    XREG(0x61ED)  /* RF Observation Mux Control 2 */

/* Command Strobe/CSMA-CA Processor Registers */
#define  CSPPROG0         XREG(0x61C0)  /* CSP Program Memory, Byte 0 */
#define  CSPPROG1         XREG(0x61C1)  /* CSP Program Memory, Byte 1 */
#define  CSPPROG2         XREG(0x61C2)  /* CSP Program Memory, Byte 2 */
#define  CSPPROG3         XREG(0x61C3)  /* CSP Program Memory, Byte 3 */
#define  CSPPROG4         XREG(0x61C4)  /* CSP Program Memory, Byte 4 */
#define  CSPPROG5         XREG(0x61C5)  /* CSP Program Memory, Byte 5 */
#define  CSPPROG6         XREG(0x61C6)  /* CSP Program Memory, Byte 6 */
#define  CSPPROG7         XREG(0x61C7)  /* CSP Program Memory, Byte 7 */
#define  CSPPROG8         XREG(0x61C8)  /* CSP Program Memory, Byte 8 */
#define  CSPPROG9         XREG(0x61C9)  /* CSP Program Memory, Byte 9 */
#define  CSPPROG10        XREG(0x61CA)  /* CSP Program Memory, Byte 10 */
#define  CSPPROG11        XREG(0x61CB)  /* CSP Program Memory, Byte 11 */
#define  CSPPROG12        XREG(0x61CC)  /* CSP Program Memory, Byte 12 */
#define  CSPPROG13        XREG(0x61CD)  /* CSP Program Memory, Byte 13 */
#define  CSPPROG14        XREG(0x61CE)  /* CSP Program Memory, Byte 14 */
#define  CSPPROG15        XREG(0x61CF)  /* CSP Program Memory, Byte 15 */
#define  CSPPROG16        XREG(0x61D0)  /* CSP Program Memory, Byte 16 */
#define  CSPPROG17        XREG(0x61D1)  /* CSP Program Memory, Byte 17 */
#define  CSPPROG18        XREG(0x61D2)  /* CSP Program Memory, Byte 18 */
#define  CSPPROG19        XREG(0x61D3)  /* CSP Program Memory, Byte 19 */
#define  CSPPROG20        XREG(0x61D4)  /* CSP Program Memory, Byte 20 */
#define  CSPPROG21        XREG(0x61D5)  /* CSP Program Memory, Byte 21 */
#define  CSPPROG22        XREG(0x61D6)  /* CSP Program Memory, Byte 22 */
#define  CSPPROG23        XREG(0x61D7)  /* CSP Program Memory, Byte 23 */
#define  CSPCTRL          XREG(0x61E0)  /* CSP Control Bit */
#define  CSPSTAT          XREG(0x61E1)  /* CSP Status Register */
#define  CSPX             XREG(0x61E2)  /* CSP X Register */
#define  CSPY             XREG(0x61E3)  /* CSP Y Register */
#define  CSPZ             XREG(0x61E4)  /* CSP Z Register */
#define  CSPT             XREG(0x61E5)  /* CSP T Register */
/*---------------------------------------------------------------------------
 * cc2531 USB Registers
 * (Sec. 21.12, page 196)
 *---------------------------------------------------------------------------*/
#define  USBADDR   XREG(0x6200)  /* Function Address */
#define  USBPOW    XREG(0x6201)  /* Power/Control Register */
#define  USBIIF    XREG(0x6202)  /* IN Endpoints and EP0 Interrupt Flags */
#define  USBOIF    XREG(0x6204)  /* OUT-Endpoint Interrupt Flags */
#define  USBCIF    XREG(0x6206)  /* Common USB Interrupt Flags */
#define  USBIIE    XREG(0x6207)  /* IN Endpoints and EP0 Interrupt-Enable Mask */
#define  USBOIE    XREG(0x6209)  /* Out Endpoints Interrupt Enable Mask */
#define  USBCIE    XREG(0x620B)  /* Common USB Interrupt-Enable Mask */
#define  USBFRML   XREG(0x620C)  /* Current Frame Number (Low Byte) */
#define  USBFRMH   XREG(0x620D)  /* Current Frame Number (High Byte) */
#define  USBINDEX  XREG(0x620E)  /* Current-Endpoint Index Register */
#define  USBCTRL   XREG(0x620F)  /* USB Control Register */
#define  USBMAXI   XREG(0x6210)  /* Max. Packet Size for IN Endpoint{1â€?} */
#define  USBCS0    XREG(0x6211)  /* EP0 Control and Status (USBINDEX = 0) */
#define  USBCSIL   XREG(0x6211)  /* IN EP{1â€?} Control and Status, Low */
#define  USBCSIH   XREG(0x6212)  /* IN EP{1â€?} Control and Status, High */
#define  USBMAXO   XREG(0x6213)  /* Max. Packet Size for OUT EP{1â€?} */
#define  USBCSOL   XREG(0x6214)  /* OUT EP{1â€?} Control and Status, Low */
#define  USBCSOH   XREG(0x6215)  /* OUT EP{1â€?} Control and Status, High */
#define  USBCNT0   XREG(0x6216)  /* Number of Received Bytes in EP0 FIFO (USBINDEX = 0) */
#define  USBCNTL   XREG(0x6216)  /* Number of Bytes in EP{1â€?} OUT FIFO, Low */
#define  USBCNTH   XREG(0x6217)  /* Number of Bytes in EP{1â€?} OUT FIFO, High */
#define  USBF0     XREG(0x6220)  /* Endpoint-0 FIFO */
#define  USBF1     XREG(0x6222)  /* Endpoint-1 FIFO */
#define  USBF2     XREG(0x6224)  /* Endpoint-2 FIFO */
#define  USBF3     XREG(0x6226)  /* Endpoint-3 FIFO */
#define  USBF4     XREG(0x6228)  /* Endpoint-4 FIFO */
#define  USBF5     XREG(0x622A)  /* Endpoint-5 FIFO */
/*---------------------------------------------------------------------------
 * SFR Access via XDATA (0x7080 - 0x70FF)
 *---------------------------------------------------------------------------*/
#define  X_P0         XREG(0x7080)  /* Port 0 - Read Only */
#define  X_U0CSR      XREG(0x7086)  /* USART 0 control and status */
#define  X_P0IFG      XREG(0x7089)  /* Port 0 interrupt status flag */
#define  X_P1IFG      XREG(0x708A)  /* Port 1 interrupt status flag */
#define  X_P2IFG      XREG(0x708B)  /* Port 2 interrupt status flag */
#define  X_PICTL      XREG(0x708C)  /* Port pins interrupt mask and edge */
#define  X_P1IEN      XREG(0x708D)  /* Port 1 interrupt mask */
#define  X_P0INP      XREG(0x708F)  /* Port 0 input Mode */
#define  X_P1         XREG(0x7090)  /* Port 1 - Read Only */
#define  X_RFIRQF1    XREG(0x7091)  /* RF interrupt flags MSB */
#define  X_MPAGE      XREG(0x7093)  /* Memory page select */
#define  X__XPAGE     XREG(0x7093)  /* Memory page select - SDCC name */
#define  X_T2CTRL     XREG(0x7094)  /* Timer 2 control */
#define  X_ST0        XREG(0x7095)  /* Sleep Timer 0 */
#define  X_ST1        XREG(0x7096)  /* Sleep Timer 1 */
#define  X_ST2        XREG(0x7097)  /* Sleep Timer 2 */
#define  X_T2EVTCFG   XREG(0x709C)  /* Timer 2 event configuration */
#define  X_SLEEPSTA   XREG(0x709D)  /* Sleep-mode control status */
#define  X_CLKCONSTA  XREG(0x709E)  /* Clock control status */
#define  X_FMAP       XREG(0x709F)  /* Flash-memory bank mapping */
#define  X_PSBANK     XREG(0x709F)  /* Flash-memory bank mapping - SDCC name */
#define  X_P2         XREG(0x70A0)  /* Port 2 - Read Only */
#define  X_T2IRQF     XREG(0x70A1)  /* Timer 2 interrupt flags */
#define  X_T2M0       XREG(0x70A2)  /* Timer 2 multiplexed register 0 */
#define  X_T2M1       XREG(0x70A3)  /* Timer 2 multiplexed register 1 */
#define  X_T2MOVF0    XREG(0x70A4)  /* Timer 2 multiplexed overflow register 0 */
#define  X_T2MOVF1    XREG(0x70A5)  /* Timer 2 multiplexed overflow register 1 */
#define  X_T2MOVF2    XREG(0x70A6)  /* Timer 2 multiplexed overflow register 2 */
#define  X_T2IRQM     XREG(0x70A7)  /* Timer 2 interrupt mask */
#define  X_P0IEN      XREG(0x70AB)  /* Port 0 interrupt mask */
#define  X_P2IEN      XREG(0x70AC)  /* Port 2 interrupt mask */
#define  X_STLOAD     XREG(0x70AD)  /* Sleep-timer load status */
#define  X_PMUX       XREG(0x70AE)  /* Power-down signal mux */
#define  X_T1STAT     XREG(0x70AF)  /* Timer 1 status */
#define  X_ENCDI      XREG(0x70B1)  /* Encryption/decryption input data */
#define  X_ENCDO      XREG(0x70B2)  /* Encryption/decryption output data */
#define  X_ENCCS      XREG(0x70B3)  /* Encryption/decryption control and status */
#define  X_ADCCON1    XREG(0x70B4)  /* ADC control 1 */
#define  X_ADCCON2    XREG(0x70B5)  /* ADC control 2 */
#define  X_ADCCON3    XREG(0x70B6)  /* ADC control 3 */
#define  X_ADCL       XREG(0x70BA)  /* ADC data low */
#define  X_ADCH       XREG(0x70BB)  /* ADC data high */
#define  X_RNDL       XREG(0x70BC)  /* Random number generator data low */
#define  X_RNDH       XREG(0x70BD)  /* Random number generator data high */
#define  X_SLEEPCMD   XREG(0x70BE)  /* Sleep-mode control command */
#define  X_RFERRF     XREG(0x70BF)  /* RF error interrupt flags */
#define  X_U0DBUF     XREG(0x70C1)  /* USART 0 receive/transmit data buffer */
#define  X_U0BAUD     XREG(0x70C2)  /* USART 0 baud-rate control */
#define  X_T2MSEL     XREG(0x70C3)  /* Timer 2 multiplex select */
#define  X_U0UCR      XREG(0x70C4)  /* USART 0 UART control */
#define  X_U0GCR      XREG(0x70C5)  /* USART 0 generic control */
#define  X_CLKCONCMD  XREG(0x70C6)  /* Clock control command */
#define  X_MEMCTR     XREG(0x70C7)  /* Memory system control */
#define  X_WDCTL      XREG(0x70C9)  /* Watchdog Timer Control */
#define  X_T3CNT      XREG(0x70CA)  /* Timer 3 counter */
#define  X_T3CTL      XREG(0x70CB)  /* Timer 3 control */
#define  X_T3CCTL0    XREG(0x70CC)  /* Timer 3 channel 0 compare control */
#define  X_T3CC0      XREG(0x70CD)  /* Timer 3 channel 0 compare value */
#define  X_T3CCTL1    XREG(0x70CE)  /* Timer 3 channel 1 compare control */
#define  X_T3CC1      XREG(0x70CF)  /* Timer 3 channel 1 compare value */
#define  X_DMAIRQ     XREG(0x70D1)  /* DMA interrupt flag */
#define  X_DMA1CFGL   XREG(0x70D2)  /* DMA channel 1â€? configuration address low */
#define  X_DMA1CFGH   XREG(0x70D3)  /* DMA channel 1â€? configuration address high */
#define  X_DMA0CFGL   XREG(0x70D4)  /* DMA channel 0 configuration address low */
#define  X_DMA0CFGH   XREG(0x70D5)  /* DMA channel 0 configuration address high */
#define  X_DMAARM     XREG(0x70D6)  /* DMA channel armed */
#define  X_DMAREQ     XREG(0x70D7)  /* DMA channel start request and status */
#define  X_TIMIF      XREG(0x70D8)  /* Timers 1/3/4 joint interrupt mask/flags */
#define  X_RFD        XREG(0x70D9)  /* RF data */
#define  X_T1CC0L     XREG(0x70DA)  /* Timer 1 channel 0 capture/compare value low */
#define  X_T1CC0H     XREG(0x70DB)  /* Timer 1 channel 0 capture/compare value high */
#define  X_T1CC1L     XREG(0x70DC)  /* Timer 1 channel 1 capture/compare value low */
#define  X_T1CC1H     XREG(0x70DD)  /* Timer 1 channel 1 capture/compare value high */
#define  X_T1CC2L     XREG(0x70DE)  /* Timer 1 channel 2 capture/compare value low */
#define  X_T1CC2H     XREG(0x70DF)  /* Timer 1 channel 2 capture/compare value high */
#define  X_RFST       XREG(0x70E1)  /* RF command strobe */
#define  X_T1CNTL     XREG(0x70E2)  /* Timer 1 counter low */
#define  X_T1CNTH     XREG(0x70E3)  /* Timer 1 counter high */
#define  X_T1CTL      XREG(0x70E4)  /* Timer 1 control and status */
#define  X_T1CCTL0    XREG(0x70E5)  /* Timer 1 channel 0 capture/compare control */
#define  X_T1CCTL1    XREG(0x70E6)  /* Timer 1 channel 1 capture/compare control */
#define  X_T1CCTL2    XREG(0x70E7)  /* Timer 1 channel 2 capture/compare control */
#define  X_RFIRQF0    XREG(0x70E9)  /* RF interrupt flags LSB */
#define  X_T4CNT      XREG(0x70EA)  /* Timer 4 counter */
#define  X_T4CTL      XREG(0x70EB)  /* Timer 4 control */
#define  X_T4CCTL0    XREG(0x70EC)  /* Timer 4 channel 0 compare control */
#define  X_T4CC0      XREG(0x70ED)  /* Timer 4 channel 0 compare value */
#define  X_T4CCTL1    XREG(0x70EE)  /* Timer 4 channel 1 compare control */
#define  X_T4CC1      XREG(0x70EF)  /* Timer 4 channel 1 compare value */
#define  X_PERCFG     XREG(0x70F1)  /* Peripheral I/O control */
#define  X_APCFG      XREG(0x70F2)  /* Analog peripheral I/O configuration */
#define  X_P0SEL      XREG(0x70F3)  /* Port 0 function select */
#define  X_P1SEL      XREG(0x70F4)  /* Port 1 function select */
#define  X_P2SEL      XREG(0x70F5)  /* Port 2 function select */
#define  X_P1INP      XREG(0x70F6)  /* Port 1 input mode */
#define  X_P2INP      XREG(0x70F7)  /* Port 2 input mode */
#define  X_U1CSR      XREG(0x70F8)  /* USART 1 control and status */
#define  X_U1DBUF     XREG(0x70F9)  /* USART 1 receive/transmit data buffer */
#define  X_U1BAUD     XREG(0x70FA)  /* USART 1 baud-rate control */
#define  X_U1UCR      XREG(0x70FB)  /* USART 1 UART control */
#define  X_U1GCR      XREG(0x70FC)  /* USART 1 Generic control */
#define  X_P0DIR      XREG(0x70FD)  /* Port 0 direction */
#define  X_P1DIR      XREG(0x70FE)  /* Port 1 direction */
#define  X_P2DIR      XREG(0x70FF)  /* Port 2 direction */
/*---------------------------------------------------------------------------
 * Information Page (Read Only)
 *---------------------------------------------------------------------------*/
#define  X_INFOPAGE   XREG(0x7800)  /* Start of Information Page */
#define  X_IEEE_ADDR  XREG(0x780C)  /* Start of unique IEEE Address */

#endif /* __CC253X_H__ */
