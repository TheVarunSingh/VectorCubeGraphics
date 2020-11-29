#include "spi.h"

void configureSPI(SPI_TypeDef * SPIx) {
    SPIx->CR1 &= ~(SPI_CR1_SPE); // Disable SPI so we can change all the settings
                                 // I assume user calls this on startup, but if you want to change
                                 // midway, there may be more complicated shutdown procedures
    // Baud Rate: divide by 4
    SPIx->CR1 &= ~(SPI_CR1_BR);
    SPIx->CR1 |= (0b001 << SPI_CR1_BR_Pos);

    SPIx->CR1 &= ~(SPI_CR1_CPOL); // Set polarity to idle low
    SPIx->CR1 &= ~(SPI_CR1_CPHA); // Set phase to leading edge

    SPIx->CR1 |= SPI_CR1_DFF; // 16-bit data
    SPIx->CR1 |= SPI_CR1_LSBFIRST; // LSB first

    SPIx->CR1 &= ~(SPI_CR1_BIDIMODE); // For full-duplex, we use both wires, and each is unidirectional
    SPIx->CR1 &= ~(SPI_CR1_RXONLY); // For full-duplex, we don't want receive only
    SPIx->CR1 |= SPI_CR1_MSTR; // Make STM32 act as the master
    SPIx->CR1 |= SPI_CR1_SSM; // To give software control of nSS...
    SPIx->CR1 |= SPI_CR1_SSI; // ...and set internal nSS. You could also set SSOE instead.
    SPIx->CR1 &= ~(SPI_CR1_CRCEN); // Disable CRC calculation by default; if you need it, write your own lib!
    SPIx->CR1 |= SPI_CR1_SPE; // Enable SPI
}

uint16_t sendReceiveSPI(SPI_TypeDef * SPIx, uint16_t data) {
    // Transmit
    while (!(SPIx->SR & SPI_SR_TXE)); // Wait until TX buffer is ready for data to be written
    SPIx->DR = data; // load data into TX buffer to begin transfer

    // Receive
    while (!(SPIx->SR & SPI_SR_RXNE)); // Wait until RX buffer is ready for data to be read
    uint16_t message = SPIx->DR; // Read data from RX buffer
    return message;
}

void doubleSendSPI(SPI_TypeDef* SPIx, SPI_TypeDef* SPIy, uint16_t dataX, uint16_t dataY) {
    /* Small optimization for sending to two SPI ports */
    SPIx->DR = dataX;
    SPIy->DR = dataY;
    /*
    // Transmit
    int xSent = 0; // has x data been sent yet
    int ySent = 0; // has y data been sent yet
    while ((xSent == 0) || (ySent == 0)) {
        if ((xSent == 0) && (SPIx->SR & SPI_SR_TXE)) { // wait until TX buffer is ready for data to be written
            SPIx->DR = dataX; // load data into TX buffer to begin transfer
            xSent = 1; // flag x data as sent
        }
        if ((ySent == 0) && (SPIy->SR & SPI_SR_TXE)) { // wait until TX buffer is ready for data to be written
            SPIy->DR = dataY; // load data into TX buffer to begin transfer
            ySent = 1; // flag y data as sent
        }
    }*/
}