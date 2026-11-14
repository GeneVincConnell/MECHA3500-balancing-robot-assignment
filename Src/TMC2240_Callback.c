/**
 * @file    TMC2240_Callback.c
 * @brief   The callbacks ADI's TMC-API calls to reach the TMC2240s on this board.
 *
 * PROVENANCE: AI-generated. Written by Claude (Anthropic), Nov 2026, following
 * the integration instructions in the TMC-API README
 * (TMC-API-master/tmc/ic/TMC2240/README.md). Moved here from Motor_test.c on
 * 10 Nov 2026; the code is unchanged.
 *
 * TMC-API (TMC2240.c/.h, downloaded, MIT licence, Analog Devices) declares
 * these four functions in TMC2240.h and calls them from TMC2240.c whenever it
 * reads or writes a register. The library knows the chip's registers and
 * datagram format; these functions supply the board-specific part: which bus,
 * which chip-select pin, and which SPI peripheral. Nothing in this project
 * calls them directly.
 *
 * Board wiring (from the schematic):
 *   icID 0 = TMC_LEFT  = U3, chip select PB12 (TMC_CS_L)
 *   icID 1 = TMC_RIGHT = U4, chip select PB1  (TMC_CS_R)
 *   Both chips share SPI2 (PB13 SCK, PB14 MISO, PB15 MOSI), mode 3.
 */
#include "Robot.h"     /* TMC2240.h, pin names, hspi2, TMC_LEFT / TMC_RIGHT */
#include <string.h>    /* memset */

/* Which bus to use for each chip. Called before every register access. */
TMC2240BusType tmc2240_getBusType(uint16_t icID)
{
    (void)icID;
    return IC_BUS_SPI;
}

/* One SPI datagram. TMC-API passes the bytes to send in data[] and expects
   the received bytes back in the same buffer. */
void tmc2240_readWriteSPI(uint16_t icID, uint8_t *data, size_t dataLength)
{
    GPIO_TypeDef *cs_port = (icID == TMC_LEFT) ? TMC_CS_L_GPIO_Port : TMC_CS_R_GPIO_Port;
    uint16_t      cs_pin  = (icID == TMC_LEFT) ? TMC_CS_L_Pin       : TMC_CS_R_Pin;

    HAL_GPIO_WritePin(cs_port, cs_pin, GPIO_PIN_RESET);
    if (HAL_SPI_TransmitReceive(&hspi2, data, data, (uint16_t)dataLength, 10U) != HAL_OK) {
        memset(data, 0, dataLength);   /* reads back as 0 -> treated as no reply */
    }
    HAL_GPIO_WritePin(cs_port, cs_pin, GPIO_PIN_SET);

    /* Keep CS high for ~1 us before the next datagram */
    for (volatile uint32_t i = 0; i < 20U; i++) { }
}

/* UART is not used on this board, but TMC-API links against these. */
bool tmc2240_readWriteUART(uint16_t icID, uint8_t *data, size_t writeLength, size_t readLength)
{
    (void)icID; (void)data; (void)writeLength; (void)readLength;
    return false;
}

uint8_t tmc2240_getNodeAddress(uint16_t icID)
{
    (void)icID;
    return 0U;
}
