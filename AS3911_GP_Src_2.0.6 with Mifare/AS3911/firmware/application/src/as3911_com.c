/*
 *****************************************************************************
 * Copyright by ams AG                                                       *
 * All rights are reserved.                                                  *
 *                                                                           *
 * IMPORTANT - PLEASE READ CAREFULLY BEFORE COPYING, INSTALLING OR USING     *
 * THE SOFTWARE.                                                             *
 *                                                                           *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS       *
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT         *
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS         *
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT  *
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,     *
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT          *
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,     *
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY     *
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT       *
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE     *
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.      *
 *****************************************************************************
 */
/*
 *      PROJECT:   AS3911 firmware
 *      $Revision: $
 *      LANGUAGE:  ANSI C
 */

/*! \file
 *
 *  \author Christian Eisendle
 *
 *  \brief Implementation of AS3911 communication.
 *
 */

/*
******************************************************************************
* INCLUDES
******************************************************************************
*/
#include "platform.h"
#include "as3911_com.h"
#include "as3911.h"
#include "spi_driver.h"
#include "board.h"
#include "delay.h"

/*
******************************************************************************
* LOCAL DEFINES
******************************************************************************
*/
#define AS3911_SS_PIN BOARD_PIN_AS3911_SEN

#define AS3911_SPI_SELECT() AS3911_SS_PIN = 0
#define AS3911_SPI_DESELECT() AS3911_SS_PIN = 1

#define AS3911_WRITE_MODE  (0)
#define AS3911_READ_MODE   (1 << 6)
#define AS3911_FIFO_LOAD   (2 << 6)
#define AS3911_FIFO_READ   (0xbf)
#define AS3911_CMD_MODE    (3 << 6)

extern volatile u32 as3911InterruptStatus;
extern volatile u32 as3911InterruptMask;
/*
******************************************************************************
* LOCAL FUNCTION PROTOTYPES
******************************************************************************
*/
static s8 as3911Write(u8 cmd, const u8* values, u8 length);

/*
******************************************************************************
* GLOBAL FUNCTIONS
******************************************************************************
*/
s8 as3911ReadRegister(u8 reg, u8* val)
{
    u8 buf[2];
    s8 err;

    buf[0] = reg | AS3911_READ_MODE;
    buf[1] = 0;

    IRQ_INC_DISABLE();

    AS3911_SPI_SELECT();
    err = spiTxRx(buf, buf, 2);
    AS3911_SPI_DESELECT();
    *val = buf[1];

    IRQ_DEC_ENABLE();

    return err;
}

s8 as3911ReadMultipleRegisters(u8 reg, u8* val, u8 length)
{
    s8 err;
    u8 lastbyte;
    u8 i;

    val[0] = reg | AS3911_READ_MODE;

    /* make this operation atomic */
    IRQ_INC_DISABLE();
    /* since the result is right shited by one byte we can't receive
       all bytes within one transfer. So we have to do another transfer
       for receiving the last byte */
    AS3911_SPI_SELECT();
    err = spiTxRx(val, val, length);
    if (ERR_NONE != err)
    {
        goto out;
    }
    err = spiTxRx(&lastbyte, &lastbyte, 1);

    /* left shift all bytes by one position */
    for (i = 0; i < (length - 1); i++)
    {
        val[i] = val[i+1];
    }
    val[i] = lastbyte;

out:
    AS3911_SPI_DESELECT();
    IRQ_DEC_ENABLE();
    return err;
}

s8 as3911WriteTestRegister(u8 reg, u8 val)
{
    s8 err;
    u8 buf[3];

    buf[0] = AS3911_CMD_TEST_ACCESS;
    buf[1] = reg | AS3911_WRITE_MODE;
    buf[2] = val;

    IRQ_INC_DISABLE();
    AS3911_SPI_SELECT();
    err = spiTxRx(buf, NULL, 3);
    AS3911_SPI_DESELECT();
    IRQ_DEC_ENABLE();

    return err;
}

s8 as3911WriteRegister(u8 reg, u8 val)
{
    s8 err;
    u8 buf[2];

    buf[0] = reg | AS3911_WRITE_MODE;
    buf[1] = val;

    IRQ_INC_DISABLE();
    AS3911_SPI_SELECT();
    err = spiTxRx(buf, NULL, 2);
    AS3911_SPI_DESELECT();
    IRQ_DEC_ENABLE();

    return err;
}

s8 as3911ModifyRegister(u8 reg, u8 clr_mask, u8 set_mask)
{
    s8 err;
    u8 tmp;

    /* make this operation atomic */
    IRQ_INC_DISABLE();

    err = as3911ReadRegister(reg, &tmp);

    if (ERR_NONE == err)
    {
        /* mask out the bits we don't want to change */
        tmp &= ~clr_mask;
        /* set the new value */
        tmp |= set_mask;
        err = as3911WriteRegister(reg, tmp);
    }
    IRQ_DEC_ENABLE();

    return err;
}

s8 as3911WriteMultipleRegisters(u8 reg, const u8* values, u8 length)
{
    reg |= AS3911_WRITE_MODE;

    return as3911Write(reg, values, length);
}


s8 as3911WriteFifo(const u8* values, u8 length)
{
    s8 err;
    u8 cmd = AS3911_FIFO_LOAD;

    /* make this operation atomic */
    IRQ_INC_DISABLE();
    {
        AS3911_SPI_SELECT();
        {
            err = spiTxRx(&cmd, NULL, 1);

            if (ERR_NONE == err)
            {
                err = spiTxRx(values, NULL, length);
            }
        }
        AS3911_SPI_DESELECT();
    }
    IRQ_DEC_ENABLE();

    return err;
}

s8 as3911ReadFifo(u8* buf, u8 length)
{
    s8 err = ERR_NONE;
    u8 cmd = AS3911_FIFO_READ;

    if (length > 0)
    {
        /* make this operation atomic */
        IRQ_INC_DISABLE();
        AS3911_SPI_SELECT();
        err = spiTxRx(&cmd, NULL, 1);
        if (ERR_NONE == err)
        {
            err = spiTxRx(buf, buf, length);
        }
        AS3911_SPI_DESELECT();
        IRQ_DEC_ENABLE();
    }

    return err;
}

s8 as3911ExecuteCommand(u8 cmd)
{
    s8 err;
    cmd |= AS3911_CMD_MODE;

    IRQ_INC_DISABLE();
    AS3911_SPI_SELECT();
    err = spiTxRx(&cmd, NULL, 1);
    AS3911_SPI_DESELECT();
    IRQ_DEC_ENABLE();

    return err;
}

s8 as3911TestPrefix()
{
    s8 err;
    u8 cmd = AS3911_CMD_TEST_ACCESS;
    AS3911_SPI_SELECT();
    err = spiTxRx(&cmd, NULL, 1);

    return err;
}

/*
******************************************************************************
* LOCAL FUNCTIONS
******************************************************************************
*/
static s8 as3911Write(u8 cmd, const u8* values, u8 length)
{
    s8 err;

    /* make this operation atomic */
    IRQ_INC_DISABLE();
    AS3911_SPI_SELECT();
    err = spiTxRx(&cmd, NULL, 1);

    if (ERR_NONE == err)
    {
        err = spiTxRx(values, NULL, length);
    }
    AS3911_SPI_DESELECT();
    IRQ_DEC_ENABLE();

    return err;
}

