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
 *      $Revision: $
 *      LANGUAGE:  ANSI C
 */

/*! \file
 *
 *  \author Wolfgang Reichart
 *
 *  \brief SPI driver for PIC24F.
 *
 */

/*
******************************************************************************
* INCLUDES
******************************************************************************
*/
#define USE_AND_OR
#include <p24Fxxxx.h>
#include <spi.h>
#include <string.h>
#include "ams_types.h"
#include "GenericTypeDefs.h"
#include "errno.h"
#include "spi_driver.h"
#include "logger.h"

/*
******************************************************************************
* LOCAL MACROS
******************************************************************************
*/
#define SPI_FIFO_DEPTH                  8

#define SPI_MAX_FREQ_CONF_TABLE_ENTRIES 5

#define spiReadStat() (*hw->stat)
#define spiReadCon1() (*hw->con1)
#define spiReadCon2() (*hw->con2)
#define spiReadBuf()  (*hw->buf)

#define spiWriteStat(A) *hw->stat = (A)
#define spiWriteCon1(A) *hw->con1 = (A)
#define spiWriteCon2(A) *hw->con2 = (A)
#define spiWriteBuf(A)  *hw->buf  = (A)

/*
******************************************************************************
* LOCAL DATATYPES
******************************************************************************
*/
typedef struct
{
    u32 systemFrequency;
    u32 frequency;
    u32 actFrequency;
    u8 priPreScalerReg;
    u16 secPreScaler;
} spiFrequencyConfiguration_t;

struct spiHw
{
    volatile unsigned int  *stat;
    volatile unsigned int  *con1;
    volatile unsigned int  *con2;
    volatile unsigned int  *buf ;
};
/*
******************************************************************************
* LOCAL VARIABLES
******************************************************************************
*/
static umword spiTableIndex = 0;
static spiFrequencyConfiguration_t spiFreqConfTable[SPI_MAX_FREQ_CONF_TABLE_ENTRIES];

static const struct spiHw spiController[2] =
{
    {
        .stat = &SPI1STAT,
        .con1 = &SPI1CON1,
        .con2 = &SPI1CON2,
        .buf  = &SPI1BUF
    },
    {
        .stat = &SPI2STAT,
        .con1 = &SPI2CON1,
        .con2 = &SPI2CON2,
        .buf  = &SPI2BUF
    }
};
/*
******************************************************************************
* LOCAL VARIABLES
******************************************************************************
*/
static spiConfig_t myCfgData;

/*
******************************************************************************
* LOCAL FUNCTIONS
******************************************************************************
*/
/*
******************************************************************************
* GLOBAL FUNCTIONS
******************************************************************************
*/



void spiSerialiseConfig( const spiConfig_t * config, u8 * data )
{
    data[0] = config->frequency >> 24;
    data[1] = config->frequency >> 16;
    data[2] = config->frequency >> 8;
    data[3] = config->frequency;
    data[4] = config->instance;
    data[5] = config->deviceId;
    data[6] = config->clockPhase;
    data[7] = config->clockPolarity;
}

void spiDeserialiseConfig( spiConfig_t * config, const u8 * data )
{
    config->frequency = (((u32) data[0]) << 24) | (((u32) data[1]) << 16) | (((u32) data[2]) << 8) | (((u32) data[3]));
    config->instance = data[4];
    config->deviceId = data[5];
    config->clockPhase = data[6];
    config->clockPolarity = data[7];
}

s8 spiInitialize(u32 sysClk, const spiConfig_t* const spiConfigIn, spiConfig_t *spiConfigOut)
{
    u32 systemClock;
    u16 priprescaler;
    u8 priprescalerreg;
    u16 secprescaler;
    u32 actFrequency;
    spiFrequencyConfiguration_t* tblptr;
    umword i;
    BOOL configFound = FALSE;
    const struct spiHw *hw;

    if ( spiConfigIn == NULL || spiConfigIn->instance >= SPI_MAX_INSTANCES )
    {
        /* avoid severe pointer-arithmetik errors */
        return ERR_PARAM;
    }

    if (spiConfigOut != NULL)
    {
        spiConfigOut->clockPhase    = myCfgData.clockPhase;
        spiConfigOut->clockPolarity = myCfgData.clockPolarity;
        spiConfigOut->deviceId      = myCfgData.deviceId;
        spiConfigOut->frequency     = myCfgData.frequency;
        spiConfigOut->instance      = myCfgData.instance;
    }
    myCfgData.clockPhase    = spiConfigIn->clockPhase;
    myCfgData.clockPolarity = spiConfigIn->clockPolarity;
    myCfgData.deviceId      = spiConfigIn->deviceId;
    myCfgData.frequency     = spiConfigIn->frequency;
    myCfgData.instance      = spiConfigIn->instance;
    hw = spiController + myCfgData.instance;

    systemClock = sysClk;

    /* disable SPI for configuration */
    spiWriteStat(0);

    /* first check the cache table for already calculated frequencies */
    for (i = 0; i < SPI_MAX_FREQ_CONF_TABLE_ENTRIES; i++)
    {
        if ((spiFreqConfTable[i].frequency == myCfgData.frequency) && (spiFreqConfTable[i].systemFrequency == systemClock))
        {
            /* found a previously configured setting */
            configFound = TRUE;
            tblptr = &spiFreqConfTable[i];
            break;
        }
    }

    if (!configFound)
    {
        /* equation according to the datasheet for calculating spi clock speed:
         * fsck = SYSCLK / (pri_prescaler * sec_prescaler)
         */
        priprescaler = 1;
        priprescalerreg = 3;
        do
        {
            secprescaler = systemClock / (myCfgData.frequency * priprescaler);
            if (secprescaler > 8)
            {
                if (priprescaler >= 64)
                {
                    secprescaler = (secprescaler > 8) ? 8 : secprescaler;
                    break;
                }
                else
                {
                    priprescaler <<= 2;
                    priprescalerreg--;
                    continue;
                }
            }
            else
            {
                break;
            }

        }
        while(1);

        secprescaler = (secprescaler == 0) ? 1 : secprescaler;
        actFrequency = systemClock / (priprescaler * secprescaler);

        /* save this new configuration in the cache table */
        if (spiTableIndex >= SPI_MAX_FREQ_CONF_TABLE_ENTRIES)
        {
            spiTableIndex = 0;
        }
        spiFreqConfTable[spiTableIndex].actFrequency = actFrequency;
        spiFreqConfTable[spiTableIndex].frequency = myCfgData.frequency;
        spiFreqConfTable[spiTableIndex].priPreScalerReg = priprescalerreg;
        spiFreqConfTable[spiTableIndex].secPreScaler = secprescaler;
        spiFreqConfTable[spiTableIndex].systemFrequency = systemClock;
        spiTableIndex++;
    }
    else
    {
        actFrequency = tblptr->actFrequency;
        myCfgData.frequency = tblptr->frequency;
        priprescalerreg = tblptr->priPreScalerReg;
        secprescaler = tblptr->secPreScaler;
    }

    /* set pre-scaler */
    spiWriteCon1(priprescalerreg | ((8 - secprescaler) << 2));

    /* set clock and polarity */
    spiWriteCon1(spiReadCon1() |
                 (myCfgData.clockPhase << 8) |
                 (myCfgData.clockPolarity << 6));

    /* set enhanced buffer master mode and enable SPI */
    spiWriteCon2(1);
    spiWriteCon1(spiReadCon1() | (1 << 5));
    spiWriteStat((1 << 15));

    SPI_LOG( "\nSPI M=%hhx, des.f=%u%u, act.f=%u%u, clkPhase=%hhx, clkPol=%hhx, ss=%hhx\n"
             , myCfgData.instance
             , (u16)(myCfgData.frequency >> 16)
             , (u16)(myCfgData.frequency)
             , (u16)(actFrequency >> 16)
             , (u16)(actFrequency)
             , myCfgData.clockPhase
             , myCfgData.clockPolarity
             , myCfgData.deviceId
           );

    return ERR_NONE;
}

s8 spiDeinitialize()
{
    const struct spiHw *hw = spiController + myCfgData.instance;
    u8 tmp;

    /* clear receive buffer */
    if (spiReadStat() & 1)
    {
        tmp = spiReadBuf();
    }
    /* clear RX FIFO when SRXMPT bit is set (only valid in enhancled buffer mode) */
    while (!(spiReadStat() & (1 << 5)))
    {
        tmp = spiReadBuf();
    }
    /* clear receive overflow flag as it might be set */
    spiWriteStat(spiReadStat() & ~((u16)(1 << 6)));
    spiWriteStat(0);
    spiWriteCon1(0);
    spiWriteCon2(0);

    return ERR_NONE;
}

s8 spiTxRx(const u8* txData, u8* rxData, u16 length)
{
    const struct spiHw *hw = spiController + myCfgData.instance;
    u16 i = 0;

    if (length == 0) return 0;

    SPI_LOG("SPI Write:");

    /* Write one data before proceeding, this is possible because we have a FIFO */
    /* This could also be changed to a more efficient scheme always looking
       at water levels. But this only makes sense at very high SPI speeds,
       improvement for future */
    if (txData != NULL)
    {
        SPI_LOG(" %hhx", txData[i]);
        spiWriteBuf(txData[i]);
    }
    else
    {
        SPI_LOG(" %hhx", 0);
        spiWriteBuf(0);
    }
    i++;

    /* Write the next byte, then receive the previous byte */
    while (i < length)
    {
        if (txData != NULL)
        {
            SPI_LOG(" %hhx", txData[i]);
            spiWriteBuf(txData[i]);
        }
        else
        {
            SPI_LOG(" %hhx", 0);
            spiWriteBuf(0);
        }
        i++;

        while (spiReadStat() & 0x20);

        if ((rxData != NULL))
        {
            rxData[i - 2] = spiReadBuf();
        }
        else
        {
            spiReadBuf();
        }
    }

    while (spiReadStat() & 0x20);

    /* Receive the last byte, FIFO should be empty for both directions */
    if ((rxData != NULL))
    {
        rxData[i - 1] = spiReadBuf();
    }
    else
    {
        spiReadBuf();
    }

#if USE_LOGGER
    if (rxData)
    {
        SPI_LOG(" ->");

        for (i = 0; i < length; i++)
        {
            SPI_LOG(" %hhx",rxData[i]);
        }
    }

    SPI_LOG("\n");
#endif

    return ERR_NONE;
}

void spiActivateSEN()
{
    if (applSpiActivateSEN)
    {
        applSpiActivateSEN(myCfgData.deviceId);
    }
}

void spiDeactivateSEN()
{
    if (applSpiDeactivateSEN)
    {
        applSpiDeactivateSEN(myCfgData.deviceId);
    }
}
