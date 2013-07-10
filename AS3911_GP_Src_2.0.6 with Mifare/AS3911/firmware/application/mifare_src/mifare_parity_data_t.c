/*
******************************************************************************
* INCLUDES
******************************************************************************
*/
#include "platform.h"
#include "mifare_parity_data_t.h"

/*
******************************************************************************
* GLOBAL FUNCTIONS
******************************************************************************
*/
void calculateParity(parity_data_t *data, u32 length)
{
    u32 index;

    for (index = 0; index < length; index++)
    {
        if (ODD_PARITY(data[index]))
        {
            data[index] |= (parity_data_t) 0x0100;
        }
        else
        {
            data[index] &= ~((parity_data_t) 0x0100);
        }
    }
}

