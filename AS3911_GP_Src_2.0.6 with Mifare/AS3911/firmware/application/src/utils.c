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
 *      PROJECT:   AS3911
 *      $Revision: $
 *      LANGUAGE:  ANSI C
 */

/*! \file
 *
 *  \author Christian Eisendle
 *
 *  \brief Common and helpful macros and functions
 *
 */

/*
******************************************************************************
* INCLUDES
******************************************************************************
*/
#include "ams_types.h"
#include "utils.h"

/*
******************************************************************************
* GLOBAL FUNCTIONS
******************************************************************************
*/
void AMS_MEMMOVE(void *dest, const void *src, u32 length)
{
    u8* chardest = (u8*)dest;
    const u8* charsrc = (u8*)src;
    u32 i = 0;

    if (chardest < charsrc) while (i<length)
    {
        chardest[i] = charsrc[i];
        i++;
    }else while (length--)
    {
        chardest[length] = charsrc[length];
    }
}

void AMS_MEMCPY(void *dest, const void *src, u32 length)
{
    u8* chardest = (u8*)dest;
    const u8* charsrc = (u8*)src;

    while (length--)
    {
        chardest[length] = charsrc[length];
    }
}

void AMS_BYTESWAP(u8* buffer, u32 length)
{
    u8 tmp;
    u8* b1;
    u8* b2;

    b1 = buffer;
    b2 = buffer + length - 1;

    while (b1 < b2)
    {
        tmp = *b1;
        *b1 = *b2;
        *b2 = tmp;
        b1++;
        b2--;
    }
}

void* AMS_MEMSET(void* ptr, u8 val, u32 num)
{
    u8* it = (u8*)ptr;

    while (num--)
    {
        *it = val;
        it++;
    }

    return ptr;
}

