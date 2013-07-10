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
 *  \brief Application dispatcher
 *
 */
/*!
 * 
 * Moudle used to dispatch the commands received by the PC application.
 */

#ifndef DISPATCHER_H
#define DISPATCHER_H

/*
******************************************************************************
* INCLUDES
******************************************************************************
*/
#include "platform.h"

/*
******************************************************************************
* GLOBAL FUNCTION PROTOTYPES
******************************************************************************
*/
/*! 
 *****************************************************************************
 *  \brief  Initialize the dispatcher.
 *
 *  \return ERR_NONE : No error, dispatcher initialized.
 *
 *****************************************************************************
 */
extern s8 dispatcherInitialize(void);

/*! 
 *****************************************************************************
 *  \brief  Deinitialize the dispatcher.
 *
 *  \return ERR_NONE : No error.
 *
 *****************************************************************************
 */
extern s8 dispatcherDeinitialize(void);

/*! 
 *****************************************************************************
 *  \brief  Dispatch the given command
 *
 *  The dispatcher takes the command \a cmd and calls the appropriate
 *  function. Required parameters needed for the particular functions
 *  has to be placed into \a buf.
 *
 *  \param[in] cmd : Command to be executed.
 *  \param[in] buf : Additional data which might be required by the command.
 *  \param[in] length : length of \a buf
 *
 *  \return ERR_NONE : No error.
 *
 *****************************************************************************
 */
extern s8 dispatcher(u8 cmd, u8* buf, u16 length);

/*! 
 *****************************************************************************
 *  \brief  Handle interrupt requests which are targetted for the dispatcher
 *
 * The GUI may enable interrupts and read out proper results at the time of the
 * interrupt. 
 * This function handles these interrupts and stores the results for a later 
 * time when the GUI polls it.
 */
extern void dispatcherInterruptHandler(void);
#endif /* DISPATCHER_H */

