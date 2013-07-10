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
 *  \author Oliver Regenfelder
 *
 *  \brief Implementation of basic commands for the NFC-P2P stack implemented
 *  in the GUI.
 *
 * NFC-P2P defines two roles: the initiator role and the target role. One device of a P2P
 * communication must act as an initiator and the other device must act as a target. This
 * module implements just the time critical parts of the NFC-P2P protocol. These parts are
 * then used by the GUI to implement the NFC-IP1 stack in active target and active initiator
 * role. The following section describes how the functions provided by this module must
 * be used to implement the initiator and target role respectively.
 *
 * \section InitiatorRole Initiator Role
 *
 * At first nfcInitialize() must be called with appropriate paramters to configure the nfc
 * module for the active initiator role and the desired bitrate. This bitrate is used for
 * the ATR_REQ (attribute request) command and any subsequent communication.
 *
 * After the module has been initialized an ATR_REQ can be send at any time using the
 * function nfcTxNBytes() with \a perform_collision_avoidance set to \c TRUE.
 * As the reader field is not activate on the first call to nfcTxNbytes() an initial response
 * collision is performed before the ATR_REQ command is send. Afterwards nfcRxNBytes() can be used
 * to observe the status of the peers answer. This function is non blocking and will return
 * immediately with \c ERR_BUSY to notify the caller that a reception is still in progress,
 * or it will store the received message in the given buffer and report the error status of the
 * received message. If a response has indeed been received from the peer device, then a
 * response collision avoidance will already have been performed before the received message
 * is returned.
 *
 * From there on communcition with the peer device continues with calls to nfcTxNBytes() with
 * \a perform_collision_avoidance set to \c TRUE to send a message and calls to nfcRxNBytes() to
 * receive the response.
 *
 * On the last message transmission the parameter \a perform_collision_avoidance of nfcTxNBytes()
 * has to be set to \a FALSE. So that no response collision avoidance is performed after the
 * peer device acknowledges its deselction/deactivation.
 *
 * If another peer device activation using the initiator role is required, then a new communication
 * can start directly with a call to nfcTxNBytes() - the call to nfcInitialize() can be skipped.
 *
 * \section TargetRole Target Role
 *
 * At first nfcInitialize() must be called with appropriate parameters to configure the nfc
 * module for the active target role.
 * Then nfcStartInitialTargetRx() must be called. This function initializes the low power target
 * bitrate detection mode of the AS3911 and enables reception of an initiator message. Afterwards
 * calls to the non blocking nfcRxNBytes() can
 * be used to check whether a message from an initiator has been received or not. If nfcRxNBytes()
 * returns \c ERR_BUSY, then no message has been received yet. If it returns any other error status, then
 * a message has been received and the content of that message is stored in the buffers passed to nfcRxNBytes().
 * From there on communication is performed via nfcTxNBytes() and nfcRxNBytes() as in the
 * initiator role. Once communication with the peer device is completed a call to nfcStartInitialTargetRx()
 * is required to reenter the low power target bitrate detection mode and wait for the next activation
 * from an initiator.
 *
 * Also, if the first message received is not an ATR_REQ, then nfcStartInitialTargetRx() must
 * be called to go back to the low power target bitrate detection mode and wait for the next potential
 * initiator message.
 */
/*!
 * 
 */

#ifndef NFC_H
#define NFC_H

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
 *  \brief Initialize the NFC mode.
 *  \note This function needs to be called every time after switching
 *  from a different mode.
 *  \note Passive initator and passive target role (is_active = FALSE) are
 *  currently not supported.
 *
 *  \param[in] is_active : whether it should be active or passive mode
 *  \param[in] is_initiator : whether we want to be initiator or target
 *  \param[in] bitrate : target bitrate: 2^bitrate * 106kBit/s
 * 
 *  \return ERR_IO : Error during communication.
 *  \return ERR_PARAM : Selected bitrate or \a is_active / \a is_initiator combination
 *  not supported.
 *  \return ERR_NONE : No error.
 *
 *****************************************************************************
 */
extern s8 nfcInitialize(u8 is_active, u8 is_initiator, u8 bitrate);

/*!
 *****************************************************************************
 *  \brief Deinitialize the NFC mode.
 *  \note This function should be called every time NFC is not needed
 *  any more.
 *  \return ERR_IO : Error during communication.
 *  \return ERR_NONE : No error.
 *
 *****************************************************************************
 */
extern s8 nfcDeinitialize(void);

/*!
 *****************************************************************************
 *  \brief  Transfer a given number of bytes via NFC.
 *
 *  Send \a bufSize bytes from \a buf to a P2P partner device. If the RF field
 *  of the AS3911 is off, then an initial RF collision avoidance will be performed
 *  prior to the data transmission.
 *
 *  \param[in] buf : Buffer to be transmitted.
 *  \param[in] bufSize : Number of bytes to be transmitted (= length of \a buf).
 *  \param[in] perform_collision_avoidance : If set to true, then a response
 *  RF collision avoidance with n=0 will be performed after the P2P partner
 *  device has send its response.
 *
 *  \return ERR_RF_COLLISION: Initial RF collision avoidance failed, due to
 *  presence of another RF field.
 *  \return ERR_INTERNAL : Timeout while waiting for CAT or CAC interrupt.
 *  \return ERR_IO : Error during communication.
 *  \return ERR_NONE : No error.
 *
 *****************************************************************************
 */
extern s8 nfcTxNBytes(const u8 *buf, u8 bufSize, u8 perform_collision_avoidance);

/*!
 *****************************************************************************
 *  \brief Check NFC data reception status.
 *
 *  Check the status of an ongoing NFC data reception. If the data reception is
 *  completed then the received data will be stored in \a buf.
 *
 *  \param[out] buf : Pointer to memory area where received data will be stored.
 *  \param[in] bufsize : Max. number of bytes to receive (= length of \a buf).
 *  \param[out] actlen : Actual number of bytes received.
 *
 *  \return ERR_TIMEOUT : The P2P partner device has not performed its response
 *  RF collision avoidance within the required timeframe, or the response
 *  RF collision avoidance has been performed but not data has been send.
 *  \return ERR_RF_COLLISION: Response RF collision avoidance after the
 *    reception failed. \a buf may still contain valid data.
 *  \return ERR_INTERNAL : Timeout while waiting for the CAT, CAC, or OSC interrupt.
 *  \return ERR_BUSY : Reception in progress.
 *  \return ERR_IO : Error during communication.
 *  \return ERR_NONE : No error.
 *
 *****************************************************************************
 */
extern s8 nfcRxNBytes(u8 *buf, u8 bufsize, u8 *actlen);

/*!
 *****************************************************************************
 *  \brief Switch to NFC data reception.
 *
 *  Switches from NFC data transmission to NFC data reception without
 *  transmitting any data. This makes it possible to simulate a timeout on
 *  the NFC DEP connection.
 *
 *  \param[in] perform_collision_avoidance : If set to true, then a response
 *  RF collision avoidance with n=0 will be performed after the P2P partner
 *  device has send its response.
 *
 *  \return ERR_IO: Error during communication, or switch not possible because
 *  the NFC module is not waiting for data to be transmitted.
 *  \return ERR_NONE : No error.
 *****************************************************************************
 */
s8 nfcSwitchToRx(u8 perform_collision_avoidance);

/*!
 *****************************************************************************
 *  \brief Start initial target role reception.
 *
 *  Put the AS3911 into low power target mode, and start waiting for a command
 *  from an initiator device.
 *
 *  \return ERR_BUSY: The nfc module is busy and cannot enter initial target role.
 *  \return ERR_IO : Error during communication.
 *  \return ERR_NONE : No error.
 *
 *****************************************************************************
 */
extern s8 nfcStartInitialTargetRx();

/*!
 *****************************************************************************
 *  \brief  Process AS3911 interrupts while in NFC mode.
 *
 *  This function must be called regularly to allow the nfc module to process
 *  interrupts from the AS3911. If no reception is in progress, the function
 *  returns immediately, otherwise, pending reception tasks are performed before
 *  the function returns.
 *
 *  \return ERR_IO : Error during communication.
 *  \return ERR_INTERNAL : Timeout while waiting for the OSC, CAC, or CAT interrupt.
 *  \return ERR_NONE : No error.
 *
 *****************************************************************************
 */
extern s8 nfcReceptionTask();

#endif /* NFC_H */
