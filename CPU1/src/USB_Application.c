/*
 * USB_Application.c
 *
 *  Created on: 2020. 10. 27.
 *      Author: Administrator
 */

#include "inc/Global_def.h"

//******************************************************************************
//
// Globals for USB
//
//******************************************************************************
volatile uint32_t g_ui32TxCount = 0;
volatile uint32_t g_ui32RxCount = 0;

tUSBMode g_eCurrentUSBMode;   // The current USB operating mode - Host, Device
                              // or unknown.
volatile uint32_t g_ui32Flags = 0;
char *g_pcStatus;
static volatile bool g_bUSBConfigured = false; // Global flag indicating that a
                                               // USB configuration has been
                                               // set.
//
// EchoNewDataToHost - Receive new data and echo it back to the host.
//
// \param psDevice points to the instance data for the device whose data is to
// be processed.
// \param pi8Data points to the newly received data in the USB receive buffer.
// \param ui32NumBytes is the number of bytes of data available to be processed.
//
// This function is called whenever we receive a notification that data is
// available from the host. We read the data, byte-by-byte and swap the case
// of any alphabetical characters found then write it back out to be
// transmitted back to the host.
//
// \return Returns the number of bytes of data processed.
//
//******************************************************************************
static uint32_t
EchoNewDataToHost(tUSBDBulkDevice *psDevice, uint8_t *pi8Data,
                  uint_fast32_t ui32NumBytes)
{
    uint_fast32_t ui32Loop, ui32Space, ui32Count;
    uint_fast32_t ui32ReadIndex;
    uint_fast32_t ui32WriteIndex;
    tUSBRingBufObject sTxRing;

    //
    // Get the current buffer information to allow us to write directly to
    // the transmit buffer (we already have enough information from the
    // parameters to access the receive buffer directly).
    //
    USBBufferInfoGet(&g_sTxBuffer, &sTxRing);

    //
    // How much space is there in the transmit buffer?
    //
    ui32Space = USBBufferSpaceAvailable(&g_sTxBuffer);

    //
    // How many characters can we process this time round?
    //
    ui32Loop = (ui32Space < ui32NumBytes) ? ui32Space : ui32NumBytes;
    ui32Count = ui32Loop;

    //
    // Update our receive counter.
    //
    g_ui32RxCount += ui32NumBytes;

    //
    // Dump a debug message.
    //
//    DEBUG_PRINT("Received %d bytes\n", ui32NumBytes);
    //
    // Set up to process the characters by directly accessing the USB buffers.
    //
    ui32ReadIndex = (uint32_t)(pi8Data - g_pui8USBRxBuffer);
    ui32WriteIndex = sTxRing.ui32WriteIndex;

    while(ui32Loop)
    {
        //
        // Copy from the receive buffer to the transmit buffer converting
        // character case on the way.
        //

        //
        // Is this a lower case character?
        //
        if((g_pui8USBRxBuffer[ui32ReadIndex] >= 'a') &&
           (g_pui8USBRxBuffer[ui32ReadIndex] <= 'z'))
        {
            //
            // Convert to upper case and write to the transmit buffer.
            //
            g_pui8USBTxBuffer[ui32WriteIndex] =
                (g_pui8USBRxBuffer[ui32ReadIndex] - 'a') + 'A';
        }
        else
        {
            //
            // Is this an upper case character?
            //
            if((g_pui8USBRxBuffer[ui32ReadIndex] >= 'A') &&
               (g_pui8USBRxBuffer[ui32ReadIndex] <= 'Z'))
            {
                //
                // Convert to lower case and write to the transmit buffer.
                //
                g_pui8USBTxBuffer[ui32WriteIndex] =
                    (g_pui8USBRxBuffer[ui32ReadIndex] - 'Z') + 'z';
            }
            else
            {
                //
                // Copy the received character to the transmit buffer.
                //
                g_pui8USBTxBuffer[ui32WriteIndex] =
                    g_pui8USBRxBuffer[ui32ReadIndex];
            }
        }

        //
        // Move to the next character taking care to adjust the pointer for
        // the buffer wrap if necessary.
        //
        ui32WriteIndex++;
        ui32WriteIndex =
            (ui32WriteIndex == BULK_BUFFER_SIZE) ? 0 : ui32WriteIndex;

        ui32ReadIndex++;
        ui32ReadIndex = (ui32ReadIndex == BULK_BUFFER_SIZE) ? 0 : ui32ReadIndex;

        ui32Loop--;
    }

    //
    // We've processed the data in place so now send the processed data
    // back to the host.
    //
    USBBufferDataWritten(&g_sTxBuffer, ui32Count);

//    DEBUG_PRINT("Wrote %d bytes\n", ui32Count);

    //
    // We processed as much data as we can directly from the receive buffer so
    // we need to return the number of bytes to allow the lower layer to
    // update its read pointer appropriately.
    //
    return(ui32Count);
}
//******************************************************************************
//
// TxHandler - Handles bulk driver notifications related to the transmit
//             channel (data to the USB host).
//
// \param pvCBData is the client-supplied callback pointer for this channel.
// \param ui32Event identifies the event we are being notified about.
// \param ui32MsgValue is an event-specific value.
// \param pvMsgData is an event-specific pointer.
//
// This function is called by the bulk driver to notify us of any events
// related to operation of the transmit data channel (the IN channel carrying
// data to the USB host).
//
// \return The return value is event-specific.
//
//******************************************************************************
uint32_t
TxHandler(void *pvCBData, uint32_t ui32Event, uint32_t ui32MsgValue,
          void *pvMsgData)
{
    //
    // We are not required to do anything in response to any transmit event
    // in this example. All we do is update our transmit counter.
    //
    if(ui32Event == USB_EVENT_TX_COMPLETE)
    {
        g_ui32TxCount += ui32MsgValue;
    }

    //
    // Dump a debug message.
    //
    DEBUG_PRINT("TX complete %d\n", ui32MsgValue);

    return(0);
}

//******************************************************************************
//
// Handles bulk driver notifications related to the receive channel (data from
// the USB host).
//
// \param pvCBData is the client-supplied callback pointer for this channel.
// \param ui32Event identifies the event we are being notified about.
// \param ui32MsgValue is an event-specific value.
// \param pvMsgData is an event-specific pointer.
//
// This function is called by the bulk driver to notify us of any events
// related to operation of the receive data channel (the OUT channel carrying
// data from the USB host).
//
// \return The return value is event-specific.
//
//******************************************************************************
uint32_t
RxHandler(void *pvCBData, uint32_t ui32Event, uint32_t ui32MsgValue,
          void *pvMsgData)
{
    //
    // Which event are we being sent?
    //
    switch(ui32Event)
    {
        //
        // We are connected to a host and communication is now possible.
        //
        case USB_EVENT_CONNECTED:
        {
            g_bUSBConfigured = true;
            g_pcStatus = "Host connected.";
            g_ui32Flags |= COMMAND_STATUS_UPDATE;

            //
            // Flush our buffers.
            //
            USBBufferFlush(&g_sTxBuffer);
            USBBufferFlush(&g_sRxBuffer);

            break;
        }

        //
        // The host has disconnected.
        //
        case USB_EVENT_DISCONNECTED:
        {
            g_bUSBConfigured = false;
            g_pcStatus = "Host disconn.";
            g_ui32Flags |= COMMAND_STATUS_UPDATE;
            break;
        }

        //
        // A new packet has been received.
        //
        case USB_EVENT_RX_AVAILABLE:
        {
            tUSBDBulkDevice *psDevice;

            //
            // Get a pointer to our instance data from the callback data
            // parameter.
            //
            psDevice = (tUSBDBulkDevice *)pvCBData;

            //
            // Read the new packet and echo it back to the host.
            //
            return(EchoNewDataToHost(psDevice, pvMsgData, ui32MsgValue));
        }

        //
        // Ignore SUSPEND and RESUME for now.
        //
        case USB_EVENT_SUSPEND:
        case USB_EVENT_RESUME:
            break;

        //
        // Ignore all other events and return 0.
        //
        default:
            break;
    }

    return(0);
}

//******************************************************************************
// ModeCallback - USB Mode callback
//
// \param ui32Index is the zero-based index of the USB controller making the
//        callback.
// \param eMode indicates the new operating mode.
//
// This function is called by the USB library whenever an OTG mode change
// occurs and, if a connection has been made, informs us of whether we are to
// operate as a host or device.
//
// \return None.
//
//******************************************************************************
void
ModeCallback(uint32_t ui32Index, tUSBMode eMode)
{
    //
    // Save the new mode.
    //
    g_eCurrentUSBMode = eMode;
}
