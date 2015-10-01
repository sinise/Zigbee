/******************************************************************************
  Filename:       CoordinatorApp.c
  Revised:        $Date: 2014-09-07 13:36:30 -0700 (Sun, 07 Sep 2014) $
  Revision:       $Revision: 40046 $


******************************************************************************/

/*********************************************************************
 * INCLUDES
 */
#include "OSAL.h"
#include "AF.h"
#include "ZDApp.h"
#include "ZDObject.h"
#include "ZDProfile.h"

#include "CoordinatorApp.h"
#include "DebugTrace.h"

#if !defined( WIN32 ) || defined( ZBIT )
  #include "OnBoard.h"
#endif

/* HAL */
#include "hal_lcd.h"
#include "hal_led.h"
#include "hal_key.h"
#include "hal_uart.h"

/* RTOS */
#if defined( IAR_ARMCM3_LM )
#include "RTOS_App.h"
#endif

// This list should be filled with Application specific Cluster IDs.
const cId_t CoordinatorApp_ClusterList[COORDINATORAPP_MAX_CLUSTERS] =
{
  COORDINATORAPP_CLUSTERID
};

const SimpleDescriptionFormat_t CoordinatorApp_SimpleDesc =
{
  COORDINATORAPP_ENDPOINT,              //  int Endpoint;
  COORDINATORAPP_PROFID,                //  uint16 AppProfId[2];
  COORDINATORAPP_DEVICEID,              //  uint16 AppDeviceId[2];
  COORDINATORAPP_DEVICE_VERSION,        //  int   AppDevVer:4;
  COORDINATORAPP_FLAGS,                 //  int   AppFlags:4;
  COORDINATORAPP_MAX_CLUSTERS,          //  byte  AppNumInClusters;
  (cId_t *)CoordinatorApp_ClusterList,  //  byte *pAppInClusterList;
  COORDINATORAPP_MAX_CLUSTERS,          //  byte  AppNumInClusters;
  (cId_t *)CoordinatorApp_ClusterList   //  byte *pAppInClusterList;
};

// This is the Endpoint/Interface description.  It is defined here, but
// filled-in in CoordinatorApp_Init().  Another way to go would be to fill
// in the structure here and make it a "const" (in code space).  The
// way it's defined in this sample app it is define in RAM.
endPointDesc_t CoordinatorApp_epDesc;

/*********************************************************************
 * EXTERNAL VARIABLES
 */

/*********************************************************************
 * EXTERNAL FUNCTIONS
 */

/*********************************************************************
 * LOCAL VARIABLES
 */
byte CoordinatorApp_TaskID;   // Task ID for internal task/event processing
                          // This variable will be received when
                          // CoordinatorApp_Init() is called.

devStates_t CoordinatorApp_NwkState;

byte CoordinatorApp_TransID;  // This is the unique message ID (counter)

afAddrType_t CoordinatorApp_DstAddr;

// Number of recieved messages
static uint16 rxMsgCount;

// Time interval between sending messages
static uint32 txMsgDelay = COORDINATORAPP_SEND_MSG_TIMEOUT;

unsigned char signState;
/*********************************************************************
 * LOCAL FUNCTIONS
 */
static void CoordinatorApp_ProcessZDOMsgs( zdoIncomingMsg_t *inMsg );
static void CoordinatorApp_HandleKeys( byte shift, byte keys );
static void CoordinatorApp_MessageMSGCB( afIncomingMSGPacket_t *pckt );
static void CoordinatorApp_SendTheMessage( void );

#if defined( IAR_ARMCM3_LM )
static void CoordinatorApp_ProcessRtosMessage( void );
#endif

/*********************************************************************
 * NETWORK LAYER CALLBACKS
 */

/*********************************************************************
 * PUBLIC FUNCTIONS
 */

/*********************************************************************
 * @fn      CoordinatorApp_Init
 *
 * @brief   Initialization function for the Generic App Task.
 *          This is called during initialization and should contain
 *          any application specific initialization (ie. hardware
 *          initialization/setup, table initialization, power up
 *          notificaiton ... ).
 *
 * @param   task_id - the ID assigned by OSAL.  This ID should be
 *                    used to send messages and set timers.
 *
 * @return  none
 */
void CoordinatorApp_Init( uint8 task_id )
{
  CoordinatorApp_TaskID = task_id;
  CoordinatorApp_NwkState = DEV_INIT;
  CoordinatorApp_TransID = 0;

  // Device hardware initialization can be added here or in main() (Zmain.c).
  // If the hardware is application specific - add it here.
  // If the hardware is other parts of the device add it in main().

  CoordinatorApp_DstAddr.addrMode = (afAddrMode_t)Addr16Bit;
  CoordinatorApp_DstAddr.endPoint = 10;
  CoordinatorApp_DstAddr.addr.shortAddr = 0x0000;

  // Fill out the endpoint description.
  CoordinatorApp_epDesc.endPoint = COORDINATORAPP_ENDPOINT;
  CoordinatorApp_epDesc.task_id = &CoordinatorApp_TaskID;
  CoordinatorApp_epDesc.simpleDesc
            = (SimpleDescriptionFormat_t *)&CoordinatorApp_SimpleDesc;
  CoordinatorApp_epDesc.latencyReq = noLatencyReqs;

  // Register the endpoint description with the AF
  afRegister( &CoordinatorApp_epDesc );

  // Register for all key events - This app will handle all key events
  RegisterForKeys( CoordinatorApp_TaskID );

  // Update the display
#if defined ( LCD_SUPPORTED )
  HalLcdWriteString( "CoordinatorApp", HAL_LCD_LINE_1 );
#endif

  ZDO_RegisterForZDOMsg( CoordinatorApp_TaskID, End_Device_Bind_rsp );
  ZDO_RegisterForZDOMsg( CoordinatorApp_TaskID, Match_Desc_rsp );

#if defined( IAR_ARMCM3_LM )
  // Register this task with RTOS task initiator
  RTOS_RegisterApp( task_id, COORDINATORAPP_RTOS_MSG_EVT );
#endif
}

/*********************************************************************
 * @fn      CoordinatorApp_ProcessEvent
 *
 * @brief   Generic Application Task event processor.  This function
 *          is called to process all events for the task.  Events
 *          include timers, messages and any other user defined events.
 *
 * @param   task_id  - The OSAL assigned task ID.
 * @param   events - events to process.  This is a bit map and can
 *                   contain more than one event.
 *
 * @return  none
 */
uint16 CoordinatorApp_ProcessEvent( uint8 task_id, uint16 events )
{
  afIncomingMSGPacket_t *MSGpkt;
  afDataConfirm_t *afDataConfirm;

  // Data Confirmation message fields
  byte sentEP;
  ZStatus_t sentStatus;
  byte sentTransID;       // This should match the value sent
  (void)task_id;  // Intentionally unreferenced parameter

  if ( events & SYS_EVENT_MSG )
  {
    MSGpkt = (afIncomingMSGPacket_t *)osal_msg_receive( CoordinatorApp_TaskID );
    while ( MSGpkt )
    {
      switch ( MSGpkt->hdr.event )
      {
        case ZDO_CB_MSG:
          CoordinatorApp_ProcessZDOMsgs( (zdoIncomingMsg_t *)MSGpkt );
          break;

        case KEY_CHANGE:
          CoordinatorApp_HandleKeys( ((keyChange_t *)MSGpkt)->state, ((keyChange_t *)MSGpkt)->keys );
          break;

        case AF_DATA_CONFIRM_CMD:
          // This message is received as a confirmation of a data packet sent.
          // The status is of ZStatus_t type [defined in ZComDef.h]
          // The message fields are defined in AF.h
          afDataConfirm = (afDataConfirm_t *)MSGpkt;

          sentEP = afDataConfirm->endpoint;
          (void)sentEP;  // This info not used now
          sentTransID = afDataConfirm->transID;
          (void)sentTransID;  // This info not used now

          sentStatus = afDataConfirm->hdr.status;
          // Action taken when confirmation is received.
          if ( sentStatus != ZSuccess )
          {
            // The data wasn't delivered -- Do something
          }
          break;

        case AF_INCOMING_MSG_CMD:
          CoordinatorApp_MessageMSGCB( MSGpkt );
          break;

        case ZDO_STATE_CHANGE:
          CoordinatorApp_NwkState = (devStates_t)(MSGpkt->hdr.status);
          if ( (CoordinatorApp_NwkState == DEV_ZB_COORD) ||
               (CoordinatorApp_NwkState == DEV_ROUTER) ||
               (CoordinatorApp_NwkState == DEV_END_DEVICE) )
          {
            // Start sending "the" message in a regular interval.
            osal_start_timerEx( CoordinatorApp_TaskID,
                                COORDINATORAPP_SEND_MSG_EVT,
                                txMsgDelay );
          }
          break;

        default:
          break;
      }

      // Release the memory
      osal_msg_deallocate( (uint8 *)MSGpkt );

      // Next
      MSGpkt = (afIncomingMSGPacket_t *)osal_msg_receive( CoordinatorApp_TaskID );
    }

    // return unprocessed events
    return (events ^ SYS_EVENT_MSG);
  }

  // Send a message out - This event is generated by a timer
  //  (setup in CoordinatorApp_Init()).
  if ( events & COORDINATORAPP_SEND_MSG_EVT )
  {
    return (events ^ COORDINATORAPP_SEND_MSG_EVT);
  }

#if defined( IAR_ARMCM3_LM )
  // Receive a message from the RTOS queue
  if ( events & COORDINATORAPP_RTOS_MSG_EVT )
  {
    // Process message from RTOS queue
    CoordinatorApp_ProcessRtosMessage();

    // return unprocessed events
    return (events ^ COORDINATORAPP_RTOS_MSG_EVT);
  }
#endif

  // Discard unknown events
  return 0;
}

/*********************************************************************
 * Event Generation Functions
 */

/*********************************************************************
 * @fn      CoordinatorApp_ProcessZDOMsgs()
 *
 * @brief   Process response messages
 *
 * @param   none
 *
 * @return  none
 */
static void CoordinatorApp_ProcessZDOMsgs( zdoIncomingMsg_t *inMsg )
{
  switch ( inMsg->clusterID )
  {
    case End_Device_Bind_rsp:
      if ( ZDO_ParseBindRsp( inMsg ) == ZSuccess )
      {
        // Light LED
        HalLedSet( HAL_LED_4, HAL_LED_MODE_ON );
      }
#if defined( BLINK_LEDS )
      else
      {
        // Flash LED to show failure
        HalLedSet ( HAL_LED_4, HAL_LED_MODE_FLASH );
      }
#endif
      break;

    case Match_Desc_rsp:
      {
        ZDO_ActiveEndpointRsp_t *pRsp = ZDO_ParseEPListRsp( inMsg );
        if ( pRsp )
        {
          if ( pRsp->status == ZSuccess && pRsp->cnt )
          {
            CoordinatorApp_DstAddr.addrMode = (afAddrMode_t)Addr16Bit;
            CoordinatorApp_DstAddr.addr.shortAddr = pRsp->nwkAddr;
            // Take the first endpoint, Can be changed to search through endpoints
            CoordinatorApp_DstAddr.endPoint = pRsp->epList[0];

            // Light LED
            HalLedSet( HAL_LED_4, HAL_LED_MODE_ON );
          }
          osal_mem_free( pRsp );
        }
      }
      break;
  }
}

/*********************************************************************
 * @fn      CoordinatorApp_HandleKeys
 *
 * @brief   Handles all key events for this device.
 *
 * @param   shift - true if in shift/alt.
 * @param   keys - bit field for key events. Valid entries:
 *                 HAL_KEY_SW_4
 *                 HAL_KEY_SW_2
 *
 * @return  none
 */
static void CoordinatorApp_HandleKeys( uint8 shift, uint8 keys )
{
  zAddrType_t dstAddr;

  //send command RED
  if ( keys & HAL_KEY_SW_2 )
  {
    HalLedSet ( HAL_LED_2, HAL_LED_MODE_ON );
    signState = 0;
    HalLcdWriteString( "send red", HAL_LCD_LINE_3 );
    CoordinatorApp_SendTheMessage();
  }

  //send command GREEN
  if ( keys & HAL_KEY_SW_4 )
  {
    HalLedSet ( HAL_LED_2, HAL_LED_MODE_OFF );
    signState = 1;
    HalLcdWriteString( "send Green", HAL_LCD_LINE_3 );
    CoordinatorApp_SendTheMessage();
  }
}

/*********************************************************************
 * LOCAL FUNCTIONS
 */

/*********************************************************************
 * @fn      CoordinatorApp_MessageMSGCB
 *
 * @brief   Data message processor callback.  This function processes
 *          any incoming data - probably from other devices.  So, based
 *          on cluster ID, perform the intended action.
 *
 * @param   none
 *
 * @return  none
 */
static void CoordinatorApp_MessageMSGCB( afIncomingMSGPacket_t *pkt )
{

  switch ( pkt->clusterId )
  {
    case COORDINATORAPP_CLUSTERID:
      //get the address of the sign and put it in to the sending address
      CoordinatorApp_DstAddr.addr.shortAddr = pkt->srcAddr.addr.shortAddr;

      rxMsgCount += 1;  // Count this message
      HalLedSet ( HAL_LED_4, HAL_LED_MODE_BLINK );  // Blink an LED
#if defined( LCD_SUPPORTED )
      HalLcdWriteStringValue("received", *(pkt->cmd.Data), 10 ,HAL_LCD_LINE_1 );
      HalLcdWriteStringValue( "Rcvd:", rxMsgCount, 10, HAL_LCD_LINE_2 );
#elif defined( WIN32 )
      WPRINTSTR( pkt->cmd.Data );
#endif
      break;
  }
}

/*********************************************************************
 * @fn      CoordinatorApp_SendTheMessage
 *
 * @brief   Send "the" message.
 *
 * @param   none
 *
 * @return  none
 */
static void CoordinatorApp_SendTheMessage( void )
{
  if ( AF_DataRequest( &CoordinatorApp_DstAddr, &CoordinatorApp_epDesc,
                       COORDINATORAPP_CLUSTERID,
                       1,
                       &signState,
                       &CoordinatorApp_TransID,
                       AF_DISCV_ROUTE, AF_DEFAULT_RADIUS ) == afStatus_SUCCESS )
  {
    // Successfully requested to be sent.
  }
  else
  {
    // Error occurred in request to send.
  }
}

