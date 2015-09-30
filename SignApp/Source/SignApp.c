#include "OSAL.h"
#include "AF.h"
#include "ZDApp.h"
#include "ZDObject.h"
#include "ZDProfile.h"

#include "SignApp.h"
#include "DebugTrace.h"

#if !defined( WIN32 ) || defined( ZBIT )
  #include "OnBoard.h"
#endif

/* HAL */
#include "hal_lcd.h"
#include "hal_led.h"
#include "hal_key.h"
#include "hal_uart.h"
#include "hal_adc.h"


const cId_t SignApp_ClusterList[SIGNAPP_MAX_CLUSTERS] =
{
  SIGNAPP_CLUSTERID
};

const SimpleDescriptionFormat_t SignApp_SimpleDesc =
{
  SIGNAPP_ENDPOINT,              //  int Endpoint;
  SIGNAPP_PROFID,                //  uint16 AppProfId[2];
  SIGNAPP_DEVICEID,              //  uint16 AppDeviceId[2];
  SIGNAPP_DEVICE_VERSION,        //  int   AppDevVer:4;
  SIGNAPP_FLAGS,                 //  int   AppFlags:4;
  SIGNAPP_MAX_CLUSTERS,          //  byte  AppNumInClusters;
  (cId_t *)SignApp_ClusterList,  //  byte *pAppInClusterList;
  SIGNAPP_MAX_CLUSTERS,          //  byte  AppNumInClusters;
  (cId_t *)SignApp_ClusterList   //  byte *pAppInClusterList;
};

// This is the Endpoint/Interface description.  It is defined here, but
// filled-in in SignApp_Init().  Another way to go would be to fill
// in the structure here and make it a "const" (in code space).  The
// way it's defined in this sample app it is define in RAM.
endPointDesc_t SignApp_epDesc;

/*********************************************************************
 * LOCAL VARIABLES
 */
//Is there a ship pressent
unsigned char shipPressent;
bool lastMessageSend = true;

byte SignApp_TaskID;   // Task ID for internal task/event processing
                       // This variable will be received when
                      // SignApp_Init() is called.

devStates_t SignApp_NwkState;

byte SignApp_TransID;  // This is the unique message ID (counter)

afAddrType_t SignApp_DstAddr;

// Number of recieved messages
static uint16 rxMsgCount;

// Number of send messages
static uint16 txMsgCount = 0;

// Time interval between sending messages
static uint32 CheckShipStatusChangeInterval = SIGNAPP_CHECK_SHIP_INTERVAL;



/*********************************************************************
 * LOCAL FUNCTIONS
 */

//we don't use binding
//static void SignApp_ProcessZDOMsgs( zdoIncomingMsg_t *inMsg );
static void SignApp_HandleKeys( byte shift, byte keys );
static void SignApp_MessageMSGCB( afIncomingMSGPacket_t *pckt );
static void SignApp_SendStatusChange( void );
static bool SignApp_CheckShipStatusChange( void );


#if defined( IAR_ARMCM3_LM )
static void SignApp_ProcessRtosMessage( void );
#endif

/*********************************************************************
 * PUBLIC FUNCTIONS
 */

/*********************************************************************
 * @fn      SignApp_Init
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
void SignApp_Init( uint8 task_id )
{
  SignApp_TaskID = task_id;
  SignApp_NwkState = DEV_INIT;
  SignApp_TransID = 0;

  // Device hardware initialization can be added here or in main() (Zmain.c).
  // If the hardware is application specific - add it here.
  // If the hardware is other parts of the device add it in mafin().

  //SignApp_DstAddr.addrMode = (afAddrMode_t)AddrNotPresent;
  SignApp_DstAddr.addrMode = (afAddrMode_t)Addr16Bit;
  SignApp_DstAddr.endPoint = 10;
  SignApp_DstAddr.addr.shortAddr = 0x0000;
  //SignApp_DstAddr.addr.shortAddr = 0;

  // Fill out the endpoint description.
  SignApp_epDesc.endPoint = SIGNAPP_ENDPOINT;
  SignApp_epDesc.task_id = &SignApp_TaskID;
  SignApp_epDesc.simpleDesc
            = (SimpleDescriptionFormat_t *)&SignApp_SimpleDesc;
  SignApp_epDesc.latencyReq = noLatencyReqs;

  // Register the endpoint description with the AF
  afRegister( &SignApp_epDesc );

  // Register for all key events - This app will handle all key events
  RegisterForKeys( SignApp_TaskID );

  // Update the display
#if defined ( LCD_SUPPORTED )
  HalLcdWriteString( "SignApp", HAL_LCD_LINE_1 );
#endif

//we don't use binding
//  ZDO_RegisterForZDOMsg( SignApp_TaskID, End_Device_Bind_rsp );
//  ZDO_RegisterForZDOMsg( SignApp_TaskID, Match_Desc_rsp );

#if defined( IAR_ARMCM3_LM )
  // Register this task with RTOS task initiator
  RTOS_RegisterApp( task_id, SIGNAPP_RTOS_MSG_EVT );
#endif
}

/*********************************************************************
 * @fn      SignApp_ProcessEvent
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
uint16 SignApp_ProcessEvent( uint8 task_id, uint16 events )
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
    MSGpkt = (afIncomingMSGPacket_t *)osal_msg_receive( SignApp_TaskID );
    while ( MSGpkt )
    {
      switch ( MSGpkt->hdr.event )
      {
// We don't use binding
//        case ZDO_CB_MSG:
//          SignApp_ProcessZDOMsgs( (zdoIncomingMsg_t *)MSGpkt );
//          break;

        case KEY_CHANGE:
          SignApp_HandleKeys( ((keyChange_t *)MSGpkt)->state, ((keyChange_t *)MSGpkt)->keys );
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
          SignApp_MessageMSGCB( MSGpkt );
          break;

        case ZDO_STATE_CHANGE:
          SignApp_NwkState = (devStates_t)(MSGpkt->hdr.status);
          if (SignApp_NwkState == DEV_END_DEVICE)
          {
            // Start cheking if a ship is pressent at a regular interval.
            osal_start_timerEx( SignApp_TaskID,
                                SIGNAPP_CHECK_SHIP_EVT,
                                CheckShipStatusChangeInterval );
          }
          break;

        default:
          break;
      }

      // Release the memory
      osal_msg_deallocate( (uint8 *)MSGpkt );

      // Next
      MSGpkt = (afIncomingMSGPacket_t *)osal_msg_receive( SignApp_TaskID );
    }

    // return unprocessed events
    return (events ^ SYS_EVENT_MSG);
  }

  // Check if there is a ship and send status if chere was any change - This event is generated by a timer
  //  (setup in SignApp_Init()).
  if ( events & SIGNAPP_CHECK_SHIP_EVT )
  {
    // Send "the" message
    SignApp_SendStatusChange();

    // Setup to check if ship is pressent
    osal_start_timerEx( SignApp_TaskID,
                        SIGNAPP_CHECK_SHIP_EVT,
                        CheckShipStatusChangeInterval );

    // return unprocessed events
    return (events ^ SIGNAPP_CHECK_SHIP_EVT);
  }

#if defined( IAR_ARMCM3_LM )
  // Receive a message from the RTOS queue
  if ( events & SIGNAPP_RTOS_MSG_EVT )
  {
    // Process message from RTOS queue
    SignApp_ProcessRtosMessage();

    // return unprocessed events
    return (events ^ SIGNAPP_RTOS_MSG_EVT);
  }
#endif

  // Discard unknown events
  return 0;
}



/*********************************************************************
 * @fn      SignApp_HandleKeys
 *
 * @brief   Handles all key events for this device.
 *
 * @param   shift - true if in shift/alt.
 * @param   keys - bit field for key events. Valid entries:
 *                 HAL_KEY_SW_4
 *                 HAL_KEY_SW_3
 *                 HAL_KEY_SW_2
 *                 HAL_KEY_SW_1
 *
 * @return  none
 */
static void SignApp_HandleKeys( uint8 shift, uint8 keys )
{
  zAddrType_t dstAddr;

  // Shift is used to make each button/switch dual purpose.
  if ( shift )
  {
    if ( keys & HAL_KEY_SW_1 )
    {
    }
    if ( keys & HAL_KEY_SW_2 )
    {
    }
    if ( keys & HAL_KEY_SW_3 )
    {
    }
    if ( keys & HAL_KEY_SW_4 )
    {
    }
  }
  else
  {
    if ( keys & HAL_KEY_SW_1 )
    {
#if defined( SWITCH1_BIND )
      // We can use SW1 to simulate SW2 for devices that only have one switch,
      keys |= HAL_KEY_SW_2;
#elif defined( SWITCH1_MATCH )
      // or use SW1 to simulate SW4 for devices that only have one switch
      keys |= HAL_KEY_SW_4;
#else
      // Normally, SW1 changes the rate that messages are sent
      if ( CheckShipStatusChangeInterval > 100 )
      {
        // Cut the message TX delay in half
        CheckShipStatusChangeInterval /= 2;
      }
      else
      {
        // Reset to the default
        CheckShipStatusChangeInterval = SIGNAPP_CHECK_SHIP_INTERVAL;
      }
#endif
    }

    if ( keys & HAL_KEY_SW_2 )
    {
      HalLedSet ( HAL_LED_4, HAL_LED_MODE_OFF );

      // Initiate an End Device Bind Request for the mandatory endpoint
      dstAddr.addrMode = Addr16Bit;
      dstAddr.addr.shortAddr = 0x0000; // Coordinator
      ZDP_EndDeviceBindReq( &dstAddr, NLME_GetShortAddr(),
                            SignApp_epDesc.endPoint,
                            SIGNAPP_PROFID,
                            SIGNAPP_MAX_CLUSTERS, (cId_t *)SignApp_ClusterList,
                            SIGNAPP_MAX_CLUSTERS, (cId_t *)SignApp_ClusterList,
                            FALSE );
    }

    if ( keys & HAL_KEY_SW_3 )
    {
    }

    if ( keys & HAL_KEY_SW_4 )
    {
      HalLedSet ( HAL_LED_4, HAL_LED_MODE_OFF );
      // Initiate a Match Description Request (Service Discovery)
      dstAddr.addrMode = AddrBroadcast;
      dstAddr.addr.shortAddr = NWK_BROADCAST_SHORTADDR;
      ZDP_MatchDescReq( &dstAddr, NWK_BROADCAST_SHORTADDR,
                        SIGNAPP_PROFID,
                        SIGNAPP_MAX_CLUSTERS, (cId_t *)SignApp_ClusterList,
                        SIGNAPP_MAX_CLUSTERS, (cId_t *)SignApp_ClusterList,
                        FALSE );
    }
  }
}

/*********************************************************************
 * LOCAL FUNCTIONS
 */

/*********************************************************************
 * @fn      SignApp_MessageMSGCB
 *
 * @brief   Data message processor callback.  This function processes
 *          any incoming data - probably from other devices.  So, based
 *          on cluster ID, perform the intended action.
 *
 * @param   none
 *
 * @return  none
 */
static void SignApp_MessageMSGCB( afIncomingMSGPacket_t *pkt )
{
  switch ( pkt->clusterId )
  {
    case SIGNAPP_CLUSTERID:
      rxMsgCount += 1;  // Count this message
      //HalLedSet ( HAL_LED_4, HAL_LED_MODE_BLINK );  // Blink an LED
      //if incoming state is 1 (indicate green sign)
      if (*pkt->cmd.Data == 1){
        HalLedSet( HAL_LED_4, HAL_LED_MODE_ON );
      }
      //if incoming state is 0 (indicate red sign)
      else {
        HalLedSet( HAL_LED_4, HAL_LED_MODE_OFF );
      }
#if defined( LCD_SUPPORTED )
      HalLcdWriteStringValue("incomming", *(pkt->cmd.Data), 10, HAL_LCD_LINE_1 );
      HalLcdWriteStringValue( "Rcvd:", rxMsgCount, 10, HAL_LCD_LINE_2 );
#elif defined( WIN32 )
      WPRINTSTR( pkt->cmd.Data );
#endif
      break;
  }
}

/*********************************************************************
 * @fn      SignApp_SendStatusChange
 *
 * @brief   Send status if a ship has arrived or left
 *
 * @param   none
 *
 * @return  none
 */
static void SignApp_SendStatusChange( void )
{
  char theMessageData = 'a';
  if ( !lastMessageSend || SignApp_CheckShipStatusChange()){
      HalLcdWriteStringValue("sizeofShipPres", sizeof(shipPressent), 10, HAL_LCD_LINE_3 );


    if ( AF_DataRequest( &SignApp_DstAddr, &SignApp_epDesc,
                         SIGNAPP_CLUSTERID,
                         1,
                         &shipPressent,
                         &SignApp_TransID,
                         AF_DISCV_ROUTE, AF_DEFAULT_RADIUS ) == afStatus_SUCCESS )
    {
      /*AF_DataRequest( &SignApp_DstAddr, &SignApp_epDesc,
                         SIGNAPP_CLUSTERID,
                         1,
                         (byte *)&theMessageData,
                         &SignApp_TransID,
                         AF_DISCV_ROUTE, AF_DEFAULT_RADIUS );*/
      // Successfully requested to be sent.
      HalLcdWriteStringValue("msgSndCount", ++txMsgCount, 10, HAL_LCD_LINE_1 );
      HalLcdWriteStringValue("shipPressent", shipPressent, 10, HAL_LCD_LINE_2 );
      lastMessageSend = true;
    }
    else
    {
      // Error occurred in request to send.
      HalLcdWriteString( "error sendMessage", HAL_LCD_LINE_1 );
      lastMessageSend = false;
    }
  }
}


/*********************************************************************
 * @fn      SignApp_CheckShipStatusChange
 *
 * @brief   Check if there is a ship pressent
 *
 * @param   none
 *
 * @return  bool. True if there is a change sinse last check
 */
static bool SignApp_CheckShipStatusChange( void )
{
   unsigned char lastShipPressent = shipPressent;
  uint16 distance = HalAdcRead(HAL_ADC_CHANNEL_7, HAL_ADC_RESOLUTION_8);
  HalLcdWriteStringValue( "ADC", distance, 10, HAL_LCD_LINE_3 );
  if (distance < 64) {
    shipPressent = 0;
  }
  else {
    shipPressent = 1;
  }
  if (lastShipPressent == shipPressent){
    return false;
  }
  else{
    return true;
  }
}
