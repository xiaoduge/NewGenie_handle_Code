#ifndef _ZB_H_
#define _ZB_H_

/* devStates_t imported from TI cc2530 */
typedef enum
{
  DEV_HOLD,               // Initialized - not started automatically
  DEV_INIT,               // Initialized - not connected to anything
  DEV_NWK_DISC,           // Discovering PAN's to join
  DEV_NWK_JOINING,        // Joining a PAN
  DEV_NWK_REJOIN,         // ReJoining a PAN, only for end devices
  DEV_END_DEVICE_UNAUTH,  // Joined but not yet authenticated by trust center
  DEV_END_DEVICE,         // Started as device after authentication
  DEV_ROUTER,             // Device joined, authenticated and is a router
  DEV_COORD_STARTING,     // Started as Zigbee Coordinator
  DEV_ZB_COORD,           // Started as Zigbee Coordinator
  DEV_NWK_ORPHAN          // Device has lost information about its parent..
} devStates_t;

enum
{
  AddrNotPresent = 0,
  AddrGroup = 1,
  Addr16Bit = 2,
  Addr64Bit = 3,
  AddrBroadcast = 15
};

typedef enum
{
  afAddrNotPresent = AddrNotPresent,
  afAddr16Bit      = Addr16Bit,
  afAddr64Bit      = Addr64Bit,
  afAddrGroup      = AddrGroup,
  afAddrBroadcast  = AddrBroadcast
} afAddrMode_t;

void zbInit(void);

void zbSecondTask(void);

void zbCanItf(uint8_t *pucData);

void zb_SerialResp(void);

void zb_PrepareReset(void);

void zbHwReset(void);

void zbLock(uint8_t ucLock);

void zbActivate(uint8_t ucEnable);

#endif
