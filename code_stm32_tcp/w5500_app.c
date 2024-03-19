/** ========================================

  Copyright Thesis Pte Ltd, 2018
  All Rights Reserved
  UNPUBLISHED, LICENSED SOFTWARE.

  CONFIDENTIAL AND PROPRIETARY INFORMATION
  WHICH IS THE PROPERTY OF THESIS PTE LTD.

  ========================================
  Version: 1.1
	Date: 15th May 2018
  Written by: Kenneth Er
  ========================================
	Application code for W5500 Ethernet chip

	======================================== */
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <string.h>
#include "w5500_app.h"
#include "w5500_spi.h"
#include "socket.h"
#include "wizchip_conf.h"
#include "dhcp.h"
#include "lowerlimb_config.h"

#if (W5500_DEBUG_PRINTF)
#include "uart_driver.h"
//#include "loopback.h"
#endif

/////////////////////
// PHYStatus check //
/////////////////////
#define SEC_PHYSTATUS_CHECK 		1		// sec
bool PHYStatus_check_enable = false;
bool PHYStatus_check_flag = true;

//////////////////////////////////////////////////
// Socket & Port number definition for Examples //
//////////////////////////////////////////////////
#define SOCK_DHCP					0		//DHCP and TCP socket # must be diff
#define SOCK_TCPS       	1
#define SOCK_UDPS       	2
#define PORT_TCPS					PORT_NUMBER
#define PORT_UDPS       	5000
#define MY_MAX_DHCP_RETRY	3

////////////////////////////////////////////////
// Shared Buffer Definition for Loopback test //
////////////////////////////////////////////////
//#define DATA_BUF_SIZE   2048 	// defined in w5500_app.h
uint8_t gReadDATABUF[DATA_BUF_SIZE];
uint8_t gWriteDATABUF[DATA_BUF_SIZE];
uint16_t ui16ReadLen = 0;
uint16_t ui16WriteLen = 0;
uint8_t ui8NewReadData = 0;

///////////////////////////
// Network Configuration //
///////////////////////////
#if (DHCP_SERV_EN) //---else if DHCP_SERV_EN == 1---//
wiz_NetInfo gHman_WIZNETINFO = { .mac = MAC_ADDRESS,
                            .ip = IP_ADDRESS,
                            .sn = {255, 255, 0, 0},
                            .gw = {255, 255, 255, 0},
                            .dns = {0, 0, 0, 0},
                            .dhcp = NETINFO_DHCP
                          };
static uint8_t my_dhcp_retry = 0;
#else
wiz_NetInfo gHman_WIZNETINFO = { .mac = MAC_ADDRESS,
														.ip = IP_ADDRESS,
														.sn = {255, 255, 0, 0},
														.gw = {255, 255, 255, 0},
														.dns = {0, 0, 0, 0},
														.dhcp = NETINFO_STATIC
													};
#endif
													
static uint8_t destination_ip[4] = 	{0, 0, 0, 0};
//static uint16_t destport = 	5000;
static uint8_t is_tcp_connected = 0; //tcp connection flag

//------------------------------
//Private Prototypes
//------------------------------
static void Net_Conf(void);
static void PHYStatus_Check(void);
static void my_ip_assign(void);
static void my_ip_assign(void);

/**
		@brief: Set the W5500 chip 6bytes MAC Id
    @param[in]: id0 = Mac ID byte #1
		@param[in]: id1 = Mac ID byte #2
		@param[in]: id2 = Mac ID byte #3
		@param[in]: id3 = Mac ID byte #4
		@param[in]: id4 = Mac ID byte #5
		@param[in]: id5 = Mac ID byte #6
*/
void set_ethernet_w5500_mac(uint8_t id0, uint8_t id1, uint8_t id2, 
																uint8_t id3, uint8_t id4, uint8_t id5)
{
	gHman_WIZNETINFO.mac[0] = id0;
	gHman_WIZNETINFO.mac[1] = id1;
	gHman_WIZNETINFO.mac[2] = id2;
	gHman_WIZNETINFO.mac[3] = id3;
	gHman_WIZNETINFO.mac[4] = id4;
	gHman_WIZNETINFO.mac[5] = id5;
}

/**
	@brief: Function to display the network information
*/
void Display_Net_Conf(void)
{
#if (W5500_DEBUG_PRINTF)
    uint8_t tmpstr[6] = {0,};
#endif

    ctlnetwork(CN_GET_NETINFO, (void*) &gHman_WIZNETINFO);

#if (W5500_DEBUG_PRINTF)
    // Display Network Information
    ctlwizchip(CW_GET_ID,(void*)tmpstr);

    if(gHman_WIZNETINFO.dhcp == NETINFO_DHCP)
        uart_printf("\r\n===== %s NET CONF : DHCP =====\r\n",(char*)tmpstr);
    else
        uart_printf("\r\n===== %s NET CONF : Static =====\r\n",(char*)tmpstr);
    uart_printf(" MAC : %02X:%02X:%02X:%02X:%02X:%02X\r\n", gHman_WIZNETINFO.mac[0], gHman_WIZNETINFO.mac[1], gHman_WIZNETINFO.mac[2], gHman_WIZNETINFO.mac[3], gHman_WIZNETINFO.mac[4], gHman_WIZNETINFO.mac[5]);
    uart_printf(" IP : %d.%d.%d.%d\r\n", gHman_WIZNETINFO.ip[0], gHman_WIZNETINFO.ip[1], gHman_WIZNETINFO.ip[2], gHman_WIZNETINFO.ip[3]);
    uart_printf(" GW : %d.%d.%d.%d\r\n", gHman_WIZNETINFO.gw[0], gHman_WIZNETINFO.gw[1], gHman_WIZNETINFO.gw[2], gHman_WIZNETINFO.gw[3]);
    uart_printf(" SN : %d.%d.%d.%d\r\n", gHman_WIZNETINFO.sn[0], gHman_WIZNETINFO.sn[1], gHman_WIZNETINFO.sn[2], gHman_WIZNETINFO.sn[3]);
    uart_printf("=======================================\r\n");
#endif
}

/**
	@brief: Configure the network on W5500 chip
*/
static void Net_Conf(void)
{
    /** wizchip netconf */
    ctlnetwork(CN_SET_NETINFO, (void*) &gHman_WIZNETINFO);
	
		HAL_Delay(10); //wait for 10ms
		
    Display_Net_Conf();
}

/**
	@brief: Function to check if ethernet is physically connected
*/
static void PHYStatus_Check(void)
{
    uint8_t tmp;

    //if not connected, stay in do...while loop
    do {
        ctlwizchip(CW_GET_PHYLINK, (void*) &tmp);

        if(tmp == PHY_LINK_OFF) {

        }
    } while(tmp == PHY_LINK_OFF);
}

/**
	@brief: Call back for ip assing & ip update from DHCP
*/
static void my_ip_assign(void)
{
    getIPfromDHCP(gHman_WIZNETINFO.ip);
    getGWfromDHCP(gHman_WIZNETINFO.gw);
    getSNfromDHCP(gHman_WIZNETINFO.sn);
    getDNSfromDHCP(gHman_WIZNETINFO.dns);
    gHman_WIZNETINFO.dhcp = NETINFO_DHCP;
    /** Network initialization */
    Net_Conf();      // apply from dhcp
#if (W5500_DEBUG_PRINTF)
    uart_printf("DHCP LEASED TIME : %ld Sec.\r\n", getDHCPLeasetime());
#endif
}

/**
	@brief: Call back for ip Conflict
*/
static void my_ip_conflict(void)
{
#if (W5500_DEBUG_PRINTF)
    uart_printf("CONFLICT IP from DHCP\r\n");
#endif
    //halt or reset or any...
    //while(1); // this example is halt. //todo??
		ethernet_w5500_reset();
}

/**
		@brief: Reset the ethernet DHCP and network
*/
void ethernet_w5500_reset(void)
{
		DHCP_stop();      // if restart, recall DHCP_init()
		Net_Conf();   		// apply the default static network and print out netinfo to serial
	
		//
    // ReInit DHCP Client
    //
    // must be set the default mac before DHCP started.
    setSHAR(gHman_WIZNETINFO.mac);
    DHCP_init(SOCK_DHCP, gReadDATABUF);
		// if you want different action instead defalut ip assign,update, conflict,
    // if cbfunc == 0, act as default.
    reg_dhcp_cbfunc(my_ip_assign, my_ip_assign, my_ip_conflict);
}

/**
		@brief: The ethernet tcp send function
		@param[in]: *buf = pointer fo write buffer
		@param[in]: size = write length.
*/
void ethernet_w5500_send_data(uint8_t buf[], uint16_t size)
{
	if(size > sizeof(gWriteDATABUF))
		size = sizeof(gWriteDATABUF);
	memcpy(gWriteDATABUF, buf, size);
	ui16WriteLen = size;
}

/**
		@brief: The ethernet tcp to reset send
*/
void ethernet_w5500_send_data_clear(void)
{
	ui16WriteLen = 0;
}

/**
		@brief: The ethernet tcp receive function
		@param[out]: *buf = pointer fo read buffer
		@retval: read length. 0 = no data
*/
uint16_t ethernet_w5500_rcv_data(uint8_t buf[])
{
	//return 0 if there's no data
	if(ui16ReadLen == 0)
		return 0;
	
	ui8NewReadData = 0; //mark data as read
	//copy read data
	memcpy(buf, gReadDATABUF, ui16ReadLen);
	return ui16ReadLen;
}

/**
		@brief: Check whether TCP is currently connected.
		@retval: 0 = not connected. 1 = connected
*/
uint8_t isTCPConnected(void)
{
	return is_tcp_connected;
}
/**
		@brief: The ethernet tcp check if there's new rx data
		@retval: 0 = no new data, 1 = new data
*/
uint8_t ethernet_w5500_new_rcv_data(void)
{
	return ui8NewReadData;
}

void SetAutoKeepAlive(uint8_t sn, uint8_t time) // time > 0
{
    setSn_KPALVTR(sn, time);
}

/**
		@brief: The ethernet tcp state to be run within the DHCP state (ethernet_w5500_state())
		@param[in]: sn = socket number for TCP
		@param[in]: *rbuf = pointer fo read buffer
		@param[in]: *rLen = pointer fo read length. 0 = no new data 
		@param[in]: *wbuf = pointer fo write buffer
		@param[in]: *wLen = pointer fo write length. 0 = no new data 
		@param[in]: *destip = pointer to the destination IPv4 address
		@param[in]: destport = port number for the TCP socket
		@retval[in]: 1 = ok. Rest = error code. Refer to socket.h
*/
static int32_t ethernet_w5500_tcp_state(uint8_t sn, uint8_t* rbuf, uint16_t *rLen, uint8_t* wbuf, 
																	uint16_t *wLen, uint8_t* destip, uint16_t destport)
{
    int32_t ret; // return value for SOCK_ERRORs
    uint16_t size = 0, sentsize=0;

    // Port number for TCP client (will be increased)
    uint16_t any_port = 	50000;

    if(wizphy_getphylink() == 0)
    {
        setSn_CR(sn,Sn_CR_CLOSE);
    }

    // Socket Status Transitions
    // Check the W5500 Socket n status register (Sn_SR, The 'Sn_SR' controlled by Sn_CR command or Packet send/recv status)
    switch(getSn_SR(sn)) {
    case SOCK_ESTABLISHED :
        if(getSn_IR(sn) & Sn_IR_CON) {	// Socket n interrupt register mask; TCP CON interrupt = connection with peer is successful
#if (W5500_DEBUG_PRINTF)
            uart_printf("%d:Connected to - %d.%d.%d.%d : %d\r\n",sn, destip[0], destip[1], destip[2], destip[3], destport);
#endif
            setSn_IR(sn, Sn_IR_CON);  // this interrupt should be write the bit cleared to '1'
					
						is_tcp_connected = 1; //indicate TCP connected
        }

        //////////////////////////////////////////////////////////////////////////////////////////////
        // Data Transaction Parts; Handle the [data receive and send] process
        //////////////////////////////////////////////////////////////////////////////////////////////
				*rLen = 0; //reset read length
        if((size = getSn_RX_RSR(sn)) > 0) { // Sn_RX_RSR: Socket n Received Size Register, Receiving data length
            if(size > DATA_BUF_SIZE) size = DATA_BUF_SIZE; // DATA_BUF_SIZE means user defined buffer size (array)
            ret = recv(sn, rbuf, size); // Data Receive process (H/W Rx socket buffer -> User's buffer)
            if(ret <= 0) return ret; // If the received data length <= 0, receive failed and process end
						*rLen = size; //update read length
						ui8NewReadData = 1;
        }
				
				// Send data if there's data to be written
				sentsize = 0;
				while((*wLen != sentsize) && (*wLen > 0)) {
						ret = send(sn, wbuf+sentsize, *wLen-sentsize); // Data send process (User's buffer -> Destination through H/W Tx socket buffer)
						if(ret < 0) { // Send Error occurred (sent data length < 0)
								close(sn); // socket close
								return ret;
						}
						sentsize += ret; // Don't care SOCKERR_BUSY, because it is zero.
				}
        //////////////////////////////////////////////////////////////////////////////////////////////
        break;

    case SOCK_CLOSE_WAIT :
        if((ret=disconnect(sn)) != SOCK_OK) return ret;
		
				is_tcp_connected = 0; //indicate TCP disconnected
#if (W5500_DEBUG_PRINTF)
        uart_printf("%d:Socket Closed\r\n", sn);
#endif
        break;

    case SOCK_INIT :
#if (W5500_DEBUG_PRINTF)
        uart_printf("%d:Try to connect to the %d.%d.%d.%d : %d\r\n", sn, destip[0], destip[1], destip[2], destip[3], destport);
#endif
				is_tcp_connected = 0; //indicate TCP disconnected
        if( (ret = connect(sn, destip, destport)) != SOCK_OK) return ret;	//	Try to TCP connect to the TCP server (destination)
        break;

    case SOCK_CLOSED:
        close(sn);
				is_tcp_connected = 0; //indicate TCP disconnected
        if((ret=socket(sn, Sn_MR_TCP, any_port++, 0x00)) != sn) return ret; // TCP socket open with 'any_port' port number

        SetAutoKeepAlive(sn, 1); // set Auto keepalive 10sec(2*5)

        break;
    default:
        break;
    }
    return 1;
}

/**
		@brief: The ethernet state which check for DHCP and TCP state
*/
void ethernet_w5500_state(void)
{
    int32_t ret;
	// uint16_t index;

    //this keep checking the TCP port. Needs to be included.
    if(PHYStatus_check_flag) {
        PHYStatus_check_flag = false;
        PHYStatus_Check();
    }
#if (DHCP_SERV_EN) //---else if DHCP_SERV_EN == 1---//
    switch(DHCP_run()) {
    case DHCP_IP_ASSIGN:
    case DHCP_IP_CHANGED:
        /** If this block empty, act with default_ip_assign & default_ip_update */

        break;
    case DHCP_IP_LEASED:
        //
        // TO DO NETWORK APPs.
        //

        //find and set server IP
        memcpy(destination_ip, &gHman_WIZNETINFO.ip, sizeof(destination_ip));
        destination_ip[3] = 1;
				
		// test rapid data dump
		if ((ret = ethernet_w5500_tcp_state(SOCK_TCPS, gReadDATABUF, &ui16ReadLen,
				gWriteDATABUF, &ui16WriteLen, destination_ip, PORT_TCPS)) < 0)
		{
#if (W5500_DEBUG_PRINTF)
            uart_printf("SOCKET ERROR : %ld\r\n", ret);
#endif
        }
										
#if (W5500_DEBUG_PRINTF)
//				if(ui16ReadLen > 0) //print rx data
//				{
//					uart_printf("Rx data:");
//					for(index = 0; index < ui16ReadLen; index++)
//						uart_send_byte_block(gReadDATABUF[index]);
//					uart_printf("\r\n");
//				}
#endif			
			
        break;
    case DHCP_FAILED:
        /** ===== Example pseudo code =====  */
        // The below code can be replaced your code or omitted.
        // if omitted, retry to process DHCP
        my_dhcp_retry++;
        if(my_dhcp_retry > MY_MAX_DHCP_RETRY) {
#if (W5500_DEBUG_PRINTF)
            uart_printf(">> DHCP %d Failed\r\n", my_dhcp_retry);
#endif					
            my_dhcp_retry = 0;
            ethernet_w5500_reset();
        }
        break;
    default:
        break;
    }
#else		//---else if DHCP_SERV_EN == 0---//
	//find and set server IP
	memcpy(destination_ip, &gHman_WIZNETINFO.ip, sizeof(destination_ip));
	destination_ip[3] = 1;

	// test rapid data dump
	if ((ret = ethernet_w5500_tcp_state(SOCK_TCPS, gReadDATABUF, &ui16ReadLen,
			gWriteDATABUF, &ui16WriteLen, destination_ip, PORT_TCPS)) < 0)
	{
#if (W5500_DEBUG_PRINTF)
	printf("SOCKET ERROR : %ld\r\n", ret); // was uart_printf
#endif
	}
										
#if (W5500_DEBUG_PRINTF)
//				if(ui16ReadLen > 0) //print rx data
//				{
//					uart_printf("Rx data:");
//					for(index = 0; index < ui16ReadLen; index++)
//						uart_send_byte_block(gReadDATABUF[index]);
//					uart_printf("\r\n");
//				}
#endif	

#endif //end of DHCP_SERV_EN
	//clear TCP send data whether it has been sent or not.
	ethernet_w5500_send_data_clear();
}

/**
		@brief: Init the ethernet system and W5500 chip
    @retval: 0 = OK. Non-zero = error;
*/
uint8_t ethernet_w5500_sys_init(void)
{
    //init W5500 chip
    W5500_Chip_Init();

    //
    // Init DHCP Client
    //
    // must be set the default mac before DHCP started.
    setSHAR(gHman_WIZNETINFO.mac);
#if (DHCP_SERV_EN) //---else if DHCP_SERV_EN == 1---//
    DHCP_init(SOCK_DHCP, gReadDATABUF);
    // if you want different action instead defalut ip assign,update, conflict,
    // if cbfunc == 0, act as default.
    reg_dhcp_cbfunc(my_ip_assign, my_ip_assign, my_ip_conflict);
#endif
    Net_Conf();

    /** PHY Status check enable */
    PHYStatus_check_enable = true;

    return HAL_OK;
}
/** [] END OF FILE */
