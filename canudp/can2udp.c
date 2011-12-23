// Compilation:
//   gcc -lpopt -lpcan -g -O4 can2udp.c -o can2udp
#include <stdlib.h>
#include <stdio.h>
#include <assert.h>
#include <errno.h>
#include <popt.h>
#include "pcan.h"
#include "libpcan.h" 
#include "sockit.h"

// from <cntl.h> TODO: not hardcode this
#define CAN_O_RDRW 00000002

static const char *OPT_canDevicePath = "/dev/pcan32";
static int OPT_baudrate = 250000;
static const char *OPT_udpDstHost = "localhost";
static int OPT_udpDstPort = 0xC000; // =49152
static int OPT_verbose = 1;
static int OPT_hexdump = 1;
static int OPT_rxPort = 0;
static int OPT_singlePort = 0;
static int OPT_uDelay = 5000;
static void *can = NULL;
static int sock = -1;
static const char *MAGIC = "PCAN";
#define MAGIC_LEN 4
#define UDP_BUFLEN 2048
static char buffer [ UDP_BUFLEN ];

void _failedMsg( const char *file, int line, const char* cmd ) {
  fprintf(stderr,"%s:%d FAILED: %s\n", file,line,cmd );
}; // ENDS: failedMsg
#define FAILED_MSG( cmd ) _failedMsg( __FILE__, __LINE__, cmd )

#define IF_LOUD if (OPT_verbose>0)
#define IF_DBG if (OPT_verbose>3)

static TPCANMsg *parseMsg( char *csr[] ) {
  static TPCANMsg msg;
  char *nxt;
  /* Skip initial whitespace */
  for (nxt=csr[0]; 
    *nxt==' ' || *nxt=='\n'; 
    nxt++);
  csr[0] = nxt;
  if (!*csr[0]) {
    return NULL;
  };
  /* Scan to end of line / message */
  for (nxt=csr[0]; 
    *nxt && *nxt != '\n'; 
    nxt++);
  *nxt = 0;
  /* Parse message string */
  int rc = sscanf(csr[0], "%x %hhx%hhx%hhx%hhx%hhx%hhx%hhx%hhx",
     &msg.ID, msg.DATA, msg.DATA+1, msg.DATA+2, msg.DATA+3,
      msg.DATA+4, msg.DATA+5, msg.DATA+6, msg.DATA+7 );
  if (rc<1) {
    IF_LOUD printf("Failed to parse '%s'\n", *csr );
    csr[0] = nxt+1;
    return NULL;
  };
  msg.LEN = rc-1;
  msg.MSGTYPE = MSGTYPE_STANDARD;
  /* Advance cursor to next message / terminating null */
  csr[0] = nxt+1;
  /* In debug mode -- dump the message */
  IF_DBG { 
    printf("PARSED 0x%04X ", (int)msg.ID );
    for (nxt=(char*)msg.DATA; (nxt-(char*)msg.DATA)<msg.LEN; nxt++)
      printf("%02X ", 0xFF & (int)*nxt );
    printf("\n");
  };
  return &msg;
}; /* ENDS: parseMsg */

static void *canOpener( void ) {
    IF_DBG printf("Opening CAN bus at %s\n", OPT_canDevicePath);
    void *can = LINUX_CAN_Open(OPT_canDevicePath, CAN_O_RDRW);
    IF_DBG printf("  Requested bitrate %d\n", OPT_baudrate);
    int bitRate = LINUX_CAN_BTR0BTR1(can, OPT_baudrate);
    IF_DBG printf("  ---> got %d\n", bitRate);
    int rc;
    // TODO: no return values?
    CAN_Status(can);
    IF_DBG printf("  Initializing\n");
    if ((rc=CAN_Init(can, bitRate, MSGTYPE_STANDARD))) {
      FAILED_MSG("CAN_Init");
      IF_DBG fprintf(stderr,"\t--> returned %d\n",rc);
      return NULL;
    };
    IF_DBG printf("  Setting message filter\n");
    CAN_ResetFilter(can);
    CAN_MsgFilter(can, 0x000, 0x7FF,MSGTYPE_STANDARD);
    IF_LOUD printf("CAN bus %s is open at %d baud\n", 
       OPT_canDevicePath, bitRate );
    return can;
}; // ENDS: canOpener

static int sockOpener( void ) {
    IF_DBG printf("Opening UDP socket\n");
    sock = newUdpSocket();
    if (sock<=0) {
      FAILED_MSG("newUdpSocket");
      return -1;
    };
    if (OPT_rxPort) {
      IF_DBG printf("   attempting to bind port %d\n", OPT_rxPort );
    } else {
      IF_DBG printf("   finding a free port\n"); 
    };
    if (setLocalPort( sock, OPT_rxPort )) {
      FAILED_MSG("binding local socket");
      return -2;
    };
    if (OPT_rxPort) {
      IF_DBG printf("   changing to non-blocking mode\n");
      if (setNonBlocking(sock)) {
        FAILED_MSG("setting socket to non-blocking mode");
        return -3;
      };
    };
    IF_LOUD printf("UDP socket opened at port %d\n",getLocalPort(sock));
    return sock;    
}; // ENDS: sockOpener

int main( int argc, char ** argv ) {
    struct sockaddr_in dst;
    struct poptOption opts [] = {
      { "candev",   'c',    POPT_ARG_STRING,    &OPT_canDevicePath, 0,
        "path to device node of the CAN device", "<device-path>" },
      { "dst",      'd',    POPT_ARG_STRING,    &OPT_udpDstHost, 0,
        "UDP destination hostname",              "<DNS-name-or-IP>" },
      { "dport",    'p',    POPT_ARG_INT,       &OPT_udpDstPort, 0,
        "UDP destination port number",           "<port>" },
      { "rport",    'r',    POPT_ARG_INT,       &OPT_rxPort, 0,
        "UDP receive port number (default 0 -- no receive function)",           "<port>" },
      { "baud",     'b',    POPT_ARG_INT,       &OPT_baudrate, 0,
        "CAN baud rate",                         "<baud>" },
      { "quiet",    'q',    POPT_ARG_VAL,       &OPT_verbose,  0,
        "be less verbose", },
      { "no hexdump",'x',    POPT_ARG_VAL,       &OPT_hexdump, 0,
        "disable hexdump of traffic", },
      { "singlePort",'s',   POPT_ARG_VAL,       &OPT_singlePort, 1,
        "use a single dst port (otherwise CAN ID added to port)", },
      { "debug",    'D',    POPT_ARG_INT,       &OPT_verbose, 0,
        "debug log level 0-9",                   "<verbosity>" },
       POPT_AUTOHELP
       POPT_TABLEEND 
    };
    TPCANRdMsg canMsg;
    int rc,k;
    char *csr [1];
    TPCANMsg *txMsg;
    
    poptContext pctx = poptGetContext(NULL, argc, (const char**)argv, opts,0);
    poptGetNextOpt(pctx);
    sock = sockOpener();
    if (sock<0) {
      perror("socket error");
      return -errno;
    };
    can = canOpener();
    if (!can) {
      perror("CAN error");
      return -errno;
    };
    
    if (hostAndPort( &dst, OPT_udpDstHost, OPT_udpDstPort )) {
      perror("Resolve destination host and port");
      return -errno;
    };
    
    for (;;) { // BEGINS: main loop
      rc = LINUX_CAN_Read_Timeout( can, &canMsg, OPT_uDelay );
      switch (rc) {
      case CAN_ERR_QRCVEMPTY: 
        break;
      case CAN_ERR_OK:
        if (!OPT_singlePort)
          dst.sin_port = htons( OPT_udpDstPort + canMsg.Msg.ID );
        rc = sendTo(sock, &canMsg, sizeof(canMsg), &dst);
        if (rc<0) {
          perror("UDP sendto");
        };
        if (OPT_hexdump) {
          printf( "%6d.%03d %04X %02X", canMsg.dwTime, canMsg.wUsec,
              canMsg.Msg.ID, canMsg.Msg.MSGTYPE );
          for (k=0; k<canMsg.Msg.LEN; k++)
            printf(" %02X",canMsg.Msg.DATA[k] );
          printf("\n");
        };
        break;
      default:
        fprintf(stderr,"CAN returned error code 0x%X\n", rc );
      }; /* ENDS: case on CAN RX */
      
      /* If receiving data enabled */
      if (!OPT_rxPort) 
        continue;
      rc = recvFrom( sock, buffer, UDP_BUFLEN, NULL );
      if (!rc) 
        continue;
      if (strncmp(buffer,MAGIC,MAGIC_LEN)) {
        IF_LOUD printf("Got packet with bad MAGIC string '%c%c%c%c'\n",
          buffer[0], buffer[1],buffer[2],buffer[3] );        
      } else { /* else --> got valid magic */
        /* Loop over packet contents */
        csr[0] = buffer + MAGIC_LEN;
        for (;;) {
          txMsg = parseMsg( csr );
          if (!txMsg)
             break;
          rc = LINUX_CAN_Write_Timeout( can, txMsg, OPT_uDelay );
          if (rc) {
            IF_LOUD printf("LINUX_CAN_Write_Timeout returned %d\n", rc );
          };
        }; /* ENDS loop on packet contents */
      }; /* ENDS: valid magic in RX packet */
    }; //ENDS: main loop
    
    CAN_Close(can);
    closeSocket(sock);
    poptFreeContext(pctx);
    return 0;
}; // ENDS: main

