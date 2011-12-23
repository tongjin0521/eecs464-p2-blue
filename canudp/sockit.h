/** \file sockit.h
    A UDP socket interface with little to no hassle.<br>
    If you ever programmed the socket interface directly, you realize two things
    rather quickly: (1) there is no simpler interface that can give all the 
    functionality -- i.e. all "object" wrappers for sockets make things more
    complicated instead of simpler; (2) you need to remember a heck of alot 
    of deails to get everything done right. <br>
    The interface in this header file tries to correct (2) without the sin 
    of (1). That's it. Sockets are still just file-desciprtors -- no fancy
    object wrappers. But the common use-cases have simple, easy to use API-s
    and you don't need to remember which header inet_addr was defined in...<br>
    On error, the global variable sockit_msg contains a human readable error
    message. <br>
    Everything is defined in the header as static functions, so that the 
    compiler's optimizer can inline the nonsense and you don't lose any
    performance. <br>
    Usage is geared towards nonblocking operation, rather than poll() or
    select(). <br>

*/
#ifndef __SOCKIT__HH
#define __SOCKIT__HH
#include <sys/types.h>       // For data types
#include <sys/socket.h>      // For socket(), connect(), send(), and recv()
#include <netdb.h>           // For gethostbyname()
#include <arpa/inet.h>       // For inet_addr()
#include <unistd.h>          // For close()
#include <netinet/in.h>      // For sockaddr_in
#include <fcntl.h>           // For non-blocking operation
#include <errno.h>           // Error decode; required for non-blocking ops
#include <string.h>          // For memset()
static const char *sockit_msg = NULL;

// Location string macro needs to be done in multiple levels 
//   as suggested in section 3.4 of GNU preprocessor docs
#define STR(line) #line
#define __SOCKIT_AT(line) __FILE__ ":" STR(line) 
#define SOCKIT_HERE __SOCKIT_AT(__LINE__)
#define SOCKIT_SAYS( msg ) \
    sockit_msg = (SOCKIT_HERE  " -- " msg)

#ifndef SOCKET_ERROR
#define SOCKET_ERROR (int)-1
#endif 

/** Return the local address of the socket as a string.
    \return a string from inet_ntoa(). DANGER: read the manpage! */
static const char *getLocalAddress(int sock) {
  struct sockaddr_in addr;
  unsigned int addr_len = sizeof(addr);

  SOCKIT_SAYS("called getsockname()");
  if (getsockname(sock, 
          (struct sockaddr *) &addr, 
          (socklen_t *) &addr_len
        ) < 0) {
    return (const char*)0;
  }
  return inet_ntoa(addr.sin_addr);
};

/** Get the local port number for the socket
    \return port number
  */
static unsigned short getLocalPort(int sock) {
  struct sockaddr_in addr;
  unsigned int addr_len = sizeof(addr);

  SOCKIT_SAYS("getsockname() called");
  if (getsockname(sock, 
          (struct sockaddr *) &addr, 
          (socklen_t *) &addr_len
        ) < 0) {
    return 0;
  }
  return ntohs(addr.sin_port);
};

/** set the local port a socket is using.
    Only call this on unbound sockets. It binds the socket to the specified
    port on all interfaces.
    \param sock the socket
    \param localPort the port
    \return the return value of bind() 
*/
static int setLocalPort(int sock, unsigned short localPort) {
  // Bind the socket to its port
  struct sockaddr_in localAddr;
  memset(&localAddr, 0, sizeof(localAddr));
  localAddr.sin_family = AF_INET;
  localAddr.sin_addr.s_addr = htonl(INADDR_ANY);
  localAddr.sin_port = htons(localPort);
  SOCKIT_SAYS("bind() called");
  return bind(sock, (struct sockaddr *) &localAddr, sizeof(struct sockaddr_in));
};

/** Convert string and port number to a sockaddr_in.
    Will process both DNS names and dot notation strings
    \param addr output
    \param address address in string form
    \param port port number
    \return 0 on success, other on failure
*/    
static int hostAndPort(
  struct sockaddr_in *addr, 
  const char *address, 
  unsigned short port
) {
  memset(addr, 0, sizeof(*addr));  // Zero out address structure
  addr->sin_family = AF_INET;       // Internet address

  SOCKIT_SAYS("inet_aton() called");
  int res = inet_aton( address, &addr->sin_addr );
  if (res==0) { // if failed --> resolve with DNS
    struct hostent *host;  // Resolve name
    SOCKIT_SAYS("gethostbyname() called");
    if ((host = gethostbyname(address)) == NULL) {
      return -1;
    }
    addr->sin_addr.s_addr = *((unsigned long *) host->h_addr_list[0]);
  };
  addr->sin_port = htons(port);     // Assign port in network byte order
  return 0;
};

/** set local address and local port for a socket
    Use to bind and unbound socket to and address and port.
    \param sock the socket
    \param localAddress local address as string for hostAndPort()
    \param localPort local port number
    \return the return value of bind()
*/    
static int setLocalAddressAndPort(
    int sock, 
    const char *localAddress,
    unsigned short localPort
) {
  // Get the address of the requested host
  struct sockaddr_in localAddr;
  int res = hostAndPort(&localAddr, localAddress, localPort);
  if (res)
    return res;
  
  SOCKIT_SAYS("bind() called");
  return bind(sock, (struct sockaddr *) &localAddr, sizeof(struct sockaddr_in));
};

/** Create a new UDP socket 
    \return socket or SOCKET_ERROR if failed
*/
static int newUdpSocket( void ) {
  SOCKIT_SAYS("socket() called");
  int sock = socket( AF_INET, SOCK_DGRAM, 0  );
  if (sock<0)
    return SOCKET_ERROR;
  
  int True = 1;
  if (0 != setsockopt( sock,  SOL_SOCKET, SO_REUSEADDR, &True, sizeof(True))) {
    SOCKIT_SAYS("setsockopt(SO_REUSEADDR) failed");
  };
  return sock;
};

/** Close a socket created with newUdpSocket()
    Use this in case someone decides to port the code to a platform like 
    windows, that never heard of file desciptors.
    \param sock the socket
*/
static void closeSocket( int sock ) {
  close(sock);
};

/** Make the socket non-blocking
    \param sock socket
    \return the return value of fcntl()
*/
static int setNonBlocking( int sock ) {
  SOCKIT_SAYS("fcntl(O_NONBLOCK) failed");
  return fcntl( sock, F_SETFD, O_NONBLOCK );
};

/** Send a packet to the specified destination.
    Sends a packet to destination specified as a sockaddr_in. You can resolve 
    a string and a number into a sockaddr_in using hostAndPort().
    \param sock the socket
    \param msg pointer to the data block to send
    \param msgLen length of the data to send
    \param dst destination to send to
    \return 0 on success, nonzero on error.
*/
static int sendTo( 
    int sock, 
    const void *msg, 
    unsigned short msgLen, 
    struct sockaddr_in *dst 
) {
  SOCKIT_SAYS("sendto() called");
  int res = sendto(sock, msg, msgLen, 0, 
    (const struct sockaddr*)dst, sizeof(struct sockaddr_in) );
  return (res-msgLen);
};

/** receive a message from the socket.
    If the socket is non-blocking and not data is ready, returns 0. If the 
    buffer is too small for the message, only the first part of the message is
    retrieved; the rest is discarded.<br>
    If src is not NULL (i.e. 0), the sender's address is returned. <br>
    My apologies to the purists who want to be able to send 0-length UDP
    keep-alive messages. Those won't work with this API.
    \param sock the socket
    \param msg a buffer for the data
    \param maxLen the maximal length to read into the buffer.
    \param src pointer to storage for sender's address
    \return length of message, or 0 if nothing was received. */
static int recvFrom( 
    int sock,
    void *msg,
    unsigned int maxLen,
    struct sockaddr_in *src 
) {

  int len;
  int res;
  
  if (src) {
    SOCKIT_SAYS( "recvfrom() called" );
    res = recvfrom( sock, msg, maxLen, MSG_DONTWAIT, 
      (struct sockaddr*) src, &len );
  } else {
    SOCKIT_SAYS( "recv() called" );
    res = recv( sock, msg, maxLen, MSG_DONTWAIT );
  };
  
  if (res==-1 && errno==EAGAIN)
    res = 0;
  return res;
};


#endif // __SOCKIT_HH
// :mode=c++:tabSize=2:indentSize=2:wrap=none:folding=indent:
