#ifndef SERVERSOCKET_H
#define SERVERSOCKET_H

#include "AbstractSocket.h"
#include "Socket.h"
#include "SocketState.h"

#include <ostream>

class ServerSocket : public AbstractSocket {
	/* attributes */
protected:
	unsigned int back_log_; //!< Member variable "back_log_"
	int local_port_;
	/* methods */
public:
	/** Default constructor */
	ServerSocket();
	/** Default destructor */
	virtual ~ServerSocket();
	/** Construtor with back_log definition
	 * @param back_log The maximum number of clients the server will handle. */
	ServerSocket(const unsigned int back_log);
	/** Constructor with port definition
	 * @param port The port to wich this Server will be listening to
	 * connections.
	 */
	ServerSocket(const int port);
	/** Constructor with port definition
	 * @param port The port to wich this Server will be listening to
	 * connections.
	 * @param back_log The maximum number of clients the server will handle.
	 */
	ServerSocket(const int port, const unsigned int back_log);
	/** Access back_log
	 * \return The current value of back_log_
	 */
	unsigned int get_back_log();
	/** Modifier back_log
	 * \param back_log Number of max clients.
	 */
	void set_back_log(const unsigned int back_log);
	/** Opens this socket, bounds it and sets to listen.
	 * \return True if successful, false otherwise. If false, error will be set
	 * accordingly.
	 */
	bool open();
	/** Opens this socket, bounds it and sets to listen.
	 * \param port Port number to bind.
	 * \return True if successful, false otherwise.
	 */
	bool open(const int port);
	/** Accepts incoming connections an returns a Socket.
	 * \return Configured Socket to talk to the other side.
	 */
	Socket* accept();
};

#endif
