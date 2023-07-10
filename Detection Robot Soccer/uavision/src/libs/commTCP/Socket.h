// Copyright 2013 Joaquim Vasco Oliveira dos Santos 42421
#ifndef SRC_VISION_SOCKET_H_
#define SRC_VISION_SOCKET_H_

#include <ostream>
#include <istream>
#include <string>
#include <vector>

#include "SocketState.h"
#include "AbstractSocket.h"

class Socket : public AbstractSocket {
  /* attributes */
  private:
    std::string remote_address_;
    int remote_port_;
    /* methods */
  public:
    /** Default Constructor */
    Socket();
    /** Default Destructor */
    virtual ~Socket();
    /** Copy Constructor
     * @param other Object to copy from.
     */
    Socket(const Socket& other);
    /** Constructor
     * @param remote_address The remote hostname or remote ip to which this 
     * socket will connect to.
     * @param remote_port The port to which the server is listening to.
     */
    Socket(const std::string& remote_address, const int remote_port);
    /** Constructor 
     * @param file_descriptor A pre-initialized network file_descriptor.
     * @param remote_address The remote hostname or remote ip to which this 
     * socket will connect to.
     * @param remote_port The port to wich the server is listening to.
     * @param state The state of this socket. View SocketState for more details.
     */
    Socket(const int file_descriptor,
        const std::string& remote_address,
        const int remote_port, const SocketState state);
    /** Assignment operator
     *  @param other Object to assign from
     *  @return A reference to this.
     */
    Socket& operator=(const Socket& rhs);
    int get_file_descriptor() const;
    /** Access remote_address_
     * @return The current value of remote_address_.
     */
    const std::string get_remote_address() const;
    /** Access remote_port_
     * @return The current value of remote_port_.
     */
    int get_remote_port() const;
    /** Opens this socket, bounding it to the stored port and connecting to the
     * stored host or remote_address.
     * @return True if successful, false otherwise.
     */
    bool open();
    /** Opens this socket, connecting to the specified host on the specified 
     * port.
     * @param hostname Host to connect to.
     * @param port Port host is listening.
     * @return True if successful, false otherwise.
     */
    bool open(const std::string &hostname, const int port);
    /** Sends the rhs string to the other side of this socket. Use this method 
     * to send human readable data. For binary data check the recv and send 
     * methods.
     * @param rhs The string/text to be sent.
     * @return A reference to this.
     */
    Socket& operator<<(const std::string &rhs);
    /** Sends data_size bytes of data to the other side. Use this method to 
     * send binary data. For human readable text, check "operator<<" and
     * "operator>>".
     * @param data Array with size equal or greater than data_size. This array 
     * may contain any type of chars, including the null character.
     * @param data_size The size of data to be sent in bytes.
     * @return The number of bytes sent or -1 in case of internal socket error.
     */
    int send(const void* data, const uint32_t data_size);
    /** Sends the contents of the vector in buffer to the other side. For 
     * human readable text, check "operator<<" and "operator<<".
     * @param buffer A vector of unsigned chars containing the data to be sent.
     * @return The number of bytes sent or -1 in case of internal socket error.
     */
    int send(const std::vector<unsigned char>* buffer);
    /** Sends the full content of data to the other side. First 4 bytes will be 
     * sent containing the size of data to be sent, and then send the data, 
     * finishing when all data has been sent, or if the socket fails for some 
     * reason. Use this method to send binary data, for human readable text, 
     * check "operator<<" and "operator>>".
     * @param data Array with size equal or greater than data_size. This array 
     * may contain any type of characters, including the null character.
     * @param data_size The size of data to be sent in bytes.
     * @return the number of bytes sent or -1 in case of internal socket error.
     */
    int sendAll(const void* data, const uint32_t data_size);
    /** Sends the full content of data to the other side. First 4 bytes will be 
     * sent containing the size of data to be sent, and then send the data, 
     * finishing when all data has been sent, or if the socket fails for some 
     * reason. Use this method to send binary data, for human readable text, 
     * check "operator<<" and "operator>>".
     * \param buffer A smart pointer to a char buffer.
     * \return the number of bytes sent or -1 in case of internal socket error.
     */
    int sendAll(const std::vector<unsigned char>* buffer);
    /** Receives the message, if any, from the other side of this socket.
     * One should use this method to receive human readable data. For binary 
     * data check the recv and send method.
     * @param rhs The string to be received from the other side.
     * @return A reference to this.
     */
    Socket& operator>>(std::string &rhs);
    /** Receives the content, if any, from the other side up to data_size 
     * bytes. 
     * Use this method to receive binary data. For human readable text, check 
     * "operator<<" and "operator>>".
     * @param data Array with size equal or greater than data_size. This array 
     * may obtain any type of chars, including the null character.
     * @param data_size The size of data in bytes.
     * @return The number of bytes read onto data or -1 in case of internal 
     * socket error.
     */
    int recv(void* data, unsigned int data_size);
    /** Receives the content, if any, from the other side up to the size of the
     * vector. 
     * Use this method to receive binary data. For human readable text, check 
     * "operator<<" and "operator>>".
     * @param buffer Vector of unsigned chars.
     * @return The number of bytes read or -1 in case of internal socket error.
     */
    int recv(std::vector<unsigned char>* buffer);
    /** Attempts to receive the full content of the next message. This method
     * will first receive 4 bytes indicating the amount of bytes the next 
     * message has. This will read the 4 bytes and place them onto data_size.
     * Then it will place the message onto data, either until the full message 
     * has been read or until the size of data has been reached.
     * For human readable text, check "operator<<" and "operator>>". 
     * @param data The buffer to place sizes.
     * @param data_size The size of data. This will be the changed to the size 
     * of the message.
     * @return The number of bytes read onto data, check get_error to check for 
     * internal socket error.
     */
    int recvAll(void* data, uint32_t* data_size);
    /** Receives the full content of the next message. This method will first
     * receive 4 bytes indicating the amount of bytes the next message has. 
     * Then this will change the capacity of buffer if needed and place the data
     * inside the vector.
     * For human readable text, check "operator<<" and "operator>>". 
     * @param buffer A vector of unsigned chars.
     * @return The number of bytes read onto buffer, check get_error to check 
     * for internal socket error.
     */
    int recvAll(std::vector<unsigned char>* buffer);

    void print();
  protected:
    //virtual void printOn(std::ostream& out) const;
};

#endif  // SRC_VISION_SOCKET_H_
