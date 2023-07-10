// Copyright 2013 Joaquim Vasco Oliveira dos Santos 42421
#ifndef SRC_VISION_ABSTRACTSOCKET_H_
#define SRC_VISION_ABSTRACTSOCKET_H_

extern "C" {
#include <netdb.h>
}

#include <ostream>

#include "SocketState.h"

class AbstractSocket {
  /* atributes */
 protected:
  mutable int error_;  //!< Member variable "error_"
  int file_descriptor_;  //!< Member variable "file_descriptor_"
  SocketState state_;  //!< Member variable "state_"
  /* methods */
 public:
  /** Default constructor */
  AbstractSocket();
  /** Default destructor */
  virtual ~AbstractSocket();
  /** Copy constructor
   * \param other Object to copy from
   */
  explicit AbstractSocket(const AbstractSocket& other);
  /** Constructor that allows to define all parameters.
   * \param file_descriptor File descriptor of socket.
   * \param state State of this socket.
   */
  AbstractSocket(const int file_descriptor, const SocketState state);
  friend std::ostream& operator<<(std::ostream& out, AbstractSocket&
      abstractSocket);
  /** Assignment operator
   * \param other Object to assign from
   * \return A reference to this
   */
  AbstractSocket& operator=(const AbstractSocket& rhs);
  int get_error() const;
  bool close();
  bool isOpen() const;
  bool isBounded() const;
  bool isClosed() const;
  bool isConnected() const;
  bool isListening() const;

 protected:
  virtual bool open() = 0;
  //virtual void printOn(std::ostream& out) const = 0;
  // get sockaddr, IPv4 or IPv6:
  void* get_in_addr(struct sockaddr *sa);
};

#endif  // SRC_VISION_ABSTRACTSOCKET_H_
