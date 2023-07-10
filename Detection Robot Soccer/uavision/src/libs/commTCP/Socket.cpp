// Copyright 2013 Joaquim Vasco Oliveira dos Santos 42421
#include "Socket.h"

extern "C" {
#include <netdb.h>
#include <unistd.h>
#include <string.h>
#include <arpa/inet.h>
}

#include <sstream>
#include <iostream>
#include <cerrno>
#include <string>
#include <vector>

//#include "addrinfo_extra.h"

using namespace std;

Socket::Socket() : AbstractSocket() {
  remote_address_ = "";
  remote_port_ = 6000;
}

Socket::~Socket() {
}

Socket::Socket(const Socket& other) : AbstractSocket(other) {
  remote_address_ = other.remote_address_;
  remote_port_ = other.remote_port_;
}

Socket::Socket(const string& remote_address, const int remote_port) : 
  AbstractSocket() {
  remote_address_ = remote_address;
  remote_port_ = remote_port;
}

Socket::Socket(const int file_descriptor, const std::string& remote_address,
    const int remote_port, const SocketState state) :
  AbstractSocket(file_descriptor, state) {
    remote_address_ = remote_address;
    remote_port_ = remote_port;
}

Socket& Socket::operator=(const Socket& rhs) {
  AbstractSocket::operator= (rhs);
  remote_address_ = rhs.remote_address_;
  remote_port_ = rhs.remote_port_;
  return *this;
}

int Socket::get_file_descriptor() const {
  return file_descriptor_;
}

const string Socket::get_remote_address() const {
  return remote_address_;
}

int Socket::get_remote_port() const {
  return remote_port_;
}

bool Socket::open() {
  if (remote_address_.compare("") == 0) {
    error_ = -1;
    return false;
  }
  struct addrinfo hints, *servinfo, *p;
  int return_value;
  char s[INET6_ADDRSTRLEN];

  stringstream strs;
  strs << remote_port_;
  char const *port_str = strs.str().c_str();

  // Load up address structs with getaddrinfo():
  memset(&hints, 0, sizeof(hints));
  hints.ai_family = AF_UNSPEC;  // use either IPv4 or IPv6
  hints.ai_socktype = SOCK_STREAM;

  return_value = getaddrinfo(remote_address_.c_str(), port_str, &hints,
      &servinfo);
  if (return_value != 0) {
    cerr << "CERR: " << "getaddrinfo :" << gai_strerror(return_value) << endl;
    return false;
  }

  // loop through all the results and connect to the first possible
  for (p = servinfo; p != NULL; p = p->ai_next) {
    file_descriptor_ = ::socket(p->ai_family, p->ai_socktype, p->ai_protocol);
    if (file_descriptor_ == -1) {
      cerr << "CERR: " << "client: socket" << strerror(errno) << endl;
      error_ = errno;
      continue;
    }

    state_ = OPEN;

    if (::connect(file_descriptor_, p->ai_addr, p->ai_addrlen) == -1) {
      ::close(file_descriptor_);
      cerr << "CERR: " << "client: connect" << strerror(errno) << endl;
      error_ = errno;
      continue;
    }

    state_ = CONNECTED;

    break;
  }

  if (p == NULL) {
    cerr << "CERR: " << "client: failed to connect!" << endl;
    error_ = -1;
    return false;
  }

  inet_ntop(p->ai_family, 
      get_in_addr((struct sockaddr*)p->ai_addr), s,
      sizeof(s));
  remote_address_ = s;

  error_ = 0;

  freeaddrinfo(servinfo);

  return true;
}

bool Socket::open(const std::string &hostname, const int port) {
  remote_address_ = hostname;
  remote_port_ = port;
  return open();
}

Socket& Socket::operator<<(const std::string &rhs) {
  send(rhs.c_str(), rhs.size());
  return *this;
}

int Socket::send(const void* data, const uint32_t data_size) {
  ssize_t sent_bytes;
  /* Send the data. */
  sent_bytes = ::send(file_descriptor_, data, data_size, MSG_NOSIGNAL);
  /* Check for error on ::send. */
  if (sent_bytes == -1) {
    error_ = errno;
    cerr << "Socket error on send: " << strerror(error_) << endl;
  }

  return sent_bytes;
}

int Socket::send(const vector<unsigned char>* buffer) {
  ssize_t sent_bytes;
  /* send the data. */
  sent_bytes = ::send(file_descriptor_, buffer->data(), buffer->size(),
      MSG_NOSIGNAL);
  /* Check for error on ::send. */
  if (sent_bytes == -1) {
    error_ = errno;
    cerr << "Socket error on send: " << strerror(error_) << endl;
    return -1;
  }
  /* send the contents of buf. */
  return sent_bytes;
}

int Socket::sendAll(const void* data, const uint32_t data_size) {
  unsigned int total_bytes_sent = 0;
  int bytes_left = data_size;
  int bytes_sent;

  /* Send 4 bytes containing the number of bytes we are about to send. */
  uint32_t network_data_size = htonl(data_size);
  bytes_sent = ::send(file_descriptor_, &data_size,
      sizeof(network_data_size), MSG_NOSIGNAL);
  //cerr << "sendAll data_size" << data_size << " bytes " << "with value " << network_data_size << endl ;
  if (bytes_sent == -1) {
    error_ = errno;
    cerr << "Socket error on send: " << strerror(error_) << endl;
    return -1;
  } else if (bytes_sent < 4) {
    error_ = errno;
    cerr << "Failed to send 4 bytes at once????" << endl;
    return -1;
  }

  /* Send the total data until fail or until all bytes are sent. */
  while (total_bytes_sent < data_size) {
    bytes_sent = ::send(file_descriptor_,
        ((const unsigned char*)data)+total_bytes_sent, bytes_left,
        MSG_NOSIGNAL);
    //cerr << "sent " << bytes_sent << "( " << total_bytes_sent << " )" << std::endl;
    if (bytes_sent == -1) {
      error_ = errno;
      cerr << "Socket error on send: " << strerror(error_) << endl;
      return -1;
    }
    total_bytes_sent += bytes_sent;
    bytes_left -= bytes_sent;
  }

  return total_bytes_sent;
}

int Socket::sendAll(const vector<unsigned char>* buffer) {
  unsigned int total_bytes_sent = 0;
  int bytes_left = buffer->size();
  int bytes_sent;

  /* Send 4 bytes containing the number of bytes we are about to send. */
  uint32_t network_data_size = htonl(buffer->size());
  bytes_sent = ::send(file_descriptor_, &network_data_size,
      sizeof(network_data_size), MSG_NOSIGNAL);
  if (bytes_sent == -1) {
    error_ = errno;
    cerr << "Socket error on send: " << strerror(error_) << endl;
    return -1;
  } else if (bytes_sent < 4) {
    error_ = errno;
    cerr << "Failed to send 4 bytes at once????" << endl;
    return -1;
  }

  /* Send the total data until fail or until all bytes are sent. */
  while (total_bytes_sent < buffer->size()) {
    bytes_sent = ::send(file_descriptor_, buffer->data()+total_bytes_sent,
        bytes_left, MSG_NOSIGNAL);
    if (bytes_sent == -1) {
      error_ = errno;
      cerr << "Socket error on send: " << strerror(error_) << endl;
      return -1;
    }
    total_bytes_sent += bytes_sent;
    bytes_left -= bytes_sent;
  }

  return total_bytes_sent;
}

Socket& Socket::operator>>(std::string &rhs) {
  char buf[1500+1];
  rhs = "";
  memset(buf, 0, 1500+1);
  if (recv(buf, 1500+1) == -1) {
    rhs = "";
  }

  rhs = buf;

  return *this;
}

int Socket::recv(void* data, unsigned int data_size) {
  int recv_bytes;

  recv_bytes = ::recv(file_descriptor_, data, data_size, 0);
  if (recv_bytes == -1) {
    cerr << "CERR: " << "Socket error on receive: " << strerror(errno) << endl;
    error_ = errno;
  }

  return recv_bytes;
}

int Socket::recv(vector<unsigned char>* buffer) {
  int recv_bytes;

  recv_bytes = ::recv(file_descriptor_, buffer->data(), buffer->capacity(), 0);

  if (recv_bytes == -1) {
    cerr << "CERR: " << "Socket error on receive: " << strerror(errno) << endl;
    error_ = errno;
  }

  return recv_bytes;
}

int Socket::recvAll(void* data, uint32_t* data_size) {
  unsigned int buffer_size = *data_size;
  unsigned int min = 0;
  int bytes_recv;
  unsigned int total_recv_bytes = 0;

  bytes_recv = ::recv(file_descriptor_, data_size, 4, 0);
  if (bytes_recv == -1) {
    cerr << "CERR: " << "Socket error on receive: " << strerror(errno) << endl;
    error_ = errno;
    return -1;
  } else if (bytes_recv != 4) {
    cerr << "CERR: " << "Received more than 4 bytes?" << strerror(errno)
      << endl;
    error_ = errno;
    return -1;
  }

  //cerr << "revAll  data_size " << *data_size;
  min = (buffer_size < *data_size)?buffer_size:*data_size;
  //*data_size = ntohl(*data_size);
  //cerr << "revAll  data_size after " << *data_size << std::endl;
  while (total_recv_bytes != *data_size && total_recv_bytes != buffer_size) {
    bytes_recv = ::recv(file_descriptor_,
        ((unsigned char*)data)+total_recv_bytes, min-total_recv_bytes, 0);
    //cerr << "received " << bytes_recv << " should be " << min-total_recv_bytes << " ( " << total_recv_bytes << " )" << std::endl;
    if (bytes_recv == -1) {
      cerr << "CERR: " << "Socket error on receive: " << strerror(errno)
        << endl;
      error_ = errno;
      break;
    }
    total_recv_bytes += bytes_recv;
  }

  return total_recv_bytes;
}

int Socket::recvAll(vector<unsigned char>* buffer) {
  unsigned int total_bytes_recv = 0;
  int bytes_left;
  int bytes_recv;
  uint32_t buffer_size;

  /* Recv 4 bytes containing the number of bytes we are about to recv. */
  bytes_recv = ::recv(file_descriptor_, &buffer_size, sizeof(buffer_size),
      MSG_NOSIGNAL);
  buffer_size = ntohl(buffer_size);
  if (bytes_recv == -1) {
    error_ = errno;
    cerr << "Socket error on send: " << strerror(error_) << endl;
    return -1;
  } else if (bytes_recv < 4) {
    error_ = errno;
    cerr << "Failed to recv 4 bytes at once????" << endl;
    return -1;
  }
  buffer->clear();
  buffer->resize(buffer_size);
  bytes_left = buffer_size;

  /* Recv the total data until fail or until all bytes are recv. */
  while (total_bytes_recv < buffer_size) {
    bytes_recv = ::recv(file_descriptor_, buffer->data()+total_bytes_recv,
        bytes_left, 0);
    if (bytes_recv == -1) {
      error_ = errno;
      cerr << "Socket error on send: " << strerror(error_) << endl;
      break;
    }
    total_bytes_recv += bytes_recv;
    bytes_left -= bytes_recv;
  }

  return total_bytes_recv;
}

void Socket::print()
{
	std::cerr << "Address: " << remote_address_<< " port: " << remote_port_ << std::endl;
}
/*
void Socket::printOn(ostream& out) const {
  out << "Socket #" << file_descriptor_ << "{ ";
  out << " R@" << remote_address_ << ":" << remote_port_ << "}";
}
*/
