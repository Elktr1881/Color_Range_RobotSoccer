// Copyright 2013 Joaquim Vasco Oliveira dos Santos 42421
#include "AbstractSocket.h"

extern "C" {
#include <stdlib.h>
#include <string.h>
#include <arpa/inet.h>
#include <unistd.h>
}

#include <ostream>
#include <cerrno>

using namespace std;

// ctor
AbstractSocket::AbstractSocket() {
  file_descriptor_ = -1;
  state_ = CLOSED;
  error_ = 0;
}

// dtor
AbstractSocket::~AbstractSocket() {
  if (state_ != CLOSED) {
    close();
  }
}

// ctor
AbstractSocket::AbstractSocket(const AbstractSocket& other) {
  file_descriptor_ = other.file_descriptor_;
  state_ = other.state_;
  error_ = 0;
}

AbstractSocket::AbstractSocket(const int file_descriptor,
    const SocketState state) {
  file_descriptor_ = file_descriptor;
  state_ = state;
  error_ = 0;
}

std::ostream& operator<<(std::ostream& out, AbstractSocket&
    abstractSocket) {
  //abstractSocket.printOn(out);
  return out;
}

AbstractSocket& AbstractSocket::operator=(const AbstractSocket& rhs) {
  // handle self assignment
  if (this == &rhs) return *this;
  file_descriptor_ = rhs.file_descriptor_;
  state_ = rhs.state_;

  return *this;
}

int AbstractSocket::get_error() const {
  return error_;
}

bool AbstractSocket::close() {
  if (state_ == CLOSED) {
    return true;
  }

  if (::close(file_descriptor_) == -1) {
    error_ = errno;
    return false;
  }

  state_ = CLOSED;

  return true;
}

bool AbstractSocket::isOpen() const {
  return state_ == OPEN;
}

bool AbstractSocket::isBounded() const {
  return state_ == BOUNDED;
}

bool AbstractSocket::isClosed() const {
  return state_ == CLOSED;
}

bool AbstractSocket::isConnected() const {
  return state_ == CONNECTED;
}

bool AbstractSocket::isListening() const {
  return state_ == LISTENING;
}

// get sockaddr, IPv4 or IPv6:
void* AbstractSocket::get_in_addr(struct sockaddr *sa)
{
	if (sa->sa_family == AF_INET) {
		return &(((struct sockaddr_in*)sa)->sin_addr);
	}
	return &(((struct sockaddr_in6*)sa)->sin6_addr);
}   
