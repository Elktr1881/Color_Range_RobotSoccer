// Copyright 2013 Joaquim Vasco Oliveira dos Santos 42421
#ifndef SRC_VISION_SOCKETSTATE_H_
#define SRC_VISION_SOCKETSTATE_H_

#include <ostream>

enum SocketState {
    OPEN = 0,
    BOUNDED = 1,
    LISTENING = 2,
    CONNECTED = 3,
    CLOSED = 4,
    UNKNOWN_ = 5
};

std::ostream &operator<<(std::ostream &out, const SocketState state);

#endif  // SRC_VISION_SOCKETSTATE_H_
