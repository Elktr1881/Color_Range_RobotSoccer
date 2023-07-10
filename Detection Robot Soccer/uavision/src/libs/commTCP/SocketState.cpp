// Copyright 2013 Joaquim Vasco Oliveira dos Santos 42421
#include "SocketState.h"

std::ostream& operator<<(std::ostream& out, const SocketState state) {
    out << "state[";

    switch (state) {
        case OPEN:
            out << "open";
            break;
        case CLOSED:
            out << "closed";
            break;
        case BOUNDED:
            out << "bounded";
            break;
        case CONNECTED:
            out << "connected";
            break;
        case LISTENING:
            out << "listening";
            break;
        default:
            out << "unknown state";
            break;
    }

    out << "]";

    return out;
}
