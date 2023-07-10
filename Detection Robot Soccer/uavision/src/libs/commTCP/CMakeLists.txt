ADD_LIBRARY( commTCP SocketState.cpp AbstractSocket.cpp Socket.cpp ServerSocket.cpp udpclientserver.cpp )
SET_TARGET_PROPERTIES( commTCP PROPERTIES COMPILE_FLAGS "-std=c++0x" )
