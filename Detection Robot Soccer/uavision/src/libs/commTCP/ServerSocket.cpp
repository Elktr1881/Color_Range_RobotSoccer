#include "ServerSocket.h"
//#include "addrinfo_extra.h"

extern "C" {
#include <string.h>
}

#include <cerrno>
#include <sstream>
#include <iostream>

using namespace std;

ServerSocket::ServerSocket() : AbstractSocket() {
	back_log_ = 10;
	local_port_ = 6000;
}

ServerSocket::~ServerSocket() {
}

ServerSocket::ServerSocket(const unsigned int back_log) : AbstractSocket() {
	back_log_ = back_log;
	local_port_ = 6000;
}

ServerSocket::ServerSocket(const int port) : AbstractSocket() {
	back_log_ = 5;
	local_port_ = port;
}

ServerSocket::ServerSocket(const int port, const unsigned int back_log) {
	back_log_ = back_log;
	local_port_ = port;
}

unsigned int ServerSocket::get_back_log() {
	return back_log_;
}

void ServerSocket::set_back_log(const unsigned int back_log) {
	back_log_ = back_log;
}

bool ServerSocket::open() {
	if(isOpen()) {
		error_ = -1;
		return false;
	}

	int return_value;
	struct addrinfo hints, *servinfo, *p;
	stringstream strs;
	strs << local_port_;
	char const *port_str = strs.str().c_str();

	memset(&hints, 0, sizeof(hints));
	hints.ai_family = AF_UNSPEC;
	hints.ai_socktype = SOCK_STREAM;
	hints.ai_flags = AI_PASSIVE;
	return_value = getaddrinfo(NULL, port_str, &hints, &servinfo);
	if(return_value != 0) {
		cerr << "getaddrinfo: " << gai_strerror(return_value) << endl;
		error_ = -1;
		return false;
	}

	for(p = servinfo; p != NULL; p = p->ai_next) {
		file_descriptor_ = ::socket(p->ai_family, p->ai_socktype, p->ai_protocol);
		if(file_descriptor_ == -1) {
			cerr << "Server: socket" << strerror(errno) << endl;
			error_ = errno;
			continue;
		}
		state_ = OPEN;

		//setsockopt
		int yes = 1;
		if(::setsockopt(file_descriptor_, SOL_SOCKET, SO_REUSEADDR, &yes,
				sizeof(int)) == -1) {
			cerr << "Server: setsockopt" << strerror(errno) << endl;
			error_ = errno;
			continue;
		}

		if(::bind(file_descriptor_, p->ai_addr, p->ai_addrlen) == -1) {
			close();
			cerr << "ServerSocket error on bind: " << strerror(errno) << endl;
			error_ = errno;
			continue;
		}

		state_ = BOUNDED;
		error_ = 0;
		break;
	}

	if(error_ != 0) {
		return false;
	}

	freeaddrinfo(servinfo);

	if(::listen(file_descriptor_, back_log_) == -1) {
		cerr << "ServerSocket error on listen: " << strerror(errno) << endl;
		error_ = errno;
		return false;
	}

	return true;

}

bool ServerSocket::open(const int port) {
	local_port_ = port;
	return open();
}

Socket *ServerSocket::accept() {
	struct sockaddr_storage remote_address;
	socklen_t sin_size = sizeof(remote_address);
	int client_fd = -1;

	client_fd = ::accept(file_descriptor_, (struct sockaddr*)&remote_address, 
			&sin_size);

	if(client_fd == -1) {
		cerr << "ServerSocket error on accept: " << strerror(errno) << endl;
		error_ = errno;
		return new Socket();
	}
	//clone_addrinfo_from_sockaddr_storage(&remote_address);

	Socket *tmp = new Socket(client_fd, "", -1, CONNECTED);
	return tmp;
}
