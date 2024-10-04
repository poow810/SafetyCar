﻿#include "WebSocketSender.h"

WebSocketSender::WebSocketSender()
{	
	// https 통신으로 아이피 등록하기
	set_connection();


	//WebSocket 라이브러리 설정하기 2.2 버전
	WSAStartup(MAKEWORD(2, 2), &wsadata);
	//통신 형식 설정, AF_INET = IP v4, SOCK_DGRAM = UDP, 0 = 자동 설정
	m_clientSock = socket(AF_INET, SOCK_DGRAM, 0);
	//초기화
	ZeroMemory(&m_ClientAddr, sizeof(m_ClientAddr));
	m_ClientAddr.sin_family = AF_INET;
	m_ClientAddr.sin_addr.S_un.S_addr = inet_addr(SERVER_IP);
	//DOMAIN TEST
	/*ADDRINFO hints;
	ZeroMemory(&hints, sizeof(hints));
	hints.ai_family = AF_INET;
	hints.ai_socktype = SOCK_STREAM;

	if (!getaddrinfo(SERVER_DOMAIN, PORT, hints, host_domainAddr)) {
		std::cerr << "ERROR\n";
		exit(-1);
		return;
	}

	SOCKADDR_IN* inAddr = (LPSOCKADDR_IN)host_domainAddr->ai_addr;
	m_ClientAddr.sin_addr = inAddr->sin_addr;

	std::cout << inAddr->sin_addr.S_un.S_addr << std::endl;*/
	// TEST END
	m_ClientAddr.sin_port = htons(PORT);

	connected = true;
	set_cameraId();
	std::cout << "UDP WebSocket Connected. \n";
	
}

WebSocketSender::~WebSocketSender()
{
	connected = false;
	//소켓 메모리 정리
	freeaddrinfo(host_domainAddr);
	WSACleanup();
}

void WebSocketSender::sendframe_via_udp(cv::InputArray frame)
{
	if (!isconnected()) {
		std::cerr << "UDP Not Connected! Failed to send.\n";
		return;
	}
	std::vector<BYTE> bytes = mat2jpg(frame);

	int img_packet_size = bytes.size();
	int total_bytes_sent = 0, sent_bytes, chunk_size;
	BYTE num = 0;
	BYTE buffer[IMG_SEG_SIZE + (sizeof(BYTE) * INFO_SIZE)] = {};

	//전송할때 데이터 맨 앞 3바이트에 패킷 정보를 함께 전송
	//각 바이트는 마지막 패킷여부, 카메라아이디, 이미지 번호를 의미한다.
	while (total_bytes_sent < img_packet_size) {
		chunk_size = min(IMG_SEG_SIZE, img_packet_size - total_bytes_sent);
		
		memset(buffer, 0, sizeof(buffer));	//0으로 초기화
		buffer[0] = total_bytes_sent + chunk_size < img_packet_size ? 0 : 255;	//마지막 패킷인지 검사
		buffer[1] = camera_id;
		buffer[2] = num++;
		memcpy(buffer + (sizeof(BYTE) * INFO_SIZE), bytes.data() + total_bytes_sent, chunk_size);
		sent_bytes = sendto(m_clientSock, reinterpret_cast<char*>(buffer), chunk_size + (sizeof(BYTE) * INFO_SIZE), 0, (SOCKADDR*)&m_ClientAddr, sizeof(m_ClientAddr));

		//보낸 데이터의 양이 -1인 경우는 오류인 경우
		if (sent_bytes == SOCKET_ERROR) {
			//check when error occured
			//https://learn.microsoft.com/en-us/windows/win32/api/winsock2/nf-winsock2-sendto
			std::cerr << "ERROR code: " << WSAGetLastError() << "\n";
			//delete &bytes;
			return;
		}

		total_bytes_sent += chunk_size;
	}

	std::cout << "Packet sent : " << total_bytes_sent << "\n" << "Seg sent : " << (short)num << "\n";
	//std::cout << "(%) : " << (float)total_bytes_sent / (float)IMG_FULL_SIZE << "\n";
}

std::vector<BYTE> WebSocketSender::mat2jpg(cv::InputArray mat)
{
	std::vector<BYTE> buff;
	cv::imencode(".jpg", mat, buff, encode_param);
	return buff;
}

void WebSocketSender::set_cameraId()
{
	int id;
	std::cout << "Please, Enter Camera ID : ";
	std::cin >> id;
	camera_id = (BYTE)id;
}

void WebSocketSender::set_connection()
{
	SSL_CTX* ctx;
	SSL* ssl;

	// OpenSSL 초기화
	SSL_library_init();
	OpenSSL_add_all_algorithms();
	SSL_load_error_strings();
	ctx = SSL_CTX_new(TLS_client_method());



	std::string ipAddress = SERVER_IP;
	//std::string ipAddress = SERVER_DOMAIN;
	int port = 443;
	//int port = 8080;

	// initialise winsock
	WSADATA data;
	WORD ver = MAKEWORD(2, 2);
	int wsResult = WSAStartup(ver, &data);
	if (wsResult != 0) {
		std::cerr << "Can't start winsock, Err #" << wsResult << std::endl;
		return;
	}

	// create socket
	SOCKET clientSocket = socket(AF_INET, SOCK_STREAM, 0);
	if (clientSocket == INVALID_SOCKET) {
		std::cerr << "Can't create socket, Err #" << WSAGetLastError() << std::endl;
		WSACleanup();
		return;
	}

	// hint structure
	sockaddr_in hint;
	hint.sin_family = AF_INET;
	hint.sin_port = htons(port);
	inet_pton(AF_INET, ipAddress.c_str(), &hint.sin_addr);

	// connect
	int connResult = connect(clientSocket, (sockaddr*)&hint, sizeof(hint));
	if (connResult == SOCKET_ERROR) {
		std::cerr << "Can't connect to server, Err #" << WSAGetLastError << std::endl;
		closesocket(clientSocket);
		WSACleanup();
		return;
	}



	// SSL 연결 설정
	ssl = SSL_new(ctx);
	SSL_set_fd(ssl, clientSocket);
	SSL_connect(ssl);
	std::string body = "{\"Access_token\":\"1234\"}"; // JSON 데이터
	size_t body_length = strlen(body.c_str());
	std::string request = "POST /api/connect HTTP/1.1\r\nHost: j11b209.p.ssafy.io\r\nConnection: close\r\nContent-Type: application/json\r\nContent-Length: " + std::to_string(body_length) + "\r\n\r\n" + body;
	
	SSL_write(ssl, request.c_str(), strlen(request.c_str()));

	// 응답 받기
	char buf[1024];
	int bytes;
	while ((bytes = SSL_read(ssl, buf, sizeof(buf))) > 0) {
		buf[bytes] = 0;
		printf("%s", buf);
	}

	/*
	// do while loop to send and receive data
	char buff[4096];

	char cmd[] = "GET /test HTTP/1.0\r\nHost: j11b209.p.ssafy.io\r\n\r\n";

	int sendResult = send(clientSocket, cmd, sizeof(cmd), 0);

	if (sendResult != SOCKET_ERROR) {
		// wait for response
		ZeroMemory(buff, 4096);
		int bytesReceived = recv(clientSocket, buff, 4096, 0);

		// echo response to console
		if (bytesReceived > 0) {
			std::cout << std::string(buff, 0, bytesReceived) << std::endl;
		}
	}*/

	// 연결 종료
	SSL_shutdown(ssl);
	SSL_free(ssl);
	SSL_CTX_free(ctx);

	closesocket(clientSocket);
	WSACleanup();
	return;
}
