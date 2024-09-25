#include "WebSocketSender.h"

WebSocketSender::WebSocketSender()
{
	//WebSocket 라이브러리 설정하기 2.2 버전
	WSAStartup(MAKEWORD(2, 2), &wsadata);
	//통신 형식 설정, AF_INET = IP v4, SOCK_DGRAM = UDP, 0 = 자동 설정
	m_clientSock = socket(AF_INET, SOCK_DGRAM, 0);
	//초기화
	ZeroMemory(&m_ClientAddr, sizeof(m_ClientAddr));
	m_ClientAddr.sin_family = AF_INET;
	m_ClientAddr.sin_addr.S_un.S_addr = inet_addr(SERVER_IP);
	m_ClientAddr.sin_port = htons(PORT);

	connected = true;

	std::cout << "UDP WebSocket Connected. \n";
}

WebSocketSender::~WebSocketSender()
{
	connected = false;
	//소켓 메모리 정리
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
	BYTE cameraid = 0, num = 0;
	BYTE buffer[IMG_SEG_SIZE + (sizeof(BYTE) * INFO_SIZE)] = {};

	//전송할때 데이터 맨 앞 3바이트에 패킷 정보를 함께 전송
	//각 바이트는 마지막 패킷여부, 카메라아이디, 이미지 번호를 의미한다.
	while (total_bytes_sent < img_packet_size) {
		chunk_size = min(IMG_SEG_SIZE, img_packet_size - total_bytes_sent);
		
		memset(buffer, 0, sizeof(buffer));	//0으로 초기화
		buffer[0] = total_bytes_sent + chunk_size < img_packet_size ? 0 : 255;	//마지막 패킷인지 검사
		buffer[1] = cameraid;
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
