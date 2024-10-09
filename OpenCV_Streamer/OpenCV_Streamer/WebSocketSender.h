#pragma once
#define _WINSOCK_DEPRECATED_NO_WARNINGS
#include <opencv2/opencv.hpp>
#include <WinSock2.h>
#include <WS2tcpip.h>
#include <openssl/ssl.h>
#include <openssl/err.h>

#pragma comment(lib, "ws2_32")

//upd max packet size	65535 bytes
//mat 2 jpg size		about 23805 ?
//cv::Mat size			mat.step[0] * mat.rows;
//						mat.total() * mat,elemSize();
//						640 * 640 * 3 = 1,228,800 bytes
//						1,228,800 / 20 = 61,440

//MTU는 보통 1500으로 설정되어 있다.
//CMD에서 이를 변경할 수 있지만 권장되지 않는다고 한다.
//따라서 패킷을 1500에 맞추어 잘라서 보내는 로직이 필요하다.
#define PORT			5432
#define MTU				1500
#define UDP_HEADER_SIZE 28
#define INFO_SIZE		4						//카메라 번호 및 이미지 번호를 담는 임의의 사용자 정의 패킷 헤더(UDP 헤더와 별개)의 크기(바이트) - End_Flag, CameraID, ImageSegNum
#define PACKET_SIZE		MTU - UDP_HEADER_SIZE	//MTU - UPD-Header = 1500 - 28 = 1472
#define IMG_SEG_SIZE	PACKET_SIZE - INFO_SIZE
#define IMG_QUALITY		20						//jpeg 형식으로 변환할때 화질 설정
//#define SERVER_IP		"127.0.0.1"
#define SERVER_IP		"43.202.61.242"
#define SERVER_DOMAIN	"j11b209.p.ssafy.io"

#define MAX_CACHE 3

//이미지 Mat의 크기
constexpr int IMG_FULL_SIZE = 640 * 480 * 3;

class WebSocketSender
{
public:
	WebSocketSender();
	~WebSocketSender();

	//데이터 전송
	void sendframe_via_udp(cv::InputArray frame);

	std::vector<BYTE> mat2jpg(cv::InputArray mat);
	inline bool isconnected() { return connected; }
private:
	bool connected;
	BYTE cache_idx;
	BYTE camera_id;
	WSADATA wsadata;
	SOCKET m_clientSock;
	SOCKADDR_IN m_ClientAddr;
	ADDRINFO* host_domainAddr;
	std::vector<int> encode_param = { cv::IMWRITE_JPEG_QUALITY, IMG_QUALITY };
	void set_cameraId();	
	void set_connection();
	void disconnection();
};

