import socket
import cv2

class UdpSender:
    def __init__(self, ip, port, camera_id):
        self.server_ip = ip
        self.server_port = port  # 포트 변수 이름 수정
        self.camera_id = camera_id
        self.mtu = 1500
        self.udp_header_size = 28
        self.info_size = 3
        self.packet_size = self.mtu - self.udp_header_size
        self.img_seg_size = self.packet_size - self.info_size
        self.img_quality = 95

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    def send_frame(self, frame):
        _, encoded_frame = cv2.imencode('.jpg', frame, [int(cv2.IMWRITE_JPEG_QUALITY), self.img_quality])
        bytes_data = encoded_frame.tobytes()
        total_bytes_sent = 0
        num = 0
        img_packet_size = len(bytes_data)

        while total_bytes_sent < img_packet_size:
            # 전송할 데이터 크기 계산
            chunk_size = min(self.img_seg_size, img_packet_size - total_bytes_sent)
            buffer = bytearray(self.packet_size + self.info_size)  # 패킷 크기 + 헤더

            # 헤더 설정
            buffer[0] = 0 if total_bytes_sent + chunk_size < img_packet_size else 255  # 마지막 패킷 여부
            buffer[1] = self.camera_id  # 카메라 ID
            buffer[2] = num  # 이미지 번호

            # 데이터 복사
            buffer[3:3 + chunk_size] = bytes_data[total_bytes_sent:total_bytes_sent + chunk_size]

            # 패킷 전송
            sent_bytes = self.sock.sendto(buffer[:3 + chunk_size], (self.server_ip, self.server_port))  # 수정된 부분

            if sent_bytes == 0:
                print("Error sending packet.")
                return

            total_bytes_sent += chunk_size
            num += 1

        print(f"Packet sent: {total_bytes_sent}, Seg sent: {num}")

    def close(self):
        self.sock.close()
