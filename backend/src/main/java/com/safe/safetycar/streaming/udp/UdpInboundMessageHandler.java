package com.safe.safetycar.streaming.udp;

import com.safe.safetycar.log.LogManager;
import com.safe.safetycar.streaming.Image.Image;
import com.safe.safetycar.streaming.Image.ImageManager;
import com.safe.safetycar.streaming.socket.manager.WebSocketManager;
import com.safe.safetycar.streaming.udp.filter.UDPFilter;
import org.springframework.beans.factory.annotation.Autowired;
import org.springframework.integration.annotation.MessageEndpoint;
import org.springframework.integration.annotation.ServiceActivator;
import org.springframework.messaging.Message;
import org.springframework.messaging.MessageHandler;
import org.springframework.messaging.MessagingException;
import org.springframework.messaging.handler.annotation.Headers;

import java.io.ByteArrayInputStream;
import java.io.IOException;
import java.util.Map;

@MessageEndpoint
public class UdpInboundMessageHandler {
    @Autowired
    private UDPFilter udpFilter;
    @Autowired
    private WebSocketManager wsm;
    @Autowired
    private ImageManager imageManager;

    private final static LogManager logManager = new LogManager(UdpInboundMessageHandler.class);

    //미리 공간을 열어놓기 640*480 크기의 jpg를 테스트해본결과 약 60000 바이트가 나올때가 있고 20000 바이트가 될때가 있다.
    //최악의 경우를 가정해서 넉넉하게 공간을 만들어놓기
//    final static short IMG_SEG_SIZE = 1469;
//    final static short MAX_CAMERA_NUM = 8;
//    final static short MAX_SEG_NUM = 150;
//    final static short HEADER_SIZE = 1;     //카메라 정보를 담을 커스텀 헤더 크기
//
//    public static byte[][] camera_data_assembled = new byte[MAX_CAMERA_NUM][(MAX_SEG_NUM * IMG_SEG_SIZE) + HEADER_SIZE];

    public UdpInboundMessageHandler() {
        logManager.setInterval(LogManager.LOG_TYPE.INFO, 100, "image received");
        
        
        // 카메라 번호 할당
//        for(byte i = 0; i < MAX_CAMERA_NUM; i++) {
//            camera_data_assembled[i][0] = i;
//        }
    }

    //headerMap 내용 example
    //{ip_packetAddress=/127.0.0.1:58011, ip_address=127.0.0.1, id=6626e9b4-fac2-e7d2-d2a0-afd7ce5fa366, ip_port=58011, ip_hostname=127.0.0.1, timestamp=1727833051957}
    @ServiceActivator(inputChannel = "inboundChannel")
    public void handleMessage(Message message, @Headers Map<String, Object> headerMap) throws IOException {
//        System.out.println(headerMap.toString());
        if(!udpFilter.accept(message)) {
            logManager.sendLog("ACCESS DENIED", LogManager.LOG_TYPE.WARN);
            return;
        }
        ByteArrayInputStream bis = new ByteArrayInputStream((byte[])message.getPayload());
        //endflag를 byte로 받으면 아래 if문에서 정상적으로 식별이 되지 않는다. 카메라에서 255(모든 비트를 1)으로 설정하고 전송하는데 if문에서 음수로 판단하는 것 같다...
        //c++에서는 BYTE는 unsigned char(0 ~ 255) 으로 선언되어있다. 하지만 자바에서는 byte의 표현범위가 -127 ~ 128이기 때문에 음수로 인식되어 아래 if문에서 항상 거짓이게 된다...
        //해결 방법은 다음과 같다.
        //1. int형으로 변환하면 원하는 값을 얻을 수 있다.
        //2. 음수로 인식되는 범위라면 부호 비트를 반전하기
        //3. char 자료형을 사용하기
        // 가장 간단한 1번을 사용하기로 하였다.
        int endflag = bis.read();
        byte cameraId = (byte)bis.read();
        byte segNum = (byte)bis.read();

        if(segNum >= Image.MAX_SEG_NUM) {
            logManager.sendLog("segNum is greater than MAX_SEG_NUM", LogManager.LOG_TYPE.ERROR);
            return;
        }
//        bis.read(camera_data_assembled[cameraId], (segNum * IMG_SEG_SIZE) + HEADER_SIZE, IMG_SEG_SIZE);
        imageManager.write(bis, cameraId, segNum);

        if(endflag > 0){
            logManager.sendLog("Received", LogManager.LOG_TYPE.INFO);
            logManager.sendInterval();
            wsm.sendFrame(cameraId);
        }
    }
}
