package com.safe.safetycar.streaming.udp;

import com.safe.safetycar.log.LogManager;
import com.safe.safetycar.streaming.socket.manager.WebSocketManager;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import org.springframework.beans.factory.annotation.Autowired;
import org.springframework.integration.annotation.MessageEndpoint;
import org.springframework.integration.annotation.ServiceActivator;
import org.springframework.messaging.Message;
import org.springframework.messaging.handler.annotation.Headers;

import java.io.ByteArrayInputStream;
import java.io.IOException;
import java.util.Map;

@MessageEndpoint
public class UdpInboundMessageHandler {

    @Autowired
    private WebSocketManager wsm;
//    private final static Logger LOGGER = LoggerFactory.getLogger(UdpInboundMessageHandler.class);
    private final static LogManager logManager = new LogManager(UdpInboundMessageHandler.class);

    //미리 공간을 열어놓기 640*480 크기의 jpg를 테스트해본결과 약 60000 바이트가 나올때가 있고 20000 바이트가 될때가 있다.
    //최악의 경우를 가정해서 넉넉하게 공간을 만들어놓기
    final static short IMG_SEG_SIZE = 1469;
    final static short MAX_CAMERA_NUM = 8;
    final static short MAX_SEG_NUM = 150;
    final static short HEADER_SIZE = 1;     //카메라 정보를 담을 커스텀 헤더 크기
//    public static byte[][][] camera_datas = new byte[MAX_CAMERA_NUM][MAX_SEG_NUM][IMG_SEG_SIZE];
//    public static byte[][] camera_data_assembled2 = new byte[MAX_CAMERA_NUM][];

    public static byte[][] camera_data_assembled = new byte[MAX_CAMERA_NUM][(MAX_SEG_NUM * IMG_SEG_SIZE) + HEADER_SIZE];

    public UdpInboundMessageHandler() {
        logManager.setInterval(LogManager.LOG_TYPE.INFO, 100, "image received");
        
        
        // 카메라 번호 할당
        for(byte i = 0; i < MAX_CAMERA_NUM; i++) {
            camera_data_assembled[i][0] = i;
        }
    }

    @ServiceActivator(inputChannel = "inboundChannel")
    public void handeMessage(Message message, @Headers Map<String, Object> headerMap) throws IOException {

        ByteArrayInputStream bis = new ByteArrayInputStream((byte[])message.getPayload());
        int endflag = bis.read();
        int cameraId = bis.read();
        int segNum = bis.read();
//        bis.read(camera_datas[cameraId][segNum]);
//
////        LOGGER.info("seg received : " + segNum);
//        if(endflag > 0) {
//            byte[] img_bytes = new byte[IMG_SEG_SIZE * (segNum + 1)];
//
//            for(int i = 0; i <= segNum; i++) {
//                System.arraycopy(camera_datas[cameraId][i], 0, img_bytes, i * IMG_SEG_SIZE, IMG_SEG_SIZE);
//            }
//
//            camera_data_assembled[cameraId] = img_bytes;
////
//            LOGGER.info("JPG received");
//            wsm.sendFrame(cameraId);
//        }

        if(segNum >= MAX_SEG_NUM) {
            logManager.sendLog("segNum is greater than MAX_SEG_NUM", LogManager.LOG_TYPE.ERROR);
            return;
        }
        bis.read(camera_data_assembled[cameraId], (segNum * IMG_SEG_SIZE) + HEADER_SIZE, IMG_SEG_SIZE);
        if(endflag > 0){
            logManager.sendInterval();
//            logManager.sendLog("image Received", LogManager.LOG_TYPE.INFO);
            wsm.sendFrame(cameraId);
        }
    }
}
