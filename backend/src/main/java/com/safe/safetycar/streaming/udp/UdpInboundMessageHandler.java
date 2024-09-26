package com.safe.safetycar.streaming.udp;

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
    private final static Logger LOGGER = LoggerFactory.getLogger(UdpInboundMessageHandler.class);
    
    //미리 공간을 열어놓기 640*480 크기의 jpg를 테스트해본결과 약 60000 바이트가 나올때가 있고 20000만 바이트가 될때가 있다.
    //최악의 경우를 가정해서 넉넉하게 공간을 만들어놓기
    final static short IMG_SEG_SIZE = 1469;
    final static short MAX_CAMERA_NUM = 8;
    final static short MAX_SEG_NUM = 70;
    public static byte[][][] camera_datas = new byte[MAX_CAMERA_NUM][MAX_SEG_NUM][IMG_SEG_SIZE];
    public static byte[][] camera_data_assembled = new byte[MAX_CAMERA_NUM][];

    @ServiceActivator(inputChannel = "inboundChannel")
    public void handeMessage(Message message, @Headers Map<String, Object> headerMap) throws IOException {

        ByteArrayInputStream bis = new ByteArrayInputStream((byte[])message.getPayload());
        int endflag = bis.read();
        int cameraId = bis.read();
        int segNum = bis.read();
        bis.read(camera_datas[cameraId][segNum]);

//        LOGGER.info("seg received : " + segNum);
        if(endflag > 0) {
            byte[] img_bytes = new byte[IMG_SEG_SIZE * (segNum + 1)];

            for(int i = 0; i <= segNum; i++) {
                System.arraycopy(camera_datas[cameraId][i], 0, img_bytes, i * IMG_SEG_SIZE, IMG_SEG_SIZE);
            }

            camera_data_assembled[cameraId] = img_bytes;
//            ByteArrayInputStream img_bis = new ByteArrayInputStream(img_bytes);
//            BufferedImage image = ImageIO.read(img_bis);
//            ImageIO.write(image, "jpg", new java.io.File("received_image.jpg"));
            LOGGER.info("JPG received");
            wsm.sendFrame(cameraId);
        }

    }
}
