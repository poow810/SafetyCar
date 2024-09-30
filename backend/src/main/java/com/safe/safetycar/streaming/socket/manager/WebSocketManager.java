package com.safe.safetycar.streaming.socket.manager;

import com.safe.safetycar.log.LogManager;
import com.safe.safetycar.streaming.udp.UdpInboundMessageHandler;
import jakarta.websocket.OnClose;
import jakarta.websocket.OnOpen;
import jakarta.websocket.Session;
import jakarta.websocket.server.ServerEndpoint;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import org.springframework.stereotype.Service;

import java.io.IOException;
import java.nio.ByteBuffer;
import java.util.Collections;
import java.util.HashSet;
import java.util.Set;

@ServerEndpoint(value = "/socket")
@Service
public class WebSocketManager {
//    private final static Logger LOGGER = LoggerFactory.getLogger(UdpInboundMessageHandler.class);
    private final static LogManager LOGGER = new LogManager(WebSocketManager.class);
    private static Set<Session> CLIENTS = Collections.synchronizedSet(new HashSet<>());


    @OnOpen
    public void onOpen(Session session) {
        LOGGER.sendLog(session.toString(), LogManager.LOG_TYPE.INFO);

        if(CLIENTS.contains(session)) {
            LOGGER.sendLog("already connected", LogManager.LOG_TYPE.INFO);
        } else {
            CLIENTS.add(session);
            LOGGER.sendLog("new Session", LogManager.LOG_TYPE.INFO);
        }
    }

    @OnClose
    public void onClose(Session session) throws Exception {
        CLIENTS.remove(session);
        LOGGER.sendLog("Closed : " + session, LogManager.LOG_TYPE.INFO);
    }

    public void sendFrame(int cameraId) throws IOException {
//        LOGGER.info("sending Frame");

        for(Session client : CLIENTS) {
            try {
                synchronized (client) {
                    client.getBasicRemote().sendBinary(ByteBuffer.wrap(UdpInboundMessageHandler.camera_data_assembled[cameraId]));
                }
            } catch (IOException e) {
                e.printStackTrace();
            }
        }
    }
}
