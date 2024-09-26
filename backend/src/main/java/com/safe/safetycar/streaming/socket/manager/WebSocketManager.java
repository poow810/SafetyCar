package com.safe.safetycar.streaming.socket.manager;

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
    private final static Logger LOGGER = LoggerFactory.getLogger(UdpInboundMessageHandler.class);
    private static Set<Session> CLIENTS = Collections.synchronizedSet(new HashSet<>());


    @OnOpen
    public void onOpen(Session session) {
        LOGGER.info(session.toString());

        if(CLIENTS.contains(session)) {
            LOGGER.info("already connected");
        } else {
            CLIENTS.add(session);
            LOGGER.info("new Session");
        }
    }

    @OnClose
    public void onClose(Session session) throws Exception {
        CLIENTS.remove(session);
        System.out.println("Closed : " + session);
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
