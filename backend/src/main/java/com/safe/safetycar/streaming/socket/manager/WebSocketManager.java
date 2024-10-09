package com.safe.safetycar.streaming.socket.manager;

import com.safe.safetycar.log.LogManager;
import com.safe.safetycar.streaming.Image.ImageManager;
import com.safe.safetycar.streaming.udp.UdpInboundMessageHandler;
import jakarta.websocket.OnClose;
import jakarta.websocket.OnOpen;
import jakarta.websocket.Session;
import jakarta.websocket.server.ServerEndpoint;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import org.springframework.beans.factory.annotation.Autowired;
import org.springframework.scheduling.annotation.Async;
import org.springframework.stereotype.Service;
import org.springframework.web.socket.WebSocketSession;
import org.springframework.web.socket.handler.ConcurrentWebSocketSessionDecorator;

import java.io.IOException;
import java.nio.ByteBuffer;
import java.util.Collections;
import java.util.HashSet;
import java.util.Set;
import java.util.concurrent.ConcurrentHashMap;

@ServerEndpoint(value = "/socket")
@Service
public class WebSocketManager {
//    private final static Logger LOGGER = LoggerFactory.getLogger(UdpInboundMessageHandler.class);
    private final static LogManager LOGGER = new LogManager(WebSocketManager.class);
    private static Set<Session> CLIENTS = ConcurrentHashMap.newKeySet();
//    private static Set<Session> CLIENTS = Collections.synchronizedSet(new HashSet<>());

    @Autowired
    private ImageManager imageManager;

    @OnOpen
    public void onOpen(Session session) {
        LOGGER.sendLog(session.toString(), LogManager.LOG_TYPE.INFO);

        if(CLIENTS.contains(session)) {
            LOGGER.sendLog("already connected", LogManager.LOG_TYPE.INFO);
        } else {
            CLIENTS.add(session);
            LOGGER.sendLog("new Session", LogManager.LOG_TYPE.INFO);
        }
        LOGGER.sendLog("Clients Num : " + CLIENTS.size(), LogManager.LOG_TYPE.INFO);
    }

    @OnClose
    public void onClose(Session session) throws Exception {
        CLIENTS.remove(session);
        LOGGER.sendLog("Closed : " + session, LogManager.LOG_TYPE.INFO);
        LOGGER.sendLog("Clients Num : " + CLIENTS.size(), LogManager.LOG_TYPE.INFO);

    }

    public void sendFrame(byte cameraId) throws IOException {
//        LOGGER.info("sending Frame");

        for(Session client : CLIENTS) {
            try {
                if(!client.isOpen()) {
                    onClose(client);
                    continue;
                }
                synchronized (client) {
//                    client.getBasicRemote().sendBinary(ByteBuffer.wrap(UdpInboundMessageHandler.camera_data_assembled[cameraId]));
//                    client.getBasicRemote().sendBinary(ByteBuffer.wrap(imageManager.read(cameraId).getPrevData()));
                    client.getBasicRemote().sendBinary(ByteBuffer.wrap(imageManager.read(cameraId).getCurrentData()));
//                    sendFrame2Client(client, cameraId);
                }
            } catch (IOException e) {
                e.printStackTrace();
            } catch (Exception e) {
                throw new RuntimeException(e);
            }
        }
    }

//    @Async
    public void sendFrame2Client(Session client, byte cameraId) throws IOException {
        client.getBasicRemote().sendBinary(ByteBuffer.wrap(imageManager.read(cameraId).getPrevData()));
//        client.getAsyncRemote().sendBinary(ByteBuffer.wrap(imageManager.read(cameraId).getData()));
//        ConcurrentWebSocketSessionDecorator c = new ConcurrentWebSocketSessionDecorator(client);
//        client.getAsyncRemote().sendBinary(ByteBuffer.wrap(imageManager.read(cameraId).getData()));
    }

    public int getClientSize() {
        return CLIENTS.size();
    }
}
