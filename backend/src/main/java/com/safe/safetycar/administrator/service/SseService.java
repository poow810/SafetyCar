package com.safe.safetycar.administrator.service;

import org.springframework.stereotype.Service;
import org.springframework.web.servlet.mvc.method.annotation.SseEmitter;

import java.io.IOException;
import java.util.Map;
import java.util.concurrent.ConcurrentHashMap;

@Service
public class SseService {
    private final Map<String, SseEmitter> emitters = new ConcurrentHashMap<>();

    public SseEmitter subscribe(String clientId) {
        SseEmitter emitter = new SseEmitter();

        // 초기 메시지 전송 (더미 데이터)
        try {
            emitter.send("연결이 성공적으로 이루어졌습니다.");
        } catch (IOException e) {
            emitter.completeWithError(e);
        }

        emitters.put(clientId, emitter);

        emitter.onCompletion(() -> emitters.remove(clientId));
        emitter.onTimeout(() -> emitters.remove(clientId));

        return emitter;
    }

    public void sendCoordinate(float x, float y) {
        for (Map.Entry<String, SseEmitter> entry : emitters.entrySet()) {
            SseEmitter emitter = entry.getValue();
            try {
                emitter.send("좌표 수신: x = " + x + ", y = " + y);
            } catch (IOException e) {
                emitters.remove(entry.getKey());
            }
        }
    }
}