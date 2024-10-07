package com.safe.safetycar.streaming.controller;

import com.safe.safetycar.log.LogManager;
import com.safe.safetycar.streaming.request.AuthRequest;
import com.safe.safetycar.streaming.response.AuthResponse;
import com.safe.safetycar.streaming.response.DisconnectResponse;
import com.safe.safetycar.streaming.service.CameraService;
import jakarta.servlet.http.HttpServletRequest;
import org.springframework.beans.factory.annotation.Autowired;
import org.springframework.http.HttpStatus;
import org.springframework.http.ResponseEntity;
import org.springframework.web.bind.annotation.*;

import java.util.HashSet;

@RestController
public class CameraController {

    private final static LogManager logManager = new LogManager(CameraController.class);
    @Autowired
    private CameraService cameraService;

    private String[] headerTypes = {"X-Forwarded-For", "Proxy-Client-IP", "WL-Proxy-Client-IP", "HTTP_CLIENT_IP", "HTTP_X_FORWARDED_FOR"};
    private static HashSet<String> whiteList = new HashSet<>();

    @GetMapping ("/headertest")
    public ResponseEntity test(HttpServletRequest request) {
        String ip = null;
        for(String headerType: headerTypes) {
            ip = request.getHeader(headerType);
            if(ip != null) break;
        }

        // 적용
        if (ip == null) ip = request.getRemoteAddr();
//        System.out.println("Real Remote(Client) IP Address: " + ip);
        logManager.sendLog("Real Remote(Client) IP Address: " + ip, LogManager.LOG_TYPE.INFO);
        return new ResponseEntity<>("Success", HttpStatus.CREATED);
    }

    /**
     * 카메라 연결을 요청
     * 새로운 카메라의 요청일 경우 공간 할당 후 카메라 아이디 부여
     * 기존 카메라인 경우 기존 아이디 반환
     * @param request
     * @param authRequest       Token
     * @return 카메라 아이디가 담긴 응답
     */
    @PostMapping("/connect")
    public ResponseEntity<AuthResponse> setWhitelist(HttpServletRequest request, @RequestBody AuthRequest authRequest) {
        String ip = null;
        for(String headerType: headerTypes) {
            ip = request.getHeader(headerType);
            if(ip != null) break;
        }
        if (ip == null) ip = request.getRemoteAddr();
        logManager.sendLog("CCTV Connected : " + ip, LogManager.LOG_TYPE.INFO);
    
//        if(!cameraService.addCamera(ip)) {
//            logManager.sendLog("Already Connected", LogManager.LOG_TYPE.INFO);
//            return new ResponseEntity<>(new AuthResponse(0, "Already Connected", 222), HttpStatus.OK);
//        }
        byte cameraId = cameraService.addCamera(ip);

        return new ResponseEntity<>(new AuthResponse(cameraId, "Success", 200), HttpStatus.OK);
    }

    @PostMapping("/disconnect/{cameraId}")
    public ResponseEntity<DisconnectResponse> removeWhitelist(HttpServletRequest request, @PathVariable String cameraId, @RequestBody AuthRequest authRequest) {
        String ip = null;
        for(String headerType: headerTypes) {
            ip = request.getHeader(headerType);
            if(ip != null) break;
        }
        if (ip == null) ip = request.getRemoteAddr();
        logManager.sendLog("CCTV Disconnect : " + ip, LogManager.LOG_TYPE.INFO);

        return cameraService.removeCamera(cameraId) ?
            new ResponseEntity<>(new DisconnectResponse("Disconnected", 200), HttpStatus.OK)
                : new ResponseEntity<>(new DisconnectResponse("Already Disconnected", 400), HttpStatus.BAD_REQUEST);
    }

}