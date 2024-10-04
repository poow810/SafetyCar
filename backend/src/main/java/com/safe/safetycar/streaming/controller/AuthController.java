package com.safe.safetycar.streaming.controller;

import com.safe.safetycar.log.LogManager;
import com.safe.safetycar.streaming.request.AuthRequest;
import com.safe.safetycar.streaming.response.AuthResponse;
import com.safe.safetycar.streaming.service.AuthService;
import com.safe.safetycar.streaming.udp.UdpInboundMessageHandler;
import jakarta.servlet.http.HttpServletRequest;
import org.springframework.beans.factory.annotation.Autowired;
import org.springframework.http.HttpRequest;
import org.springframework.http.HttpStatus;
import org.springframework.http.ResponseEntity;
import org.springframework.web.bind.annotation.*;

import java.util.HashSet;
import java.util.Map;

@RestController
public class AuthController {

    private final static LogManager logManager = new LogManager(AuthController.class);
    @Autowired
    private AuthService authService;

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

    @PostMapping("/connect")
    public ResponseEntity<AuthResponse> setWhitelist(HttpServletRequest request, @RequestBody AuthRequest authRequest) {
        //TODO : authRequst 에서 토큰 검사하기
        
        String ip = null;
        for(String headerType: headerTypes) {
            ip = request.getHeader(headerType);
            if(ip != null) break;
        }
        if (ip == null) ip = request.getRemoteAddr();
        logManager.sendLog("CCTV Connected : " + ip, LogManager.LOG_TYPE.INFO);
    
        if(!authService.addWhite(ip)) {
            logManager.sendLog("Already Connected", LogManager.LOG_TYPE.INFO);
        }

        return new ResponseEntity<>(new AuthResponse(0, "success", 200), HttpStatus.OK);
    }

}
