package com.safe.safetycar.streaming.controller;

import com.safe.safetycar.log.LogManager;
import com.safe.safetycar.streaming.udp.UdpInboundMessageHandler;
import jakarta.servlet.http.HttpServletRequest;
import org.springframework.http.HttpRequest;
import org.springframework.http.HttpStatus;
import org.springframework.http.ResponseEntity;
import org.springframework.web.bind.annotation.GetMapping;
import org.springframework.web.bind.annotation.PostMapping;
import org.springframework.web.bind.annotation.RequestHeader;
import org.springframework.web.bind.annotation.RestController;

import java.util.Map;

@RestController
public class AuthController {

    private final static LogManager logManager = new LogManager(AuthController.class);

    private String[] headerTypes = {"X-Forwarded-For", "Proxy-Client-IP",
            "WL-Proxy-Client-IP", "HTTP_CLIENT_IP", "HTTP_X_FORWARDED_FOR"};


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
}
