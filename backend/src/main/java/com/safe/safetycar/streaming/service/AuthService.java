package com.safe.safetycar.streaming.service;

import com.safe.safetycar.log.LogManager;
import org.springframework.stereotype.Service;

import java.util.Arrays;
import java.util.HashSet;

@Service
public class AuthService {

    private LogManager logManager = new LogManager(AuthService.class);
    private HashSet<String> whiteList = new HashSet<>();

    public boolean checkWhite(String ip) {
//        logManager.sendLog("try : "+ ip, LogManager.LOG_TYPE.WARN);
//        logManager.sendLog(Arrays.toString(whiteList.toArray()), LogManager.LOG_TYPE.INFO);

        return whiteList.contains(ip);
    }
    public boolean addWhite(String ip) {
        return whiteList.add(ip);
    }
}
