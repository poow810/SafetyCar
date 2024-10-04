package com.safe.safetycar.streaming.service;

import org.springframework.stereotype.Service;

import java.util.HashSet;

@Service
public class AuthService {

    private static HashSet<String> whiteList = new HashSet<>();

    public boolean checkWhite(String ip) {
        return whiteList.contains(ip);
    }
    public boolean addWhite(String ip) {
        return whiteList.add(ip);
    }
}
