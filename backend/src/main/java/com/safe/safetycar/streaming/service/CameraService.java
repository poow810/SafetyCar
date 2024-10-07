package com.safe.safetycar.streaming.service;

import com.safe.safetycar.log.LogManager;
import com.safe.safetycar.streaming.Image.ImageManager;
import lombok.AllArgsConstructor;
import lombok.Getter;
import org.springframework.beans.factory.annotation.Autowired;
import org.springframework.stereotype.Service;

import java.util.HashMap;
import java.util.HashSet;
import java.util.concurrent.ConcurrentHashMap;

@Service
public class CameraService {

    private LogManager logManager = new LogManager(CameraService.class);
    private ConcurrentHashMap<String, Byte> whiteList = new ConcurrentHashMap<>();
    @Autowired
    private ImageManager imageManager;

    public boolean checkWhite(String ip) {
//        logManager.sendLog("try : "+ ip, LogManager.LOG_TYPE.WARN);
//        logManager.sendLog(Arrays.toString(whiteList.toArray()), LogManager.LOG_TYPE.INFO);

        return whiteList.containsKey(ip);
    }

    /**
     * 카메라를 등록한다. 새로운 카메라라면 아이디를 발급하여 반환하고 새로운 카메라를 위한 공간을 할당한다.
     * @param ip    카메라 아이피
     * @return      생성된 카메라의 번호
     */
    public byte addCamera(String ip) {
        if(!whiteList.containsKey(ip)) {
            whiteList.put(ip, imageManager.initCamera());   //카메라를 위한 공간 할당 및 번호 등록
        } else logManager.sendLog("Camera Already Registered!", LogManager.LOG_TYPE.WARN);

        return whiteList.get(ip);
    }

    public boolean removeCamera(String ip) {
        return whiteList.remove(ip) != null;
    }

}
