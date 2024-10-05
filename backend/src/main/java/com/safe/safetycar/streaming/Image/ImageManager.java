package com.safe.safetycar.streaming.Image;

import com.safe.safetycar.log.LogManager;
import org.springframework.stereotype.Service;

import java.io.ByteArrayInputStream;
import java.util.ArrayList;

@Service
public class ImageManager {

    private LogManager logManager = new LogManager(ImageManager.class);
    private static ArrayList<Image> images = new ArrayList<>();

    public void write(ByteArrayInputStream bis, byte cameraId, byte segNum) {
        images.get((int)cameraId).write(bis, segNum);
    }

    public Image read(byte cameraId) {
        return images.get((int)cameraId);
    }

    /**
     * 새로운 카메라의 데이터를 담을 공간을 생성하고 아이디를 발급한다.
     * @return 새로운 카메라의 아이디
     */
    public byte initCamera() {
        byte id = (byte)images.size();
        images.add(new Image(id));
        return id;
    }

    public void clear(byte cameraId) {
        images.remove((int)cameraId);
    }

    public void clearAll() {
        images.clear();
    }
}
