package com.safe.safetycar.streaming.Image;

import com.safe.safetycar.log.LogManager;
import org.springframework.stereotype.Service;

import java.io.ByteArrayInputStream;
import java.util.ArrayList;

@Service
public class ImageManager {

    private LogManager logManager = new LogManager(ImageManager.class);
    private static ArrayList<Image> images = new ArrayList<>();
    private static int recyclableCount = 0;

    public void write(ByteArrayInputStream bis, byte cameraId, byte segNum) {
        images.get(cameraId).write(bis, segNum);
    }

    public Image read(byte cameraId) {
        return images.get(cameraId);
    }

    /**
     * 새로운 카메라의 데이터를 담을 공간을 생성하고 아이디를 발급한다. 만약 사용하지 않는 공간이 있다면 (이전 카메라 연결이 종료되었을경우) 해당 공간을 재사용한다.
     * @return 새로운 카메라의 아이디
     */
    public byte initCamera() {
        //재사용 가능한 공간이 없다면 새로운 공간 할당
        if(recyclableCount == 0) {
            byte id = (byte)images.size();
            images.add(new Image(id));
            return id;
        } else {
            //가능하다면 공간을 재사용한다.
            byte id = 0;
            while(id < images.size() && images.get(id).isOpen()) {
                id++;
            }
            recyclableCount--;
            images.get(id).setOpen(true);
            return id;
        }

    }

    public int getFlag(byte cameraId) {
        return images.get(cameraId).getFlag();
    }
    public void setFlag(byte cameraId, int flag) {
        images.get(cameraId).setFlag(flag);
    }

    public void remove(int idx) {
        images.get(idx).setOpen(false);
        recyclableCount++;
    }

    public void clear(byte cameraId) {
        images.remove(cameraId);
    }

    public void clearAll() {
        images.clear();
    }
}
