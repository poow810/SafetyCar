package com.safe.safetycar.streaming.Image;

import com.safe.safetycar.log.LogManager;
import lombok.Getter;

import java.io.ByteArrayInputStream;

@Getter
public class Image {
    //미리 공간을 열어놓기 640*480 크기의 jpg를 테스트해본결과 약 60000 바이트가 나올때가 있고 20000 바이트가 될때가 있다.
    //최악의 경우를 가정해서 넉넉하게 공간을 만들어놓기

    public static short IMG_SEG_SIZE = 1469;
    public static short MAX_SEG_NUM = 150;
    public static short HEADER_SIZE = 1;     //카메라 정보를 담을 커스텀 헤더 크기

    private byte[] data = new byte[(MAX_SEG_NUM * IMG_SEG_SIZE) + HEADER_SIZE];

    public Image(byte id) {
        data[0] = id;
    }

    /**
     * 주어진 세그먼트를 입력받는다.
     * @param bis       바이트 데이터
     * @param segNum    읽을 데이터 번호
     * @return          정상적으로 읽었다면 읽은 만큼의 바이트 그렇지 않다면  -1, ByteArrayInputStream.read()와 같다.
     */
    public int write(ByteArrayInputStream bis, byte segNum) {
        return bis.read(data, (segNum * IMG_SEG_SIZE) + HEADER_SIZE, IMG_SEG_SIZE);
    }

}
