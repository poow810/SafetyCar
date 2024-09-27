package com.safe.safetycar.streaming.udp.dto;

import lombok.AllArgsConstructor;

@AllArgsConstructor
public class ImagePacket {
    private byte[][] data;
    private byte counter;
    private byte full_seg_num;
    private boolean end_flag;
    private byte beginIdx;

    void flush() {
        counter = 0;
        full_seg_num = -1;
        end_flag = false;
    }

    /**
     * MTU 단위(+ UDP Header)로 나뉜 세그먼트를 담을 배열을 반환한다.
     * @param segNum 세그먼트 번호
     * @return 세그먼트 번호의 배열
     */
    public byte[] getData(int segNum) {
        return data[segNum];
    }

    /**
     * 세그먼트가 입력된 횟수를 카운트한다.
     */
    public void incrementCounter() { counter++; }

    public boolean isFull() {
        return false;
    }


}
