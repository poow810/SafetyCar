package com.safe.safetycar.streaming.udp;

import com.safe.safetycar.streaming.udp.dto.ImagePacket;
import org.springframework.stereotype.Component;

@Component
public class ImagePacketManager {
    final short IMG_SEG_SIZE = 1469;
    final short MAX_CAMERA_NUM = 8;
    final short MAX_SEG_NUM = 90;
    final short MAX_CACHE_FRAME = 5;

    ImagePacket[] imagePackets;




}
