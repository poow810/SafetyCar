package com.safe.safetycar.streaming.socket.controller;

import com.safe.safetycar.streaming.udp.UdpInboundMessageHandler;
import org.springframework.beans.factory.annotation.Value;
import org.springframework.http.MediaType;
import org.springframework.http.ResponseEntity;
import org.springframework.web.bind.annotation.*;

@RestController
public class FrameController {

    @Value("${react.server.address}")
    private String reactServerAddress;


    @CrossOrigin(origins = "https://j11b209.p.ssafy.io/")
    @GetMapping("/test2")
    public ResponseEntity<String> test() {
        System.out.println("TEST");
        return ResponseEntity.ok("Success");
    }

    @CrossOrigin(origins = "https://j11b209.p.ssafy.io/")
    @GetMapping(
            value = "/{cameraId}",
            produces = MediaType.IMAGE_JPEG_VALUE
    )
    public @ResponseBody byte[] getFrame(@PathVariable("cameraId") String cameraId) {
//        InputStream in = new ByteArrayInputStream(UdpInboundMessageHandler.camera_data_assembled[Integer.parseInt(cameraId)]);
        return UdpInboundMessageHandler.camera_data_assembled[Integer.parseInt(cameraId)];
    }
}
