package com.safe.safetycar.streaming.response;

import com.safe.safetycar.response_template.BaseResponseTemplate;
import lombok.*;

@Getter
@Setter
public class AuthResponse extends BaseResponseTemplate {
    int camera_id;

    public AuthResponse(int camera_id, String message, int status_code) {
        super(status_code, message);
        this.camera_id = camera_id;
    }
}
