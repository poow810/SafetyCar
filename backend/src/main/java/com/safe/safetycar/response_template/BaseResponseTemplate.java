package com.safe.safetycar.response_template;

import lombok.*;

@Data
@AllArgsConstructor
@NoArgsConstructor
public class BaseResponseTemplate {
    int status_code;
    String message;
}
