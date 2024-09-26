package com.safe.safetycar.administrator.entity;


import lombok.Getter;
import org.springframework.boot.context.properties.ConfigurationProperties;
import org.springframework.stereotype.Component;

@Component
@ConfigurationProperties(prefix = "app.user")
@Getter
public class UserConfig {

    private String username;
    private String password;
}
