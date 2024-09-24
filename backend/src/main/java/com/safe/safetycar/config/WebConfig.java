package com.safe.safetycar.config;

import org.springframework.context.annotation.Configuration;
import org.springframework.web.servlet.config.annotation.CorsRegistry;
import org.springframework.web.servlet.config.annotation.WebMvcConfigurer;

@Configuration
public class WebConfig implements WebMvcConfigurer {

    @Override
    public void addCorsMappings(CorsRegistry registry) {
        registry.addMapping("/sse")
                .allowedOrigins("http://localhost:3000") // 리액트 앱의 주소
                .allowedMethods("GET", "POST")
                .allowCredentials(true);
    }
}