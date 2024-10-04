package com.safe.safetycar.streaming.udp.filter;

import com.safe.safetycar.log.LogManager;
import com.safe.safetycar.streaming.service.AuthService;
import lombok.extern.slf4j.Slf4j;
import org.springframework.beans.factory.annotation.Autowired;
import org.springframework.integration.annotation.Filter;
import org.springframework.integration.core.MessageSelector;
import org.springframework.messaging.Message;
import org.springframework.stereotype.Service;

@Service
public class UDPFilter implements MessageSelector {

    @Autowired
    private AuthService authService;

    private LogManager logManager = new LogManager(UDPFilter.class);

    @Override
    public boolean accept(Message<?> message) {
//        logManager.sendLog(message.toString(), LogManager.LOG_TYPE.INFO);

        return authService.checkWhite((String)message.getHeaders().get("ip_address"));
    }
}
