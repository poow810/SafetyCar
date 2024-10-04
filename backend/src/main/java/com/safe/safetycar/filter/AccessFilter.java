package com.safe.safetycar.filter;

import com.safe.safetycar.log.LogManager;
import jakarta.servlet.*;
import lombok.extern.slf4j.Slf4j;
import org.springframework.beans.factory.annotation.Value;

import java.io.IOException;

@Slf4j
public class AccessFilter implements Filter {

    private static LogManager logManager = new LogManager(AccessFilter.class);

    @Value("${cctv.client.token}")
    private String TOKEN;

    @Override
    public void doFilter(ServletRequest servletRequest, ServletResponse servletResponse, FilterChain filterChain) throws IOException, ServletException {
        logManager.sendLog("DO Filter!", LogManager.LOG_TYPE.INFO);

        //TODO : token check

        filterChain.doFilter(servletRequest, servletResponse);
    }
}
