package com.safe.safetycar.administrator.service;

import com.safe.safetycar.administrator.entity.UserConfig;
import lombok.RequiredArgsConstructor;
import org.springframework.stereotype.Service;

@Service
@RequiredArgsConstructor
public class LoginService {

    private final UserConfig userConfig;

    public boolean login(String username, String password) {
        return userConfig.getUsername().equals(username) && userConfig.getPassword().equals(password);
    }
}
