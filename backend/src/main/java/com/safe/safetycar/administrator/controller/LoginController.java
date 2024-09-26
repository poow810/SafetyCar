package com.safe.safetycar.administrator.controller;

import com.safe.safetycar.administrator.service.LoginService;
import lombok.RequiredArgsConstructor;
import org.springframework.http.HttpStatus;
import org.springframework.http.ResponseEntity;
import org.springframework.web.bind.annotation.PostMapping;
import org.springframework.web.bind.annotation.RequestParam;
import org.springframework.web.bind.annotation.RestController;


@RestController
@RequiredArgsConstructor
public class LoginController {

    private LoginService loginService;

    @PostMapping("login")
    public ResponseEntity<Void> login(@RequestParam String username, @RequestParam String password) {
        if (loginService.login(username, password)) {
            return ResponseEntity.status(HttpStatus.OK).build();
        } else {
            return ResponseEntity.status(HttpStatus.UNAUTHORIZED).build();
        }
    }
}
