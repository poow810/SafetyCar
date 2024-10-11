# 목차
1. 사용 도구
2. 개발 도구
3. 개발 환경
4. 환경 변수 형태
5. CI/CD 구축


# 1. 사용 도구
- 이슈 관리 : Jira
- 형상 관리 : GitLab, Git Bash
- 커뮤니케이션 : Notion, MatterMost
- 디자인 : Figma
- CI/CD : Jenkins

# 2. 개발 도구
- Visual Studio Code
- IntelliJ
- Visual Studio 2022
- Pycharm


# 3. 개발 환경
### Frontend 
- React: 18.3.1   
- Node.js: 16   
- Socket.io: 4.7.5

---
### Backend
- JDK: 17  
- Spring Boot: 3.3.4  
- Python: 3.9  
- Fast API

---
### CCTV
- Python: 3.9  
- OpenCV: 4.10.0  
- Yolo v8  
- Yolo v8 Pose model  

#### 영상 전송 테스트
- C++  
- Visual Studio Build tools 2019  
- vcpkg: 2024.09.30  

---
### Server    
- AWS EC2  
- CPU: Intel(R) Xeon(R) CPU E5-2686 v4 @ 2.30Gz(4 Core, 4 thread)  
- Disk: 311GB  
- Ram: 16GB

---

### Service
- nginx: 1.26.2    
- jenkins: 2.462.2     
- Docker: 27.2.1  
- Docker-compose: 2.29.2  
- Redis: 5.0.8

# 4. 환경 변수 형태
### Backend
- application.yml
    ```
    udp:
        channel: inboundChannel
        task-executor:
            queue-capacity: '40'
            max-pool-size: '100'
            core-pool-size: '10'
        port: '5432'
    spring:
        application:
            name: safetycar
    server:
        servlet:
            context-path: /api
    react:
        server:
            address: https://j11b209.p.ssafy.io/

    cctv:
        client:
            token: 1234 # 임시
        save:
            path: /save/cam/list.xml

    coolsms:
        apikey: 'NCSRLFJOYQMYPRKG'
        apisecret: 'IWBDNHHYVW7JEOICTAZTDAVF3DIOCCM6'
        fromnumber: '01030479192'
    ```

### Frontend
- .env  

  ```
    # 기본 API URL
    REACT_APP_API_URL = https://j11b209.p.ssafy.io/api
    REACT_APP_WEBSOCKET_URL = wss://j11b209.p.ssafy.io/api/socket
    REACT_APP_PYTHON_URL = https://j11b209.p.ssafy.io/pyapi 
  ```
---

# 5. CI/CD 구축
### 1. UFW 방화벽 설정
```
sudo apt-get update

sudo ufw status
sudo ufw allow 22
sudo ufw allow 80
sudo ufw allow 443
sudo ufw enable
sudo ufw status numbered
```
### 2. Docker 설치
```
# Add Docker's official GPG key:
sudo apt-get update
sudo apt-get install ca-certificates curl
sudo install -m 0755 -d /etc/apt/keyrings
sudo curl -fsSL https://download.docker.com/linux/ubuntu/gpg -o /etc/apt/keyrings/docker.asc
sudo chmod a+r /etc/apt/keyrings/docker.asc

# Add the repository to Apt sources:
echo \
  "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/docker.asc] https://download.docker.com/linux/ubuntu \
  $(. /etc/os-release && echo "$VERSION_CODENAME") stable" | \
  sudo tee /etc/apt/sources.list.d/docker.list > /dev/null
sudo apt-get update

sudo apt-get install docker-ce docker-ce-cli containerd.io docker-buildx-plugin docker-compose-plugin
```
* 다음 명령어로 설치 확인
```
sudo docker run hello-world
```


### 3. Docker-compose 설치
```
sudo curl -SL https://github.com/docker/compose/releases/download/v2.29.2/docker-compose-linux-x86_64 -o /usr/local/bin/docker-compose
sudo chmod +x /usr/local/bin/docker-compose
sudo ln -s /usr/local/bin/docker-compose /usr/bin/docker-compose
```
* 설치 확인
```
docker-compose -v
```
### 4. jenkins 설치
* docker-compose.yml
```
services:
  jenkins:
    image: jenkins/jenkins:lts
    restart: unless-stopped
    container_name: compose-jenkins
    user: root
    volumes:
      # jenkins 제거시에도 관련 설정 유지
      - ./jenkins:/var/jenkins_home
      # DooD 구성을 위한 도커 소켓 연결
      - /var/run/docker.sock:/var/run/docker.sock
      - /usr/local/bin/docker-compose:/usr/local/bin/docker-compose
      - /usr/bin/docker:/usr/bin/docker
      # 데이터베이스 마운트
      - /home/ubuntu/DataBase:/shared
    environment:
      # port 번호 설정
      JENKINS_OPTS: --prefix=/jenkins --httpPort=9090
    logging:
      driver: "json-file"
      options:
        max-size: "10m"
        max-file: "1"
    networks:
      - jenkins-network
    expose:
      - "9090"
      - "50000"

networks:
  jenkins-network:
    external: true
```
* 컨테이너 실행
```
sudo docker-compose up -d
```
### 5. Nginx 설치
* Dockerfile
```
FROM nginx:1.26.2-alpine

RUN apk add python3 python3-dev py3-pip build-base libressl-dev musl-dev libffi-dev rust cargo
RUN apk add certbot-nginx
RUN mkdir /etc/letsencrypt
``` 

* docker-compose.yml
```
services:
  nginx:
    image: nginx:custom
    restart: unless-stopped
    container_name: compose-nginx
    volumes:
      - ./proxy:/etc/nginx/conf.d
      - ./nginx.conf:/etc/nginx/nginx.conf
      - ./cert:/etc/ssl
      - /etc/letsencrypt:/etc/letsencrypt
      - ./validation:/var/validation
    networks:
      - jenkins-network
      - backend-network
      - frontend-network
    ports:
      - "80:80"
      - "443:443"
      - "5432:5432/udp"

networks:
  jenkins-network:
    external: true

  backend-network:
    external: true

  frontend-network:
    external: true
```

* default.conf
```
upstream jenkins {
  server compose-jenkins:9090;
}
upstream backend {
  server safetycar-dev-backend:8080;
}
upstream frontend {
  server safetycar-dev-frontend:80;
}
upstream socketserver {
  server safetycar-dev-backend:8080;
}
upstream pybackend {
  server safetycar-dev-pybackend:8000;
}
upstream pysocketio {
  server safetycar-dev-pybackend:8000;
}

server {
    listen       80;
    listen  [::]:80;
    server_name  j11b209.p.ssafy.io;

    location / {
        #root   /usr/share/nginx/html;
        #index  index.html index.htm;
        rewrite ^(.*) https://j11b209.p.ssafy.io:443$1 permanent; #https redirect
    }
    
    ############
    #SSL       #
    ############
    location /.well-known/pki-validation {
        root /var/validation;
        try_files $uri $uri/ =404;
    }
    location /.well-known/acme-challenge {
        default_type "text/plain";
        root /var/validation;
        try_files $uri $uri/ =404;
    }

    # redirect server error pages to the static page /50x.html
    #
    error_page   500 502 503 504  /50x.html;
    location = /50x.html {
        root   /usr/share/nginx/html;
    }
}

server {
    listen 443 ssl;                   #ip v4
    listen [::]:443 ssl;              #ip v6
    server_name j11b209.p.ssafy.io;

    # 사설 SSL
    #ssl on;
    #ssl_certificate      /etc/ssl/certificate.crt; 
    #ssl_certificate_key  /etc/ssl/private.key;

    ssl_certificate /etc/letsencrypt/live/j11b209.p.ssafy.io/fullchain.pem;
    ssl_certificate_key /etc/letsencrypt/live/j11b209.p.ssafy.io/privkey.pem;
    include /etc/letsencrypt/options-ssl-nginx.conf;
    ssl_dhparam /etc/letsencrypt/ssl-dhparams.pem;

    access_log   /var/log/nginx/nginx.vhost.access.log;
    error_log    /var/log/nginx/nginx.vhost.error.log;
    location     / {
      proxy_pass http://frontend;
    }
    
    ###########
    # Jenkins #
    ###########
    location /jenkins {
      proxy_pass http://jenkins;     
    }     

    # Proxy
    proxy_set_header Host $host;
    proxy_set_header X-Real-IP $remote_addr;
    proxy_set_header X-Forwarded-For $proxy_add_x_forwarded_for;
    proxy_set_header X-Forwarded-Proto $scheme;
    proxy_set_header X-Forwarded-Proto https;
    proxy_headers_hash_bucket_size 512;
    proxy_redirect off;

    # Websockets
    proxy_http_version 1.1;
    proxy_set_header Upgrade $http_upgrade;
    proxy_set_header Connection "upgrade";
    
    ############
    #websocket #
    ############
    location ~ /websocket {
       proxy_pass http://socketserver;
    }   

    location /api {
      proxy_pass http://backend;
    }
    
    location /pyapi {
      proxy_pass http://pybackend;
    }
    location /socket.io {
      proxy_pass http://pysocketio;
    }

}
```

* udp.conf
```
upstream udpserver {
  server safetycar-dev-backend:5432;
}

server {
    proxy_bind $remote_addr transparent;
    listen      5432 udp;
    proxy_pass udpserver;
}
```

* nginx.conf
```
user  nginx;
worker_processes  auto;

error_log  /var/log/nginx/error.log notice;
pid        /var/run/nginx.pid;

events {
    worker_connections  1024;
}

http {
    include       /etc/nginx/mime.types;
    default_type  application/octet-stream;

    log_format  main  '$remote_addr - $remote_user [$time_local] "$request" '
                      '$status $body_bytes_sent "$http_referer" '
                      '"$http_user_agent" "$http_x_forwarded_for"';

    access_log  /var/log/nginx/access.log  main;

    sendfile        on;
    #tcp_nopush     on;

    keepalive_timeout  65;

    #gzip  on;

    include /etc/nginx/conf.d/*.conf;
}
stream {
    include /etc/nginx/conf.d/udp/*.conf;
}
```  

# 6. 서비스 설정

### 1. Backend
* Dockerfile
```
FROM amazoncorretto:17-alpine AS build

WORKDIR /app

# 의존성 설치 및 빌드
COPY build.gradle settings.gradle gradlew ./
COPY gradle ./gradle
COPY src ./src
RUN ./gradlew build --no-daemon

# 빌드 결과물을 실행 이미지로 복사
FROM amazoncorretto:17-alpine

WORKDIR /app

COPY --from=build /app/build/libs/safetycar-0.0.1-SNAPSHOT.jar /app/safetycar-0.0.1-SNAPSHOT.jar

CMD ["java", "-jar", "/app/safetycar-0.0.1-SNAPSHOT.jar"]
```
* docker-compose.yml
```
services:
  backend:
    image: safetycar-dev-backend:latest
    container_name: safetycar-dev-backend
    networks:
      - backend-network
    expose:
      - "8080"
      - "5432/udp" # udp

networks:
  backend-network:
    external: true
```
* Jenkinsfile
```
pipeline {
    agent any

    environment {
        TARGET_BRANCH='develop/be'
    }

    stages {
        stage('Checkout') {
            steps {
                script {
                    checkout scm
                }
            }
        }
        stage('Build') {
            steps {
                dir('backend') {
                    withCredentials([file(credentialsId: 'APPLICATION_YML', variable: 'application_yml')]) {
                        sh 'cp $application_yml ./src/main/resources/application.yml'
                    }
                    sh 'chmod +x ./gradlew'
                    sh './gradlew clean build --no-daemon' // 디버깅을 위해 캐시하지 않기
                }
            }
        }
        stage('Build Docker Image') {
            steps {
                script {
                     sh 'docker build -t safetycar-dev-backend:latest ./backend'
                }
            }
        }
        stage('Depoly') {
            steps {
                dir ('backend') {
                    script {
                         sh 'docker-compose up -d'
                    }
                }

            }
        }
        stage('Remove old Image') {
            steps {
                script {
                    sh 'docker image prune -f'
                }
            }
        }

    }
}
```

### 2. Frontend
* Dockerfile
```
FROM node:16 AS build

RUN mkdir /react

WORKDIR /react

COPY package.json .

RUN npm install

COPY . .

RUN npm run build

RUN ls

FROM nginx:latest

WORKDIR /

COPY --from=build /react/build /usr/share/nginx/html

RUN rm /etc/nginx/conf.d/default.conf

COPY nginx/nginx.conf /etc/nginx/conf.d

EXPOSE 80

CMD ["nginx", "-g", "daemon off;"]
```
* docker-compose.yml
```
services:
  frontend:
    image: safetycar-dev-frontend:latest
    container_name: safetycar-dev-frontend
    networks:
      - frontend-network
    expose:
      - "80" # 프론트가 nginx를 가지고 있으므로 컨테이너에서는 80포트를 연 상태

networks:
  frontend-network:
    external: true
```
* Jenkinsfile
```
pipeline {
    agent any

    environment {
        TARGET_BRANCH='develop/fe'
    }

    stages {
        stage('Checkout') {
            steps {
                script {
                    checkout scm
                }
            }
        }

        stage('Set .ENV File') {
            steps {
                dir('frontend') {
                    withCredentials([file(credentialsId: 'ENV_FILE', variable: 'env_file')]) {
                        sh 'cp $env_file ./.env'
                    }
                }
            }
        }
        stage('Build Docker Image') {
            steps {
                script {
                    sh 'docker build --no-cache -t safetycar-dev-frontend:latest ./frontend'
                }
            }
        }

        stage('Depoly') {
            steps {
                dir('frontend') {
                    sh 'docker-compose up -d'
                }
            }
        }

        stage('Remove old image') {
            steps {
                script {
                    sh 'docker image prune -f'
                }
            }
        }
    }
}
```
* /nginx/nginx.conf
```
server {
    listen 80;
    access_log /var/log/nginx/access.log;
    error_log /var/log/nginx/error.log;
    location / {
        # root를 /usr/share/nginx/html 을 바라보게 했으므로(Dockerfile 참고)
        # 해당 경로 아래에 배포해주면 됩니다.
        root   /usr/share/nginx/html;
        index  index.html index.htm;
        try_files $uri $uri/ /index.html;
    }
}
```

### 3. Python Backend
* Dockerfile
```
FROM python:3.9

WORKDIR /app

COPY ./requirements.txt /app/requirements.txt

RUN apt-get update -y
RUN apt-get install -y libgl1-mesa-glx
RUN apt-get install -y libglib2.0-0

RUN pip install --upgrade pip
RUN pip install --no-cache-dir --upgrade -r /app/requirements.txt

COPY ./*.py /app/
COPY ./*.txt /app/

CMD [ "python3", "Python_Server.py" ]
```
* docker-compose.yml
```
services:
  pybackend:
    image: safetycar-dev-pybackend:latest
    container_name: safetycar-dev-pybackend
    networks:
      - backend-network
    expose:
      - "8000"

networks:
  backend-network:
    external: true
```
* Jenkinsfile
```
pipeline {
    agent any

    stages {
        stage('Checkout') {
            steps {
                script {
                    checkout scm
                }
            }
        }
        stage('Build Docker Image') {
            steps {
                script {
                     sh 'docker build -t safetycar-dev-pybackend:latest ./coordinate'
                }
            }
        }
        stage('Depoly') {
            steps {
                dir ('coordinate') {
                    script {
                         sh 'docker-compose up -d'
                    }
                }

            }
        }
        stage('Remove old Image') {
            steps {
                script {
                    sh 'docker image prune -f'
                }
            }
        }
    }
}
```

### 4. CCTV 실행

requirements

* WebCam
* Graphic Card with CUDA
* Internet
* Python 3.9
  
**실행**
```
git clone https://lab.ssafy.com/s11-mobility-smarthome-sub1/S11P21B209.git
cd S11P21B209/AI

pip install -r requirements.txt
pip install torch==1.13.1+cu117 torchvision==0.14.1+cu117 torchaudio==0.13.1 --extra-index-url https://download.pytorch.org/whl/cu117

python detect.py
```








