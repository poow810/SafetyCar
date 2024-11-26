 <div align="center">

<img width="300px" src="https://github.com/user-attachments/assets/33d90d25-fe2f-4411-aa58-9e971deacb6e" />

<br>

<h2>삼성 청년 SW 아카데미 11기 특화 Safety Car B209</h2>

<p>
    CCTV를 통해 심정지 환자를 인지하고
    AED 키트를 배달해주는 로봇 <strong>Safety Car🚑</strong>
</p>

<br/>
<br/>

</div>

<div>
## **목차**

1. 기획 배경 📚
2. 서비스 소개 🛠️
   - MAP 좌표 추출
   - 객체 탐지 및 심정지 환자 판단
   - 시뮬레이터 경로 생성 및 자율 주행
   - 강제 출동 서비스
   - 119 신고 문자 접수 서비스
3. 기대 효과 🌟
4. 설계 🏗️
   - 시스템 아키텍처
   - API 명세서
5. 기술 스택 🛠️
6. 팀원 소개 👥
</div>

<div>

## 💫 프로젝트 개요

심정지 환자를 살리는 AED는 1년에 30번도 사용하지 않는 문제점이 있었습니다.

하지만, 실제로 10명 중 7명은 자동심장충격기 설치여부와 위치를 모르기 때문이라는 조사 결과가 있었습니다.

이에 보행자들을 모니터링하며 AED 키트를 직접 배달해줄 수 있다면 심정지 환자들에게 도움이 되리라 생각하여 Safety Car를 기획하게 되었습니다.

<br/>

<h2> 개발 기간📅</h2>

| 개발기간 | 2024.08.19 ~ 2024.10.11 (7주) |
| -------- | ----------------------------- |

</div>

</div>

<br/>
<br/>

<!-- 기술 스택 -->

## 📌 주요 기능

| **기능**                        | **설명**                                                                                                                                           | **사용 기술**                                                                                                              |
| ------------------------------- | -------------------------------------------------------------------------------------------------------------------------------------------------- | -------------------------------------------------------------------------------------------------------------------------- |
| **실시간 보행자 모니터링**          | - 캠을 통해 실시간 보행자를 모니터링<br>                                     | - UDP를 통한 실시간 스트리밍<br>- Yolo pose 모델을 통한 객체 트래킹                                                                |
| **로봇 2D Map** | - 실시간 로봇 좌표 동기화<br>                                                                     | - 시뮬레이터에서 받아온 맵 정보와 로봇 좌표를 받아서 동기화                                                                                             |
| **로봇 출동**                    | - 보행자의 쓰러짐이 감지되면, 해당 좌표가 시뮬레이터로 전송, AED 키트 배달이 시작                                                                                                     | - MORAI 시뮬레이터와 서버 간 통신을 통해 전달된 좌표로 출발                                                |
| **로봇 강제 출동**        | - 사용자가 화면을 클릭하면 로봇이 강제 출동<br>                                                                                    | - 사용자가 화면을 클릭하면 해당 좌표로 로봇이 강제 출동 Homography와 이미지 스티칭 기술을 결합한 3D->2D 좌표 변환                                                                                         |
| **로봇 강제 복귀**               | - 사용자가 복귀 명령을 입력하면, 초기 장소로 로봇 강제 복귀<br>                               | - 시뮬레이터에 설정된 초기 좌표값으로 로봇이 복귀 |
| **맵 등록**             | - 바닥 모서리 검출<br>                                                                        | - 두 개의 공간 사진, 바닥의 세로 길이와 가로 길이를 입력받아 모서리를 검출            |
| **맵 등록**          | - 이미지 회전/반전                                                              | - 두 개의 이미지를 하나의 맵처럼 보기 위해 회전하여 바닥을 맞춰줌                                                                                      |
| **맵 등록**          | - 타일 매칭                                                              | - 두 개의 이미지를 하나의 맵처럼 보기 위해 회전하여 바닥을 맞춰줌                                                                                      |

<br/>


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

---

### Service
- nginx: 1.26.2    
- jenkins: 2.462.2     
- Docker: 27.2.1  
- Docker-compose: 2.29.2  
- Redis: 5.0.8

## 👥 팀원 소개

<table>
<tr>
    <td align="center"><a href="https://github.com/shanaid"><b>👑박예본</b></a></td>
    <td align="center"><a href="https://github.com/poow810"><b>박하운</b></a></td>
    <td align="center"><a href="https://github.com/boeunyoon"><b>임 권</b></a></td>
    <td align="center"><a href="https://github.com/Geunbeom"><b>서근범</b></a></td>
    <td align="center"><a href="https://github.com/ssuinh"><b>홍수인</b></a></td>
    <td align="center"><a href="https://github.com/"><b>황용주</b></a></td>
  </tr>
 <tr>
     <td align="center"><a href="https://github.com/shanaid"><img src="https://avatars.githubusercontent.com/shanaid" width="130px;" alt=""></a></td>
    <td align="center"><a href="https://github.com/poow810"><img src="https://avatars.githubusercontent.com/poow810" width="130px;" alt=""></a></td>
    <td align="center"><a href="https://github.com/Al17OTON"><img src="https://avatars.githubusercontent.com/Al17OTON" width="130px;" alt=""></a></td>
    <td align="center"><a href="https://github.com/Geunbeom"><img src="https://avatars.githubusercontent.com/Geunbeom" width="130px;" alt=""></a></td>
    <td align="center"><a href="https://github.com/ssuinh"><img src="https://avatars.githubusercontent.com/ssuinh" width="130px;" alt=""></a></td>
    <td align="center"><a href="https://github.com/"><img src="https://avatars.githubusercontent.com/" width="130px;" alt=""></a></td>

  </tr>
  <tr>
    <td align="center"><b>BE & 좌표 추출</b></a></td>
    <td align="center"><b>BE & 영상</b></a></td>
    <td align="center"><b>BE & INFRA</b></a></td>
    <td align="center"><b>SIMULATOR</b></a></td>
    <td align="center"><b>FE & 영상</b></a></td>
    <td align="center"><b>SIMULATOR</b></a></td>
  </tr>
</table>

# 5. 서비스 시연
![아무거나-min (1)](https://github.com/user-attachments/assets/b07ff352-dc21-439e-87ca-9c0d08e51016)







