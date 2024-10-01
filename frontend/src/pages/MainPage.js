import { color, motion } from "framer-motion";
import React, { useState } from "react";
import { useNavigate } from "react-router-dom";

function Monitor() {
  const [expandedIndex, setExpandedIndex] = useState(null); // 클릭된 상태를 관리
  const navigate = useNavigate(); // 페이지 이동을 위한 훅

  const styles = {
    container: {
      display: "flex",
      flexDirection: "row",
      justifyContent: "space-between", // 양쪽으로 분리
      alignItems: "flex-start",
      height: "100vh",
      padding: "0 2%", // 좌우 여백 추가
      boxSizing: "border-box",
    },
    monitorSection: {
      display: "flex",
      flexDirection: "row",
      gap: "10px",
      alignItems: "center",
      justifyContent: "flex-start",
      flex: 0.5, // 모니터 섹션이 화면의 50%만 차지하도록 비율 조정
    },
    simulatorSection: {
      display: "flex",
      justifyContent: "center",
      alignItems: "center",
      width: "45%", // 시뮬레이터 지도가 화면의 45% 차지
      position: "relative", // 시뮬레이터 지도가 모니터 위로 올 수 있도록 설정
      zIndex: 1, // 시뮬레이터 지도가 모니터 위에 표시되도록 설정
    },
    monitorContainer: {
      display: "flex",
      justifyContent: "center",
      alignItems: "center",
      perspective: "1000px",
      zIndex: 0, // 모니터는 시뮬레이터 지도 아래에 위치
    },
    monitorFrameLeft: {
      width: "550px",
      height: "350px",
      backgroundColor: "#222",
      borderRadius: "20px",
      padding: "15px",
      position: "relative",
      boxShadow: "0 20px 40px rgba(0, 0, 0, 0.6)",
      transform: "rotateY(10deg)", // 왼쪽 모니터는 오른쪽을 바라보도록 회전
      transition: "transform 0.3s ease",
    },
    monitorFrameRight: {
      width: "550px",
      height: "350px",
      backgroundColor: "#222",
      borderRadius: "20px",
      padding: "15px",
      position: "relative",
      boxShadow: "0 20px 40px rgba(0, 0, 0, 0.6)",
      transform: "rotateY(-10deg)", // 오른쪽 모니터는 왼쪽을 바라보도록 회전
      transition: "transform 0.3s ease",
    },
    monitorFrameHovered: {
      transform: "rotateY(0deg)", // 호버 시 회전 해제
    },
    monitorScreen: {
      width: "100%",
      height: "100%",
      backgroundColor: "#121212",
      borderRadius: "10px",
      overflow: "hidden",
      display: "flex",
      flexDirection: "column",
      justifyContent: "center",
      alignItems: "center",
      color: "white",
      fontFamily: "'Roboto Mono', monospace",
    },
    monitorStand: {
      width: "100px",
      height: "25px",
      backgroundColor: "#333",
      borderRadius: "5px",
      position: "absolute",
      bottom: "-40px",
      left: "50%",
      transform: "translateX(-50%)",
      boxShadow: "0 10px 20px rgba(0, 0, 0, 0.4)",
    },
    simulatorOverlay: {
      width: "400px", // 시뮬레이터 지도 크기 조정
      height: "500px",
      backgroundColor: "rgba(34, 34, 34, 0.7)", // 반투명한 배경
      borderRadius: "20px",
      display: "flex",
      justifyContent: "center",
      alignItems: "center",
      color: "white",
      fontSize: "24px",
      border: "2px solid rgba(255, 255, 255, 0.2)",
      boxShadow: "0 20px 40px rgba(0, 0, 0, 0.5)",
      position: "fixed", // 화면 오른쪽에 고정
      top: "20%", // 화면 상단에서 약간 떨어져서 위치
      right: "2%", // 화면 오른쪽에 고정
      zIndex: 1, // 모니터 위에 오도록 설정
    },
    // 반응형을 위한 미디어 쿼리 추가
    "@media (max-width: 1200px)": {
      container: {
        flexDirection: "column",
        alignItems: "center",
      },
      monitorSection: {
        flexDirection: "column",
        gap: "20px",
        flex: "none",
        width: "100%",
        justifyContent: "center",
      },
      simulatorSection: {
        width: "100%",
        height: "400px",
        marginTop: "20px",
      },
    },
    "@media (max-width: 768px)": {
      monitorFrameLeft: {
        width: "100%",
        transform: "rotateY(0deg)", // 작은 화면에서 회전 제거
      },
      monitorFrameRight: {
        width: "100%",
        transform: "rotateY(0deg)", // 작은 화면에서 회전 제거
      },
      simulatorOverlay: {
        width: "100%", // 작은 화면에서는 전체 너비 사용
        height: "auto",
        fontSize: "20px",
      },
    },
  };

  const handleMouseEnter = (index) => {
    setExpandedIndex(index);
  };

  const handleMouseLeave = () => {
    setExpandedIndex(null);
  };

  return (
    <>
      <h1
        style={{
          color: "white",
          textAlign: "center",
          marginTop: "50px",
          marginBottom: "70px",
        }}
      >
        안전 관제 통합 대시보드
      </h1>
      <div style={styles.container}>
        {/* 모니터 섹션 */}
        <div style={styles.monitorSection}>
          {/* 첫 번째 모니터: CCTV 1 */}
          <div style={styles.monitorContainer}>
            <motion.div
              style={{
                ...styles.monitorFrameLeft,
                ...(expandedIndex === 0 ? styles.monitorFrameHovered : {}),
              }}
              onMouseEnter={() => handleMouseEnter(0)}
              onMouseLeave={handleMouseLeave}
            >
              <div style={styles.monitorScreen}>
                <h3 style={{ textAlign: "center", paddingTop: "20px" }}>
                  CCTV 1 화면
                </h3>
              </div>
              <div style={styles.monitorStand}></div>
            </motion.div>
          </div>

          {/* 두 번째 모니터: CCTV 2 */}
          <div style={styles.monitorContainer}>
            <motion.div
              style={{
                ...styles.monitorFrameRight,
                ...(expandedIndex === 1 ? styles.monitorFrameHovered : {}),
              }}
              onMouseEnter={() => handleMouseEnter(1)}
              onMouseLeave={handleMouseLeave}
            >
              <div style={styles.monitorScreen}>
                <h3 style={{ textAlign: "center", paddingTop: "20px" }}>
                  CCTV 2 화면
                </h3>
              </div>
              <div style={styles.monitorStand}></div>
            </motion.div>
          </div>
        </div>

        {/* 시뮬레이터 지도 섹션 */}
        <div style={styles.simulatorOverlay}>
          <h2>시뮬레이터 지도</h2>
        </div>
      </div>
    </>
  );
}

export default Monitor;
