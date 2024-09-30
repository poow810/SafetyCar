import { motion } from "framer-motion";
import React, { useState } from "react";
import { useNavigate } from "react-router-dom";

function Monitor() {
  const [expandedIndex, setExpandedIndex] = useState(null); // 클릭된 상태를 관리
  const navigate = useNavigate(); // 페이지 이동을 위한 훅

  const styles = {
    main: {
      display: "flex",
      flexDirection: "row",
      gap: "20px",
      alignItems: "center",
      justifyContent: "center",
    },
    monitorContainer: {
      display: "flex",
      justifyContent: "center",
      alignItems: "center",
      height: "100vh",
    },
    monitorFrame: {
      width: "600px",
      height: "400px",
      backgroundColor: "#333",
      borderRadius: "15px",
      padding: "20px",
      position: "relative",
      boxShadow: "0 10px 30px rgba(0, 0, 0, 0.2)",
    },
    monitorScreen: {
      width: "100%",
      height: "100%",
      backgroundColor: "#000",
      borderRadius: "10px",
      overflow: "hidden",
      display: "flex",
      flexDirection: "column",
      justifyContent: "center",
      alignItems: "center",
      color: "white",
    },
    monitorStand: {
      width: "80px",
      height: "20px",
      backgroundColor: "#333",
      borderRadius: "5px",
      position: "absolute",
      bottom: "-30px",
      left: "50%",
      transform: "translateX(-50%)",
      boxShadow: "0 5px 10px rgba(0, 0, 0, 0.3)",
    },
  };

  // 클릭 시 애니메이션 진행 후 페이지 전환 함수
  const handleClick = (index) => {
    setExpandedIndex(index);
    // setTimeout(() => {
    //   navigate(`/cctv-${index}`); // 클릭된 모니터에 따라 페이지 전환 (페이지마다 다르게 설정 가능)
    // }, 1000); // 애니메이션이 끝난 후 0.5초 뒤에 페이지 전환
  };

  return (
    <div style={styles.main}>
      <div style={styles.monitorContainer}>
        <motion.div
          style={styles.monitorFrame}
          initial={{ width: "600px", height: "400px" }}
          animate={
            expandedIndex === 0
              ? {
                  width: "100vw",
                  height: "100vh",
                  borderRadius: "0px",
                  position: "fixed", // 중앙 고정
                  top: "50%", // 화면의 세로 중앙
                  left: "50%", // 화면의 가로 중앙
                  transform: "translate(-50%, -50%)", // 중앙에서 정확하게 위치
                  zIndex: 1,
                }
              : { width: "600px", height: "400px" }
          }
          transition={{ duration: 0.5, ease: "easeInOut" }}
          onClick={() => handleClick(0)} // 첫 번째 모니터 클릭
        >
          <div style={styles.monitorScreen}>
            {/* 첫 번째 모니터 화면 내용 */}
            <h2>Monitor Display</h2>
            <p>This is where the screen content goes.</p>
          </div>
          <div style={styles.monitorStand}></div>
        </motion.div>
      </div>

      <div style={styles.monitorContainer}>
        <motion.div
          style={styles.monitorFrame}
          initial={{ width: "600px", height: "400px" }}
          animate={
            expandedIndex === 1
              ? {
                  width: "100vw",
                  height: "100vh",
                  borderRadius: "0px",
                  position: "fixed", // 중앙 고정
                  top: "50%", // 화면의 세로 중앙
                  left: "50%", // 화면의 가로 중앙
                  transform: "translate(-50%, -50%)", // 중앙에서 정확하게 위치
                  zIndex: 1,
                }
              : { width: "600px", height: "400px" }
          }
          transition={{ duration: 0.5, ease: "easeInOut" }}
          onClick={() => handleClick(1)} // 두 번째 모니터 클릭
        >
          <div style={styles.monitorScreen}>
            {/* 두 번째 모니터 화면 내용 */}
            <h2>Monitor Display</h2>
            <p>This is where the screen content goes.</p>
          </div>
          <div style={styles.monitorStand}></div>
        </motion.div>
      </div>
    </div>
  );
}

export default Monitor;
