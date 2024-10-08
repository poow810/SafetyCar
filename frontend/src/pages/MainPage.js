import { motion } from "framer-motion";
import React, { useState } from "react";
import "../styles/Mainpage.css"; // CSS 파일을 import
import MapComponent from "../components/map";

const WEBSOCKET_URL = process.env.REACT_APP_WEBSOCKET_URL;
const ws = new WebSocket(WEBSOCKET_URL);

const handlePopstate = () => {
  if (ws) {
    ws.disconnect();
  }
};


function Monitor() {
  const [expandedIndex, setExpandedIndex] = useState(null);
  const [frameSrcArr, setFrameSrcArr] = useState([null, null, null, null]);
  const [simulatorImage, setSimulatorImage] = useState(null); // 시뮬레이터 이미지 상태 추가
  const [points, setPoints] = useState([]); // 좌표 상태 추가

  ws.onmessage = async function (msg) {
    let newArr = [...frameSrcArr];
    const int8Array = new Int8Array(await msg.data.slice(0, 1).arrayBuffer());
    const idx = int8Array[0];
    newArr[idx] = URL.createObjectURL(msg.data.slice(1));
    setFrameSrcArr(newArr);
  };

  const handleMouseEnter = (index) => {
    setExpandedIndex(index);
  };

  const handleMouseLeave = () => {
    setExpandedIndex(null);
  };


  // MapComponent에서 이미지를 받는 함수
  const handleImageLoad = (url) => {
    setSimulatorImage(url); // 시뮬레이터 이미지 상태 업데이트
  };

  // MapComponent에서 좌표를 받는 함수
  const handlePointReceive = (point) => {
    setPoints([point]); // 새로운 좌표로 이전 좌표를 덮어씌움
  };

  window.addEventListener("popstate", handlePopstate);

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

      <div className="container">
        {/* 모니터 섹션 */}
        <div className="monitorSection">
          {/* 첫 번째 모니터: CCTV 1 */}
          <div className="monitorContainer">
            <motion.div
              className={`monitorFrameLeft ${
                expandedIndex === 0 ? "monitorFrameHovered" : ""
              }`}
              onMouseEnter={() => handleMouseEnter(0)}
              onMouseLeave={handleMouseLeave}
            >
              <div className="monitorScreen">
                <h3 style={{ textAlign: "center", paddingTop: "20px" }}>
                  CCTV 1 화면
                </h3>
                <img src={frameSrcArr[0]} alt="CCTV 0" />
              </div>
              <div className="monitorStand"></div>
            </motion.div>
          </div>

          {/* 두 번째 모니터: CCTV 2 */}
          <div className="monitorContainer">
            <motion.div
              className={`monitorFrameRight ${
                expandedIndex === 1 ? "monitorFrameHovered" : ""
              }`}
              onMouseEnter={() => handleMouseEnter(1)}
              onMouseLeave={handleMouseLeave}
            >
              <div className="monitorScreen">
                <h3 style={{ textAlign: "center", paddingTop: "20px" }}>
                  CCTV 2 화면
                </h3>
                <img src={frameSrcArr[1]} alt="CCTV 1" />
              </div>
              <div className="monitorStand"></div>
            </motion.div>
          </div>
        </div>

        {/* 시뮬레이터 지도 섹션 */}
        <div className="simulatorOverlay">
          <MapComponent onImageLoad={handleImageLoad} onPointReceive={handlePointReceive} />
          {/* 시뮬레이터 이미지 추가 */}
          {simulatorImage && (
            <div className="simulatorImageContainer" style={{ position: 'relative' }}>
              <img
                src={simulatorImage}
                alt="Simulator"
                style={{ width: '100%', maxHeight: '400px', objectFit: 'contain' }}
              />
              {/* 좌표 표시 */}
              {points.map((point, index) => (
                <div
                  key={index}
                  style={{
                    position: 'absolute',
                    left: point.x,
                    top: point.y,
                    width: '10px',
                    height: '10px',
                    borderRadius: '50%',
                    backgroundColor: 'red',
                    transform: 'translate(-50%, -50%)', // 중앙 정렬
                  }}
                />
              ))}
            </div>
          )}
        </div>
      </div>
    </>
  );
}

export default Monitor;
