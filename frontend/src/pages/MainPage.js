import { motion } from "framer-motion";
import React, { useState } from "react";
import { useNavigate } from "react-router-dom";
import "../styles/Mainpage.css"; // CSS 파일을 import

const WEBSOCKET_URL = process.env.REACT_APP_WEBSOCKET_URL;

const ws = new WebSocket(WEBSOCKET_URL);

const handlePopstate = () => {
  if (ws) {
    ws.disconnect();
  }
};

function Monitor() {
  const [expandedIndex, setExpandedIndex] = useState(null);
  const navigate = useNavigate();

  const [frameSrcArr, setFrameSrcArr] = useState([null, null, null, null]);

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

  window.addEventListener("popstate", handlePopstate);

  return (
    <>
      <h1>안전 관제 통합 대시보드</h1>

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
                <img src={frameSrcArr[0]} alt="CCTV 0"></img>
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
                <img src={frameSrcArr[1]} alt="CCTV 1"></img>
              </div>
              <div className="monitorStand"></div>
            </motion.div>
          </div>
        </div>

        {/* 시뮬레이터 지도 섹션 */}
        <div className="simulatorOverlay">
          <h2>시뮬레이터 지도 </h2>
        </div>
      </div>
    </>
  );
}

export default Monitor;
