import { motion } from "framer-motion";
import React, { useState } from "react";
import "../styles/Mainpage.css"; // CSS 파일을 import
import MapComponent from "../components/map";
import axios from "axios";

const WEBSOCKET_URL = process.env.REACT_APP_WEBSOCKET_URL;
const API_URL = process.env.REACT_APP_API_URL;
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
  const [showModal, setShowModal] = useState(false); // 모달 창 표시 여부

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
    // 500x500 기준에서 400x400 기준으로 변환
    const newX = (point.x / 500) * 400;
    const newY = (point.y / 500) * 400;

    setPoints([{ x: newX, y: newY }]); // 변환된 좌표로 업데이트
  };

  const handleSendSMS = () => {
    const space = "삼성화재 유성캠퍼스(SSAFY 교육동)";

    axios
      .post(`${API_URL}/sms/send?space=${space}`)
      .then((response) => {
        if (response.data.message) {
          console.log("신고가 접수 완료");
          alert("신고가 성공적으로 접수되었습니다.");
        }
      })
      .catch((error) => {
        console.error("신고 접수 중 에러 발생", error);
        alert("신고에 실패했습니다. 다시 시도해주세요.");
      });
  };

  const handleOpenModal = () => {
    setShowModal(true); // 모달 창 열기
  };

  const handleCloseModal = () => {
    setShowModal(false); // 모달 창 닫기
  };
  window.addEventListener("popstate", handlePopstate);

  return (
    <>
      {/* <h1>안전 관제 통합 대시보드</h1> */}
      <h1>SafetyCar 상황실</h1>

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
                <img src={frameSrcArr[1]} alt="CCTV 1" />
              </div>
              <div className="monitorStand"></div>
            </motion.div>
          </div>
        </div>

        {/* 시뮬레이터 지도 섹션 */}
        <div className="simulatorOverlay">
          <MapComponent
            onImageLoad={handleImageLoad}
            onPointReceive={handlePointReceive}
          />

          {/* 시뮬레이터 이미지 추가 */}
          {simulatorImage && (
            <div
              className="simulatorImageContainer"
              style={{ position: "relative" }}
            >
              <img
                src={simulatorImage}
                alt="Simulator"
                style={{
                  width: "100%",
                  height: "100%",
                  objectFit: "contain",
                  borderRadius: "20px",
                  opacity: "0.3",
                }} // 컨테이너에 맞춰 조정
              />
              {/* 좌표 표시 */}
              {points.map((point, index) => (
                <div
                  key={index}
                  style={{
                    position: "absolute",
                    left: point.x,
                    top: point.y,
                    width: "10px",
                    height: "10px",
                    borderRadius: "50%",
                    backgroundColor: "red",
                    transform: "translate(-50%, -50%)", // 중앙 정렬
                  }}
                />
              ))}
            </div>
          )}
        </div>
        <div>
          <button onClick={handleOpenModal}>119 신고</button>
        </div>

        {/* 모달 창 */}
        {showModal && (
          <div className="modal-backdrop">
            <div className="modal-content">
              <h2>신고 확인</h2>
              <p>신고를 진행하시겠습니까?</p>
              <button onClick={handleSendSMS}>확인</button>
              <button onClick={handleCloseModal}>취소</button>
            </div>
          </div>
        )}
      </div>
    </>
  );
}

export default Monitor;
