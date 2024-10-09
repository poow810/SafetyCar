import { motion } from "framer-motion";
import React, { useState, useRef } from "react";
import "../styles/Mainpage.css"; // CSS 파일을 import
import MapComponent from "../components/map";
import axios from "axios";
import NavibarComponent from "../components/navibar";

const WEBSOCKET_URL = process.env.REACT_APP_WEBSOCKET_URL;
const PYTHON_URL = process.env.REACT_APP_PYTHON_URL;
const ws = new WebSocket(WEBSOCKET_URL);
// console.log("SOCKET CONNECTED");

const handlePopstate = () => {
  if (ws) {
    ws.close();
  }
};

function Monitor() {
  const [expandedIndex, setExpandedIndex] = useState(null);
  const [frameSrcArr, setFrameSrcArr] = useState([null, null, null, null]);
  const [simulatorImage, setSimulatorImage] = useState(null); // 시뮬레이터 이미지 상태 추가
  const [points, setPoints] = useState([]); // 좌표 상태 추가
  const imageRef1 = useRef(null); // 첫 번째 이미지 참조
  const imageRef2 = useRef(null); // 두 번째 이미지 참조
  ws.onmessage = async function (msg) {
    let newArr = [...frameSrcArr];
    const int8Array = new Int8Array(await msg.data.slice(0, 1).arrayBuffer());
    const idx = int8Array[0];
    newArr[idx] = URL.createObjectURL(msg.data.slice(1));
    setFrameSrcArr(newArr);
  };

  // 이미지 저장 함수 (Blob -> Base64로 변환 후 Local Storage에 저장)
  const saveFrameToLocalStorage = (blob, cameraIndex) => {
    const reader = new FileReader();
    reader.readAsDataURL(blob); // Blob을 Base64로 변환
    reader.onloadend = function () {
      const base64Data = reader.result;
      localStorage.setItem(`savedImageCamera${cameraIndex}`, base64Data); // 카메라 인덱스에 맞게 저장
    };
  };

  ws.onmessage = async function (msg) {
    let newArr = [...frameSrcArr];
    const int8Array = new Int8Array(await msg.data.slice(0, 1).arrayBuffer());
    const idx = int8Array[0];
    newArr[idx] = URL.createObjectURL(msg.data.slice(1));
    setFrameSrcArr(newArr);
    const blob = new Blob([msg.data.slice(1)], { type: "image/jpeg" });
    const blobUrl = URL.createObjectURL(blob);
    newArr[idx] = blobUrl;

    // 각 카메라 프레임을 Local Storage에 저장
    if (idx === 0) {
      saveFrameToLocalStorage(blob, 0); // 카메라 0번의 프레임을 저장
    } else if (idx === 1) {
      saveFrameToLocalStorage(blob, 1); // 카메라 1번의 프레임을 저장
    }
  };

  const handleMouseEnter = (index) => {
    setExpandedIndex(index);
  };

  const handleMouseLeave = () => {
    setExpandedIndex(null);
  };

  const handleImageLoad = (url) => {
    setSimulatorImage(url);
  };

  const handleImageClick = (e, imageRef, imgId) => {
    const image = imageRef.current;
    const rect = image.getBoundingClientRect();
    const x = e.clientX - rect.left;
    const y = e.clientY - rect.top;
    const scaleX = image.naturalWidth / rect.width;
    const scaleY = image.naturalHeight / rect.height;
    const adjustedX = x * scaleX;
    const adjustedY = y * scaleY;

    // 서버로 좌표 전송
    const formData = new FormData();
    formData.append("x", adjustedX);
    formData.append("y", adjustedY);
    formData.append("img_id", imgId);

    axios
      .post(`${PYTHON_URL}/get_floor_coordinates/`, formData)
      .then((response) => {
        if (response.data.error) {
          alert(response.data.error);
        } else {
          const x_floor = response.data.x_floor;
          const y_floor = response.data.y_floor;
          alert(`바닥 좌표: (${x_floor.toFixed(2)}, ${y_floor.toFixed(2)})`);
        }
      })
      .catch((error) => {
        console.error("바닥 좌표 요청 에러:", error);
        alert("바닥 좌표 요청 중 오류가 발생했습니다.");
      });
  };

  // MapComponent에서 좌표를 받는 함수
  const handlePointReceive = (point) => {
    console.log(point);

    // point가 없거나 x, y가 undefined인 경우
    if (
      !point ||
      typeof point.x === "undefined" ||
      typeof point.y === "undefined"
    ) {
      setPoints([]);
      return;
    }

    const newX = (point.x / 500) * 400;
    const newY = (point.y / 500) * 400;

    setPoints([{ x: newX, y: newY }]);
  };

  window.addEventListener("popstate", handlePopstate);

  return (
    <>
      {/* <h1>안전 관제 통합 대시보드</h1> */}
      <h1>SafetyCar 상황실</h1>

      <div className="container">
        <div className="container" style={{ display: "flex" }}>
          {/* 네비바 추가 */}
          <NavibarComponent />
        </div>
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
                <img
                  src={frameSrcArr[0]}
                  alt="CCTV 0"
                  ref={imageRef1} // ref 추가
                  onClick={(e) => handleImageClick(e, imageRef1, 1)} // onClick 추가
                />
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
                <img
                  src={frameSrcArr[1]}
                  alt="CCTV 1"
                  ref={imageRef2} // ref 추가
                  onClick={(e) => handleImageClick(e, imageRef2, 2)} // onClick 추가
                />
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
              {points.length > 0 &&
                points.map((point, index) => (
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
        <div></div>
      </div>
    </>
  );
}

export default Monitor;
