import React, { useState } from "react";
import { useNavigate, useLocation } from "react-router-dom";
import axios from "axios";
import "../../styles/StepPages.css";

const PYTHON_URL = process.env.REACT_APP_PYTHON_URL;

const Step2 = () => {
  const navigate = useNavigate();
  const location = useLocation(); // Step1에서 받은 이미지 데이터
  const { image1Src: initialImage1Src, image2Src: initialImage2Src } =
    location.state || {};

  const [image1Src, setImage1Src] = useState(initialImage1Src || "");
  const [image2Src, setImage2Src] = useState(initialImage2Src || "");
  const [selectedImage, setSelectedImage] = useState(1);

  // 이미지 선택
  const handleSelectImage = (imgId) => {
    setSelectedImage(imgId);
  };

  // 이미지를 조정하는 함수
  const handleAdjustImages = (key) => {
    const formData = new FormData();
    formData.append("key", key);
    formData.append("img_id", selectedImage);

    axios
      .post(`${PYTHON_URL}/adjust_images/`, formData)
      .then((response) => {
        console.log(response.data);
        // 이미지 1에 대한 조정 결과를 처리
        setImage1Src("data:image/jpeg;base64," + response.data.image1);
        setImage2Src("data:image/jpeg;base64," + response.data.image2);
        if (response.data.step === 3) {
          navigate("/step3", { state: { image1Src, image2Src } });
        }
      })
      .catch((error) => {
        console.error("이미지 조정 에러:", error);
        alert("이미지 조정 중 오류가 발생했습니다.");
      });
  };

  return (
    <div>
      <h1>2. 이미지 회전/반전</h1>
      <div className="container">
        <div className="nav-container">
          <ul>
            <li>홈</li>
            <li>좌표 셋팅</li>
            <li>사건 기록</li>
          </ul>
        </div>

        <div className="right-container">
          <div className="image-container">
            <div className="image-box">
              {image1Src ? (
                <img
                  src={image1Src}
                  alt="변환된 이미지 1"
                  style={{ maxWidth: "100%", height: "auto" }}
                />
              ) : (
                <p>이미지 1을 불러오는 중...</p>
              )}
            </div>

            <div className="image-box">
              {image2Src ? (
                <img
                  src={image2Src}
                  alt="변환된 이미지 2"
                  style={{ maxWidth: "100%", height: "auto" }}
                />
              ) : (
                <p>이미지 2를 불러오는 중...</p>
              )}
            </div>
          </div>

          <div className="option-container">
            <p>선택된 이미지: 이미지 {selectedImage}</p>
            <button onClick={() => handleSelectImage(1)}>이미지 1 선택</button>
            <button
              onClick={() => handleSelectImage(2)}
              style={{ marginLeft: "10px" }}
            >
              이미지 2 선택
            </button>

            <button onClick={() => handleAdjustImages("r")}>
              90도 시계 방향 회전
            </button>
            <button
              onClick={() => handleAdjustImages("e")}
              style={{ marginLeft: "10px" }}
            >
              90도 반시계 방향 회전
            </button>
            <button
              onClick={() => handleAdjustImages("h")}
              style={{ marginLeft: "10px" }}
            >
              좌우 반전
            </button>
            <button
              onClick={() => handleAdjustImages("v")}
              style={{ marginLeft: "10px" }}
            >
              상하 반전
            </button>
            <button
              onClick={() => handleAdjustImages("n")} // 다음 단계로 이동하는 버튼에 조정된 이미지 넘기기
              className="submit-btn"
              style={{ marginLeft: "10px" }}
            >
              다음
            </button>
          </div>
        </div>
      </div>
    </div>
  );
};

export default Step2;
