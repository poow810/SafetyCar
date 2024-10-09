// Step5.js
import React, { useRef, useState, useEffect } from "react";
import { useNavigate, useLocation } from "react-router-dom";
import axios from "axios";

const PYTHON_URL = process.env.REACT_APP_PYTHON_URL;

const Step5 = () => {
  const videoDisplayRef1 = useRef(null);
  const videoDisplayRef2 = useRef(null);
  const location = useLocation();
  const navigate = useNavigate();
  const { mergedImageSrc } = location.state || {}; // 이전 단계에서 받은 합성 이미지
  const [transformedCoordinates, setTransformedCoordinates] = useState(null);
  // 방 번호 및 카메라 ID 상태
  const [roomId, setRoomId] = useState("");
  const [cameraId1, setCameraId1] = useState("");
  const [cameraId2, setCameraId2] = useState("");
  // 이미지가 없을 경우 Step4로 리디렉션

  const [camera0Image, setCamera0Image] = useState(null);
  const [camera1Image, setCamera1Image] = useState(null);

  useEffect(() => {
    if (!mergedImageSrc) {
      alert("이전 단계에서 합성된 이미지가 제공되지 않았습니다.");
      // navigate("/step4"); // Step4로 이동
    }

    // Local Storage에서 이미지 불러오기
    const loadImages = async () => {
      const savedImage0 = localStorage.getItem("savedImageCamera0");
      const savedImage1 = localStorage.getItem("savedImageCamera1");

      if (savedImage0) {
        setCamera0Image(savedImage0);
      }

      if (savedImage1) {
        setCamera1Image(savedImage1);
      }
    };

    loadImages();
  }, [mergedImageSrc, navigate]);

  // 비디오 클릭 시 좌표 전송 및 변환된 좌표 출력
  const handleVideoClick = (e, videoRef, imgId) => {
    const video = videoRef.current;
    const rect = video.getBoundingClientRect();
    const x = e.clientX - rect.left;
    const y = e.clientY - rect.top;
    const scaleX = video.videoWidth / rect.width;
    const scaleY = video.videoHeight / rect.height;
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
          console.log("x:", x);
          console.log("y:", y);
          alert(`바닥 좌표: (${x_floor.toFixed(2)}, ${y_floor.toFixed(2)})`);
        }
      })
      .catch((error) => {
        console.error("바닥 좌표 요청 에러:", error);
        alert("바닥 좌표 요청 중 오류가 발생했습니다.");
      });
  };

  // -------------------roomId 서버에서 받아서 자동 입력되도록 수정 예정------------------------
  // 변환 행렬을 서버에 저장하는 함수
  const handleSaveTransformations = () => {
    if (!cameraId1 || !cameraId2) {
      alert("두 카메라 번호를 모두 입력해주세요.");
      return;
    }
    const video1 = videoDisplayRef1.current;
    const rect1 = video1.getBoundingClientRect();
    const computedScaleX1 = video1.videoWidth / rect1.width;
    const computedScaleY1 = video1.videoHeight / rect1.height;
    const roomId = 1;

    const video2 = videoDisplayRef2.current;
    const rect2 = video2.getBoundingClientRect();
    const computedScaleX2 = video2.videoWidth / rect2.width;
    const computedScaleY2 = video2.videoHeight / rect2.height;

    const formData = new FormData();
    formData.append("room_id", roomId);
    formData.append("camera_id1", cameraId1);
    formData.append("camera_id2", cameraId2);
    formData.append("scaleX1", computedScaleX1);
    formData.append("scaleY1", computedScaleY1);
    formData.append("scaleX2", computedScaleX2);
    formData.append("scaleY2", computedScaleY2);

    console.log("보내는 데이터:", {
      roomId,
      cameraId1,
      cameraId2,
      scaleX1: computedScaleX1,
      scaleY1: computedScaleY1,
      scaleX2: computedScaleX2,
      scaleY2: computedScaleY2,
    }); // 디버깅을 위해 추가

    axios
      .post(`${PYTHON_URL}/save_transformations/`, formData)
      .then((response) => {
        if (response.data.message) {
          console.log(`변환 행렬 저장 성공:`, response.data.message);
          alert(response.data.message);
        } else if (response.data.error) {
          alert(response.data.error);
        }
      })
      .catch((error) => {
        console.error("변환 행렬 저장 에러:", error);
        alert("변환 행렬 저장 중 오류가 발생했습니다.");
      });
  };

  return (
    <>
      <h2>5. 영상에서 바닥 좌표 확인 및 변환 행렬 저장</h2>
      <p>비디오를 클릭하여 바닥 좌표를 확인하세요.</p>
      <div className="container">
        <div className="right-container">
          <div className="image-container">
            <div className="image-box">
              <h3>영상 1</h3>
              {/* <video
            ref={videoDisplayRef1}
            src="../../assets/cctv1.mp4"
            controls
            style={{ maxWidth: "100%", cursor: "crosshair" }}
            onClick={(e) => handleVideoClick(e, videoDisplayRef1, 1)}
          /> */}
              <img
                src={camera0Image}
                alt="Camera 1"
                ref={videoDisplayRef1}
                style={{ maxWidth: "100%", cursor: "crosshair" }}
                onClick={(e) => handleVideoClick(e, videoDisplayRef1, 1)}
              />
            </div>

            <div className="image-box">
              <h3>영상 2</h3>
              {/* <video
            ref={videoDisplayRef2}
            src="../../assets/cctv2.mp4"
            controls
            style={{ maxWidth: "100%", cursor: "crosshair" }}
            onClick={(e) => handleVideoClick(e, videoDisplayRef2, 2)}
          /> */}
              <img
                src={camera1Image}
                alt="Camera 1"
                ref={videoDisplayRef2}
                style={{ maxWidth: "100%", cursor: "crosshair" }}
                onClick={(e) => handleVideoClick(e, videoDisplayRef2, 2)}
              />
            </div>

            <div className="image-box">
              <h3>합성된 이미지</h3>
              {mergedImageSrc && (
                <img
                  src={mergedImageSrc}
                  alt="합성된 이미지"
                  style={{ maxWidth: "500px", height: "auto" }}
                />
              )}
            </div>
          </div>

          <h3>변환 행렬 저장</h3>
          <div className="option-container">
            <label>
              카메라 1 번호:
              <input
                type="number"
                value={cameraId1}
                onChange={(e) => setCameraId1(e.target.value)}
                style={{ marginLeft: "10px" }}
              />
            </label>
            <label>
              카메라 2 번호:
              <input
                type="number"
                value={cameraId2}
                onChange={(e) => setCameraId2(e.target.value)}
                style={{ marginLeft: "10px" }}
              />
            </label>
            <button
              onClick={handleSaveTransformations}
              style={{ marginTop: "10px" }}
            >
              변환 행렬 저장
            </button>
          </div>
        </div>
      </div>
    </>
  );
};

export default Step5;
