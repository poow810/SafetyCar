import React, { useState, useRef, useEffect } from "react";
import { useNavigate } from "react-router-dom";
import axios from "axios";
import "../../styles/step1.css";

const PYTHON_URL = process.env.REACT_APP_PYTHON_URL;

const Step1 = () => {
  const [floorPoints1, setFloorPoints1] = useState([]);
  const [floorPoints2, setFloorPoints2] = useState([]);
  const [floorWidth, setFloorWidth] = useState("");
  const [floorHeight, setFloorHeight] = useState("");

  const videoCaptureRef1 = useRef(null);
  const videoCaptureRef2 = useRef(null);
  const imageRef1 = useRef(null);
  const imageRef2 = useRef(null);
  const navigate = useNavigate();

  const [camera0Image, setCamera0Image] = useState(null);
  const [camera1Image, setCamera1Image] = useState(null);

  useEffect(() => {
    // Local Storage에서 이미지 불러오기
    const savedImage0 = localStorage.getItem("savedImageCamera0");
    const savedImage1 = localStorage.getItem("savedImageCamera1");
    if (savedImage0) {
      setCamera0Image(savedImage0);
    }
    if (savedImage1) {
      setCamera1Image(savedImage1);
    }
    console.log(savedImage0);
    console.log(savedImage1);
  }, []);

  // // 비디오 프레임 캡처 함수
  // const captureFrame = (video) => {
  //   const canvas = document.createElement("canvas");
  //   canvas.width = video.videoWidth;
  //   canvas.height = video.videoHeight;
  //   const ctx = canvas.getContext("2d");
  //   ctx.drawImage(video, 0, 0, canvas.width, canvas.height);
  //   return canvas.toDataURL("image/jpeg");
  // };

  // useEffect(() => {
  //   const video1 = videoCaptureRef1.current;
  //   const video2 = videoCaptureRef2.current;

  //   if (video1 && video2) {
  //     const handleLoadedMetadata1 = () => {
  //       video1.currentTime = 0;
  //     };

  //     const handleLoadedMetadata2 = () => {
  //       video2.currentTime = 0;
  //     };

  //     const handleSeeked1 = () => {
  //       const frame1 = captureFrame(video1);
  //       setImage1Src(frame1);
  //     };

  //     const handleSeeked2 = () => {
  //       const frame2 = captureFrame(video2);
  //       setImage2Src(frame2);
  //     };

  //     video1.addEventListener("loadedmetadata", handleLoadedMetadata1);
  //     video2.addEventListener("loadedmetadata", handleLoadedMetadata2);

  //     video1.addEventListener("seeked", handleSeeked1);
  //     video2.addEventListener("seeked", handleSeeked2);

  //     return () => {
  //       video1.removeEventListener("loadedmetadata", handleLoadedMetadata1);
  //       video2.removeEventListener("loadedmetadata", handleLoadedMetadata2);
  //       video1.removeEventListener("seeked", handleSeeked1);
  //       video2.removeEventListener("seeked", handleSeeked2);
  //     };
  //   }
  // }, []);

  // 이미지 클릭 핸들러 (좌표 선택)
  const handleImageClick = (e, imgRef, setPoints, maxPoints) => {
    if (e.nativeEvent.which !== 1) return;
    const img = imgRef.current;
    const rect = img.getBoundingClientRect();
    const x = e.clientX - rect.left;
    const y = e.clientY - rect.top;
    const scaleX = img.naturalWidth / rect.width;
    const scaleY = img.naturalHeight / rect.height;
    const adjustedX = x * scaleX;
    const adjustedY = y * scaleY;

    setPoints((prevPoints) => {
      if (prevPoints.length < maxPoints) {
        return [...prevPoints, [adjustedX, adjustedY]];
      } else {
        return prevPoints;
      }
    });
  };

  // 이미지 업로드 함수
  const handleUploadImages = () => {
    if (!floorWidth || !floorHeight) {
      alert("바닥의 너비와 높이를 입력해주세요.");
      return;
    }
    const formData = new FormData();
    const image1Blob = dataURLtoBlob(camera0Image);
    const image2Blob = dataURLtoBlob(camera1Image);
    // console.log("업로드", image1Blob);
    // console.log(image2Blob);
    // Blob 데이터가 정상적으로 변환되었는지 확인
    if (!image1Blob || !image2Blob) {
      alert("이미지를 처리하는 중 오류가 발생했습니다.");
      return;
    }
    formData.append("image1", image1Blob, "camera0Image.jpg");
    formData.append("image2", image2Blob, "camera1Image.jpg");
    formData.append("pts1_floor", JSON.stringify(floorPoints1));
    formData.append("pts2_floor", JSON.stringify(floorPoints2));
    formData.append("floor_width", parseFloat(floorWidth));
    formData.append("floor_height", parseFloat(floorHeight));

    axios
      .post(`${PYTHON_URL}/upload_images/`, formData)
      .then((response) => {
        console.log(response.data);
        const processedImage1 =
          "data:image/jpeg;base64," + response.data.image1;
        const processedImage2 =
          "data:image/jpeg;base64," + response.data.image2;

        // Step2 페이지로 전송
        navigate("/step2", {
          state: { image1Src: processedImage1, image2Src: processedImage2 },
        });
      })
      .catch((error) => {
        console.error("이미지 업로드 에러:", error);
        alert("이미지 업로드 중 오류가 발생했습니다.");
      });
  };

  // Base64 이미지 데이터를 Blob으로 변환하는 함수
  const dataURLtoBlob = (dataURL) => {
    const arr = dataURL.split(",");
    const mimeMatch = arr[0].match(/:(.*?);/);
    const mime = mimeMatch ? mimeMatch[1] : "image/jpeg";
    const bstr = atob(arr[1]);
    let n = bstr.length;
    const u8arr = new Uint8Array(n);
    while (n--) {
      u8arr[n] = bstr.charCodeAt(n);
    }
    return new Blob([u8arr], { type: mime });
  };

  return (
    <div className="admin-container">
      <h2 className="admin-title">1. 이미지에서 바닥의 네 끝점 선택</h2>
      {/* (roomId) BE 수정 완료되면 전송할 데이터 */}
      <div>
        <label>
          방 번호:
          <input
            type="text"
            // value={roomId}
            // onChange={(e) => setRoomId(e.target.value)}
            style={{ marginLeft: "10px" }}
          />
        </label>
      </div>
      {/* 숨겨진 비디오 요소 (이미지 캡처용) */}
      <video
        ref={videoCaptureRef1}
        style={{ display: "none" }}
        preload="metadata"
      >
        <source src="/assets/cctv1.mp4" type="video/mp4" />
      </video>
      <video
        ref={videoCaptureRef2}
        style={{ display: "none" }}
        preload="metadata"
      >
        <source src="/assets/cctv2.mp4" type="video/mp4" />
      </video>

      <div className="admin-images">
        <div className="admin-image-container">
          <h3>이미지 1</h3>
          {/* {image1Src && (
            <img
              src={image1Src}
              alt="캡처된 이미지 1"
              ref={imageRef1}
              className="admin-image"
              style={{
                cursor: floorPoints1.length < 4 ? "crosshair" : "default",
                maxWidth: "100%",
                height: "auto",
              }}
              onClick={(e) =>
                handleImageClick(e, imageRef1, setFloorPoints1, 4)
              }
            />
          )} */}
          {camera0Image ? (
            <img
              src={camera0Image}
              alt="Camera 0"
              ref={imageRef1}
              className="admin-image"
              style={{
                cursor: floorPoints1.length < 4 ? "crosshair" : "default",
                maxWidth: "100%",
                height: "auto",
              }}
              onClick={(e) =>
                handleImageClick(e, imageRef1, setFloorPoints1, 4)
              }
            />
          ) : (
            <p>No image saved for Camera 0</p>
          )}

          <p>선택한 포인트 수: {floorPoints1.length} / 4</p>
        </div>

        <div className="admin-image-container">
          <h3>이미지 2</h3>
          {/* {image2Src && (
            <img
              src={image2Src}
              alt="캡처된 이미지 2"
              ref={imageRef2}
              className="admin-image"
              style={{
                cursor: floorPoints2.length < 4 ? "crosshair" : "default",
                maxWidth: "100%",
                height: "auto",
              }}
              onClick={(e) =>
                handleImageClick(e, imageRef2, setFloorPoints2, 4)
              }
            />
          )} */}
          {camera1Image ? (
            <img
              src={camera1Image}
              alt="Camera 1"
              ref={imageRef2}
              className="admin-image"
              style={{
                cursor: floorPoints2.length < 4 ? "crosshair" : "default",
                maxWidth: "100%",
                height: "auto",
              }}
              onClick={(e) =>
                handleImageClick(e, imageRef2, setFloorPoints2, 4)
              }
            />
          ) : (
            <p>No image saved for Camera 1</p>
          )}

          <p>선택한 포인트 수: {floorPoints2.length} / 4</p>
        </div>
      </div>

      <div>
        <label>
          바닥 너비 (Floor Width)
          <input
            type="number"
            value={floorWidth}
            onChange={(e) => setFloorWidth(e.target.value)}
            style={{ marginLeft: "10px" }}
          />
        </label>
      </div>
      <div style={{ marginTop: "10px" }}>
        <label>
          바닥 높이 (Floor Height)
          <input
            type="number"
            value={floorHeight}
            onChange={(e) => setFloorHeight(e.target.value)}
            style={{ marginLeft: "10px" }}
          />
        </label>
      </div>
      <button
        onClick={handleUploadImages}
        className="admin-button"
        disabled={floorPoints1.length < 4 || floorPoints2.length < 4}
      >
        다음 단계
      </button>
    </div>
  );
};

export default Step1;
