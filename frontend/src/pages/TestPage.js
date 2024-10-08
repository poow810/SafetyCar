// App.js
import React, { useState, useEffect, useRef } from "react";
import axios from "axios";

const PYTHON_URL = process.env.REACT_APP_PYTHON_URL;

function Testpage() {
  const [step, setStep] = useState(1);
  const [image1Src, setImage1Src] = useState(null);
  const [image2Src, setImage2Src] = useState(null);
  const [mergedImageSrc, setMergedImageSrc] = useState(null);
  const [selectedImage, setSelectedImage] = useState(1); // 현재 선택된 이미지 (1 또는 2)

  // 프레임 캡처용 비디오 요소에 대한 참조 (숨겨진 비디오)
  const videoCaptureRef1 = useRef(null);
  const videoCaptureRef2 = useRef(null);

  // 단계 5에서 표시할 비디오 요소에 대한 참조 (사용자에게 보여지는 비디오)
  const videoDisplayRef1 = useRef(null);
  const videoDisplayRef2 = useRef(null);

  // 이미지 요소에 대한 참조
  const imageRef1 = useRef(null);
  const imageRef2 = useRef(null);

  // 사용자 선택한 포인트
  const [floorPoints1, setFloorPoints1] = useState([]);
  const [floorPoints2, setFloorPoints2] = useState([]);

  // 타일 모서리 선택을 위한 포인트
  const [tilePoints1, setTilePoints1] = useState([]);
  const [tilePoints2, setTilePoints2] = useState([]);

  // 대응점 선택을 위한 포인트
  const [alignPoints1, setAlignPoints1] = useState([]);
  const [alignPoints2, setAlignPoints2] = useState([]);

  // 방 번호 및 카메라 ID 상태
  const [roomId, setRoomId] = useState("");
  const [cameraId1, setCameraId1] = useState("");
  const [cameraId2, setCameraId2] = useState("");
  // 바닥 크기 상태를 추가
  const [floorWidth, setFloorWidth] = useState(""); // 바닥 너비
  const [floorHeight, setFloorHeight] = useState(""); // 바닥 높이

  // 변환된 좌표 상태
  const [transformedCoordinates, setTransformedCoordinates] = useState(null);

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
  }, []); // 초기 마운트 시에만 실행

  // 이미지 클릭 이벤트 핸들러 (최대 클릭 수 제한)
  const handleImageClick = (e, imgRef, setPoints, maxPoints) => {
    if (e.nativeEvent.which !== 1) return; // 왼쪽 마우스 버튼만 처리
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

  // 비디오 클릭 이벤트 핸들러 (바닥 좌표 요청)
  const handleVideoClick = (e, videoRef, imgId) => {
    const video = videoRef.current;
    const rect = video.getBoundingClientRect();
    const x = e.clientX - rect.left;
    const y = e.clientY - rect.top;
    const scaleX = video.videoWidth / rect.width;
    const scaleY = video.videoHeight / rect.height;
    const adjustedX = x * scaleX;
    const adjustedY = y * scaleY;

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

  // 이미지 업로드 및 바닥의 네 끝점 좌표 서버로 전송
  const handleUploadImages = () => {
    // 바닥 너비와 높이 값이 유효한지 확인
    if (!floorWidth || !floorHeight) {
      alert("바닥의 너비와 높이를 입력해주세요.");
      return;
    }
    const formData = new FormData();
    const image1Blob = dataURLtoBlob(camera0Image);
    const image2Blob = dataURLtoBlob(camera1Image);
    formData.append("image1", image1Blob, "camera0Image.jpg");
    formData.append("image2", image2Blob, "camera1Image.jpg");
    formData.append("pts1_floor", JSON.stringify(floorPoints1));
    formData.append("pts2_floor", JSON.stringify(floorPoints2));
    formData.append("floor_width", parseFloat(floorWidth));
    formData.append("floor_height", parseFloat(floorHeight));

    axios
      .post(`${PYTHON_URL}/upload_images/`, formData)
      .then((response) => {
        setStep(response.data.step);
        setImage1Src("data:image/jpeg;base64," + response.data.image1);
        setImage2Src("data:image/jpeg;base64," + response.data.image2);
      })
      .catch((error) => {
        console.error("이미지 업로드 에러:", error);
        alert("이미지 업로드 중 오류가 발생했습니다.");
      });
  };

  // // 이미지 업로드 함수
  // const handleUploadImages = () => {
  //   if (!floorWidth || !floorHeight) {
  //     alert("바닥의 너비와 높이를 입력해주세요.");
  //     return;
  //   }
  //   const formData = new FormData();
  //   const image1Blob = dataURLtoBlob(camera0Image);
  //   const image2Blob = dataURLtoBlob(camera1Image);
  //   // console.log("업로드", image1Blob);
  //   // console.log(image2Blob);
  //   // Blob 데이터가 정상적으로 변환되었는지 확인
  //   if (!image1Blob || !image2Blob) {
  //     alert("이미지를 처리하는 중 오류가 발생했습니다.");
  //     return;
  //   }
  //   formData.append("image1", image1Blob, "camera0Image.jpg");
  //   formData.append("image2", image2Blob, "camera1Image.jpg");
  //   formData.append("pts1_floor", JSON.stringify(floorPoints1));
  //   formData.append("pts2_floor", JSON.stringify(floorPoints2));
  //   formData.append("floor_width", parseFloat(floorWidth));
  //   formData.append("floor_height", parseFloat(floorHeight));

  //   axios
  //     .post(`${PYTHON_URL}/upload_images/`, formData)
  //     .then((response) => {
  //       console.log(response.data);
  //       const processedImage1 =
  //         "data:image/jpeg;base64," + response.data.image1;
  //       const processedImage2 =
  //         "data:image/jpeg;base64," + response.data.image2;

  //       // Step2 페이지로 전송
  //       navigate("/step2", {
  //         state: { image1Src: processedImage1, image2Src: processedImage2 },
  //       });
  //     })
  //     .catch((error) => {
  //       console.error("이미지 업로드 에러:", error);
  //       alert("이미지 업로드 중 오류가 발생했습니다.");
  //     });
  // };

  // dataURL을 Blob으로 변환하는 함수
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

  // 이미지 회전/반전 명령을 서버로 전송하는 함수
  const handleAdjustImages = (key) => {
    const formData = new FormData();
    formData.append("key", key);
    formData.append("img_id", selectedImage);

    axios
      .post(`${PYTHON_URL}/adjust_images/`, formData)
      .then((response) => {
        setStep(response.data.step);
        setImage1Src("data:image/jpeg;base64," + response.data.image1);
        setImage2Src("data:image/jpeg;base64," + response.data.image2);
      })
      .catch((error) => {
        console.error("이미지 조정 에러:", error);
        alert("이미지 조정 중 오류가 발생했습니다.");
      });
  };

  // 조정할 이미지를 선택하는 함수
  const handleSelectImage = (imgId) => {
    setSelectedImage(imgId);
  };

  // 타일 모서리 좌표를 서버로 전송하는 함수
  const handleUploadTilePoints = () => {
    const formData = new FormData();
    formData.append("pts1_tile", JSON.stringify(tilePoints1));
    formData.append("pts2_tile", JSON.stringify(tilePoints2));

    axios
      .post(`${PYTHON_URL}/upload_tile_points/`, formData)
      .then((response) => {
        setStep(response.data.step);
        setImage1Src("data:image/jpeg;base64," + response.data.image1);
        setImage2Src("data:image/jpeg;base64," + response.data.image2);
      })
      .catch((error) => {
        console.error("타일 포인트 업로드 에러:", error);
        alert("타일 포인트 업로드 중 오류가 발생했습니다.");
      });
  };

  // 대응점 좌표를 서버로 전송하는 함수
  const handleUploadAlignPoints = () => {
    const formData = new FormData();
    formData.append("pts1_align", JSON.stringify(alignPoints1));
    formData.append("pts2_align", JSON.stringify(alignPoints2));

    axios
      .post(`${PYTHON_URL}/upload_align_points/`, formData)
      .then((response) => {
        setStep(response.data.step);
        setMergedImageSrc(
          "data:image/jpeg;base64," + response.data.merged_image
        );
        // H1_total과 H2_total은 더 이상 프런트엔드에서 관리하지 않습니다.
      })
      .catch((error) => {
        console.error("대응점 업로드 에러:", error);
        alert("대응점 업로드 중 오류가 발생했습니다.");
      });
  };

  // 변환 행렬을 서버에 저장하는 함수
  const handleSaveTransformations = () => {
    if (!roomId) {
      alert("방 번호를 입력해주세요.");
      return;
    }

    if (!cameraId1 || !cameraId2) {
      alert("두 카메라 번호를 모두 입력해주세요.");
      return;
    }

    const saveTransformation = (cameraId, videoRef) => {
      const video = videoRef.current;
      const rect = video.getBoundingClientRect();
      const scaleX = video.videoWidth / rect.width; // 비디오 실제 너비 비율
      const scaleY = video.videoHeight / rect.height; // 비디오 실제 높이 비율

      const formData = new FormData();
      formData.append("room_id", roomId);
      formData.append("camera_id", cameraId);
      formData.append("scaleX", scaleX);
      formData.append("scaleY", scaleY);

      console.log("보내는 데이터:", roomId, cameraId, scaleX, scaleY); // 디버깅을 위해 추가

      axios
        .post(`${PYTHON_URL}/save_transformations/`, formData)
        .then((response) => {
          console.log(`Camera ${cameraId} 변환 행렬 저장 성공:`, response.data);
          alert(`Camera ${cameraId} 변환 행렬이 성공적으로 저장되었습니다.`);
        })
        .catch((error) => {
          console.error(`Camera ${cameraId} 변환 행렬 저장 에러:`, error);
          alert(`Camera ${cameraId} 변환 행렬 저장 중 오류가 발생했습니다.`);
        });
    };

    saveTransformation(cameraId1, videoDisplayRef1);
    saveTransformation(cameraId2, videoDisplayRef2);
  };

  // 좌표 변환을 서버에 요청하는 함수
  const handleTransformPoint = () => {
    if (
      !transformationCameraId ||
      transformationX === "" ||
      transformationY === ""
    ) {
      alert("카메라 번호와 X, Y 좌표를 모두 입력해주세요.");
      return;
    }

    // 로그 추가: 전송할 값 확인
    console.log(
      `보내는 데이터 - 카메라 번호: ${transformationCameraId}, X: ${transformationX}, Y: ${transformationY}`
    );

    const formData = new FormData();
    formData.append("camera_id", transformationCameraId);
    formData.append("x", parseFloat(transformationX));
    formData.append("y", parseFloat(transformationY));

    axios
      .post(`${PYTHON_URL}/transform_point/`, formData)
      .then((response) => {
        if (response.data.error) {
          alert(response.data.error);
        } else {
          setTransformedCoordinates({
            x_transformed: response.data.x_transformed,
            y_transformed: response.data.y_transformed,
          });
        }
      })
      .catch((error) => {
        console.error("좌표 변환 요청 에러:", error);
        alert("좌표 변환 요청 중 오류가 발생했습니다.");
      });
  };

  // 변환 좌표 입력 상태
  const [transformationCameraId, setTransformationCameraId] = useState("");
  const [transformationX, setTransformationX] = useState("");
  const [transformationY, setTransformationY] = useState("");

  return (
    <div className="App">
      <h1>현재 단계: {step}</h1>

      {/* 숨겨진 비디오 요소 (프레임 캡처용) */}
      <video
        ref={videoCaptureRef1}
        style={{ display: "none" }}
        preload="metadata"
      >
        {/* <source src={process.env.PUBLIC_URL + "/cctv1.mp4"} type="video/mp4" /> */}
        <source src="/assets/cctv2.mp4" type="video/mp4" />
      </video>
      <video
        ref={videoCaptureRef2}
        style={{ display: "none" }}
        preload="metadata"
      >
        {/* <source src={process.env.PUBLIC_URL + "/cctv2.mp4"} type="video/mp4" /> */}
        <source src="/assets/cctv1.mp4" type="video/mp4" />
      </video>

      {step === 1 && (
        <div>
          <h2>1. 이미지에서 바닥의 네 끝점 선택</h2>
          <div style={{ display: "flex", flexDirection: "row", gap: "20px" }}>
            <div>
              <h3>이미지 1</h3>
              <img
                src={camera0Image}
                alt="캡처된 이미지 1"
                ref={imageRef1}
                style={{
                  cursor: floorPoints1.length < 4 ? "crosshair" : "default",
                  maxWidth: "100%",
                  height: "auto",
                }}
                onClick={(e) =>
                  handleImageClick(e, imageRef1, setFloorPoints1, 4)
                }
              />
              <p>선택한 포인트 수: {floorPoints1.length} / 4</p>
            </div>
            <div>
              <h3>이미지 2</h3>
              <img
                src={camera1Image}
                alt="캡처된 이미지 2"
                ref={imageRef2}
                style={{
                  cursor: floorPoints2.length < 4 ? "crosshair" : "default",
                  maxWidth: "100%",
                  height: "auto",
                }}
                onClick={(e) =>
                  handleImageClick(e, imageRef2, setFloorPoints2, 4)
                }
              />
              <p>선택한 포인트 수: {floorPoints2.length} / 4</p>
            </div>
          </div>

          <div>
            <label>
              바닥 너비 (floor width):
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
              바닥 높이 (floor height):
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
            disabled={floorPoints1.length < 4 || floorPoints2.length < 4}
          >
            이미지 업로드 및 다음 단계로 진행
          </button>
        </div>
      )}

      {step === 2 && (
        <div>
          <h2>2. 이미지 회전/반전</h2>
          <div style={{ marginBottom: "10px" }}>
            <button onClick={() => handleSelectImage(1)}>이미지 1 선택</button>
            <button
              onClick={() => handleSelectImage(2)}
              style={{ marginLeft: "10px" }}
            >
              이미지 2 선택
            </button>
            <p>현재 선택된 이미지: 이미지 {selectedImage}</p>
          </div>
          <div style={{ marginBottom: "10px" }}>
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
              onClick={() => handleAdjustImages("n")}
              style={{ marginLeft: "10px" }}
            >
              다음 단계로 진행
            </button>
          </div>
          <div style={{ display: "flex", flexDirection: "row", gap: "20px" }}>
            <div>
              <h3>변환된 이미지 1</h3>
              {image1Src && (
                <img
                  src={image1Src}
                  alt="변환된 이미지 1"
                  style={{ maxWidth: "100%", height: "auto" }}
                />
              )}
            </div>
            <div>
              <h3>변환된 이미지 2</h3>
              {image2Src && (
                <img
                  src={image2Src}
                  alt="변환된 이미지 2"
                  style={{ maxWidth: "100%", height: "auto" }}
                />
              )}
            </div>
          </div>
        </div>
      )}

      {step === 3 && (
        <div>
          <h2>3. 타일 모서리 선택</h2>
          <div style={{ display: "flex", flexDirection: "row", gap: "20px" }}>
            <div>
              <h3>이미지 1</h3>
              {image1Src && (
                <img
                  src={image1Src}
                  alt="타일 선택 이미지 1"
                  ref={imageRef1}
                  style={{
                    cursor: tilePoints1.length < 4 ? "crosshair" : "default",
                    maxWidth: "100%",
                    height: "auto",
                  }}
                  onClick={(e) =>
                    handleImageClick(e, imageRef1, setTilePoints1, 4)
                  }
                />
              )}
              <p>선택한 포인트 수: {tilePoints1.length} / 4</p>
            </div>
            <div>
              <h3>이미지 2</h3>
              {image2Src && (
                <img
                  src={image2Src}
                  alt="타일 선택 이미지 2"
                  ref={imageRef2}
                  style={{
                    cursor: tilePoints2.length < 4 ? "crosshair" : "default",
                    maxWidth: "100%",
                    height: "auto",
                  }}
                  onClick={(e) =>
                    handleImageClick(e, imageRef2, setTilePoints2, 4)
                  }
                />
              )}
              <p>선택한 포인트 수: {tilePoints2.length} / 4</p>
            </div>
          </div>
          <button
            onClick={handleUploadTilePoints}
            disabled={tilePoints1.length < 4 || tilePoints2.length < 4}
          >
            타일 포인트 업로드 및 다음 단계로 진행
          </button>
        </div>
      )}

      {step === 4 && (
        <div>
          <h2>4. 이미지 합성을 위한 대응점 선택</h2>
          <div style={{ display: "flex", flexDirection: "row", gap: "20px" }}>
            <div>
              <h3>이미지 1</h3>
              {image1Src && (
                <img
                  src={image1Src}
                  alt="대응점 선택 이미지 1"
                  ref={imageRef1}
                  style={{
                    cursor: alignPoints1.length < 4 ? "crosshair" : "default",
                    maxWidth: "100%",
                    height: "auto",
                  }}
                  onClick={(e) =>
                    handleImageClick(e, imageRef1, setAlignPoints1, 4)
                  }
                />
              )}
              <p>선택한 포인트 수: {alignPoints1.length} / 4</p>
            </div>
            <div>
              <h3>이미지 2</h3>
              {image2Src && (
                <img
                  src={image2Src}
                  alt="대응점 선택 이미지 2"
                  ref={imageRef2}
                  style={{
                    cursor: alignPoints2.length < 4 ? "crosshair" : "default",
                    maxWidth: "100%",
                    height: "auto",
                  }}
                  onClick={(e) =>
                    handleImageClick(e, imageRef2, setAlignPoints2, 4)
                  }
                />
              )}
              <p>선택한 포인트 수: {alignPoints2.length} / 4</p>
            </div>
          </div>
          <button
            onClick={handleUploadAlignPoints}
            disabled={alignPoints1.length < 4 || alignPoints2.length < 4}
          >
            대응점 업로드 및 다음 단계로 진행
          </button>
        </div>
      )}

      {step === 5 && (
        <div>
          <h2>5. 영상에서 바닥 좌표 확인 및 변환 행렬 저장</h2>
          <p>비디오를 클릭하여 바닥 좌표를 확인하세요.</p>

          {/* 방 번호 및 카메라 ID 입력 */}
          <div style={{ marginBottom: "20px" }}>
            <h3>변환 행렬 저장</h3>
            <div>
              <label>
                방 번호:
                <input
                  type="text"
                  value={roomId}
                  onChange={(e) => setRoomId(e.target.value)}
                  style={{ marginLeft: "10px" }}
                />
              </label>
            </div>
            <div style={{ marginTop: "10px" }}>
              <label>
                카메라 1 번호:
                <input
                  type="number"
                  value={cameraId1}
                  onChange={(e) => setCameraId1(e.target.value)}
                  style={{ marginLeft: "10px" }}
                />
              </label>
            </div>
            <div style={{ marginTop: "10px" }}>
              <label>
                카메라 2 번호:
                <input
                  type="number"
                  value={cameraId2}
                  onChange={(e) => setCameraId2(e.target.value)}
                  style={{ marginLeft: "10px" }}
                />
              </label>
            </div>
            <button
              onClick={handleSaveTransformations}
              style={{ marginTop: "10px" }}
            >
              변환 행렬 저장
            </button>
          </div>

          {/* 좌표 변환 섹션 추가 */}
          <div style={{ marginBottom: "20px" }}>
            <h3>좌표 변환</h3>
            <div>
              <label>
                카메라 번호:
                <input
                  type="number"
                  value={transformationCameraId}
                  onChange={(e) => setTransformationCameraId(e.target.value)}
                  style={{ marginLeft: "10px" }}
                />
              </label>
            </div>
            <div style={{ marginTop: "10px" }}>
              <label>
                X 좌표:
                <input
                  type="number"
                  value={transformationX}
                  onChange={(e) => setTransformationX(e.target.value)}
                  style={{ marginLeft: "10px" }}
                />
              </label>
            </div>
            <div style={{ marginTop: "10px" }}>
              <label>
                Y 좌표:
                <input
                  type="number"
                  value={transformationY}
                  onChange={(e) => setTransformationY(e.target.value)}
                  style={{ marginLeft: "10px" }}
                />
              </label>
            </div>
            <button
              onClick={handleTransformPoint}
              style={{ marginTop: "10px" }}
            >
              좌표 변환
            </button>

            {transformedCoordinates && (
              <div style={{ marginTop: "10px" }}>
                <h4>변환된 좌표:</h4>
                <p>X: {transformedCoordinates.x_transformed.toFixed(2)}</p>
                <p>Y: {transformedCoordinates.y_transformed.toFixed(2)}</p>
              </div>
            )}
          </div>

          <div
            style={{
              display: "flex",
              flexDirection: "row",
              gap: "20px",
              alignItems: "flex-start",
            }}
          >
            <div>
              <h3>영상 1</h3>
              {/* <video
                ref={videoDisplayRef1}
                src={process.env.PUBLIC_URL + "/cctv1.mp4"}
                controls
                style={{
                  maxWidth: "100%",
                  height: "auto",
                  cursor: "crosshair",
                }}
                onClick={(e) => handleVideoClick(e, videoDisplayRef1, 1)}
              /> */}
              <img
                src={camera0Image}
                alt="Camera 1"
                ref={videoDisplayRef1}
                style={{
                  cursor: floorPoints1.length < 4 ? "crosshair" : "default",
                  maxWidth: "100%",
                  height: "auto",
                }}
                onClick={(e) => handleVideoClick(e, videoDisplayRef1, 1)}
              />
            </div>
            <div>
              <h3>영상 2</h3>
              {/* <video
                ref={videoDisplayRef2}
                src={process.env.PUBLIC_URL + "/cctv2.mp4"}
                controls
                style={{
                  maxWidth: "100%",
                  height: "auto",
                  cursor: "crosshair",
                }}
                onClick={(e) => handleVideoClick(e, videoDisplayRef2, 2)}
              /> */}
              <img
                src={camera1Image}
                alt="Camera 1"
                ref={videoDisplayRef2}
                style={{
                  cursor: floorPoints2.length < 4 ? "crosshair" : "default",
                  maxWidth: "100%",
                  height: "auto",
                }}
                onClick={(e) => handleVideoClick(e, videoDisplayRef2, 2)}
              />
            </div>
            <div>
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
        </div>
      )}
    </div>
  );
}

export default Testpage;
