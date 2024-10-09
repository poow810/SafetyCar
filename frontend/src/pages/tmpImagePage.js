import React from 'react';
import "../styles/LoginPage.css";


function LoginPage() {
  // 이미지 파일을 Base64로 변환하는 함수
  const convertToBase64 = (file) => {
    return new Promise((resolve, reject) => {
      const reader = new FileReader();
      reader.readAsDataURL(file);
      reader.onload = () => resolve(reader.result);
      reader.onerror = (error) => reject(error);
    });
  };

  // 버튼 클릭 시 실행되는 함수
  const handleSaveImages = async () => {
    try {
      // 이미지 URL 설정 (public 폴더 기준)
      const cctv1Url = `${process.env.PUBLIC_URL}/assets/cctv1.jpg`;
      const cctv2Url = `${process.env.PUBLIC_URL}/assets/cctv2.jpg`;

      // 이미지 파일을 fetch API를 사용하여 가져오기
      const response1 = await fetch(cctv1Url);
      if (!response1.ok) {
        throw new Error(`Failed to fetch ${cctv1Url}: ${response1.statusText}`);
      }
      const blob1 = await response1.blob();
      const base64Image1 = await convertToBase64(blob1);

      const response2 = await fetch(cctv2Url);
      if (!response2.ok) {
        throw new Error(`Failed to fetch ${cctv2Url}: ${response2.statusText}`);
      }
      const blob2 = await response2.blob();
      const base64Image2 = await convertToBase64(blob2);

      // localStorage에 저장
      localStorage.setItem("savedImageCamera0", base64Image1);
      localStorage.setItem("savedImageCamera1", base64Image2);

      alert("이미지가 성공적으로 저장되었습니다!");
    } catch (error) {
      console.error("이미지 저장 중 오류 발생:", error);
      alert("이미지 저장 중 오류가 발생했습니다. 콘솔을 확인하세요.");
    }
  };

  return (
    <div className="login-container">
      <div className="loginbox">
        <h1 className="title">이미지 저장</h1>
        <button className="save-button" onClick={handleSaveImages}>
          이미지 저장
        </button>
      </div>
    </div>
  );
}

export default LoginPage;
