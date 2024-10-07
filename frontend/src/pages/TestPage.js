import React from "react";
import "../styles/step.css";

function TestPage() {
  return (
    <>
      <h1>STEP1. 바닥 모서리 검출</h1>
      <div className="container">
        <div className="nav-container">
          <ul>
            <li>홈</li>
            <li>바닥 검출</li>
            <li>사건 기록</li>
          </ul>
        </div>
        <div className="right-container">
          <div className="image-container">
            <div className="image-box">
              <img src="image1.jpg" alt="CCTV 1" />
              <p>Image1 0/4</p>
            </div>
            <div className="image-box">
              <img src="image2.jpg" alt="CCTV 2" />
              <p>Image2 0/4</p>
            </div>
          </div>
          <div className="option-container">
            <div className="input-box">
              <label>바닥 너비 (Floor Width)</label>
              <input type="text" />
            </div>
            <div className="input-box">
              <label>바닥 높이 (Floor Height)</label>
              <input type="text" />
            </div>
            <div className="input-box">
              <label>방 번호 (roomId)</label>
              <input type="text" />
            </div>
            <button className="submit-btn">저장</button>
          </div>
        </div>
      </div>
    </>
  );
}

export default TestPage;
