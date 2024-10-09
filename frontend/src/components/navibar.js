import React, { useState } from "react";
import { useNavigate } from "react-router-dom"; // useNavigate 훅을 import
import "../styles/navibar.css"; // CSS 파일 import
import axios from "axios";

const Navbar = () => {
  const navigate = useNavigate(); 
  const PYTHON_URL = process.env.REACT_APP_PYTHON_URL; 

  const handleRotateClick = async () => {
    try {
      const response = await axios.get(`${PYTHON_URL}/safety_Car/halt`);
      console.log('Response:', response.data);
    } catch (error) {
      console.error('Error making GET request:', error);
      alert('요청 처리 중 오류가 발생했습니다.');
    }
  };

  const handleSendSMS = () => {
    const space = "삼성화재 유성캠퍼스(SSAFY 교육동)";
    const API_URL = process.env.REACT_APP_API_URL;
    axios
      .post(`${API_URL}/sms/send?space=${space}`)
      .then((response) => {
        console.log(response.data.message);
        console.log("신고 성공");
        if (response.data.message) {
          console.log(response.data);
          console.log("신고가 접수 완료");
          alert("신고가 성공적으로 접수되었습니다.");
        }
      })
      .catch((error) => {
        console.error("신고 접수 중 에러 발생", error);
        alert("신고에 실패했습니다. 다시 시도해주세요.");
      });
  };

  const [showModal, setShowModal] = useState(false); // 모달 창 표시 여부

  const handleOpenModal = () => {
    setShowModal(true); // 모달 창 열기
  };

  const handleCloseModal = () => {
    setShowModal(false); // 모달 창 닫기
  };
  return (
    <div className="navbar">
      <div className="nav-item" onClick={() => navigate("/")}>
        <img src="/assets/navi/Icon.png" alt="Home" className="nav-icon" />{" "}
        {/* Home 아이콘 */}
      </div>
      <div className="nav-item" onClick={() => navigate("/step1")}>
        <img
          src="/assets/navi/Settings.png"
          alt="Settings"
          className="nav-icon"
        />
        {/* Settings 아이콘 */}
      </div>
      <div className="nav-item" onClick={handleOpenModal}>
        <img src="/assets/navi/Mail.png" alt="Mail" className="nav-icon" />{" "}
        {/* Mail 아이콘 */}
      </div>
      <div className="nav-item" onClick={() => navigate("/rotate")}>
        <img
          src="/assets/navi/Rotate cw (1).png"
          alt="Rotate"
          className="nav-icon"
        />
        {/* Rotate 아이콘 */}
      </div>

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
  );
};

export default Navbar;