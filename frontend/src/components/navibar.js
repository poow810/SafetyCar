import React from 'react';
import { useNavigate } from 'react-router-dom'; // useNavigate 훅을 import
import '../styles/navibar.css'; // CSS 파일 import
import axios from 'axios';

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

  return (
    <div className="navbar">
      <div className="nav-item" onClick={() => navigate('/')}>
        <img src="/assets/navi/Icon.png" alt="Home" className="nav-icon" /> {/* Home 아이콘 */}
      </div>
      <div className="nav-item" onClick={() => navigate('/step1')}>
        <img src="/assets/navi/Settings.png" alt="Settings" className="nav-icon" /> {/* Settings 아이콘 */}
      </div>
      <div className="nav-item">
        <img src="/assets/navi/Mail.png" alt="Mail" className="nav-icon" /> {/* Mail 아이콘 */}
      </div>
      <div className="nav-item" onClick={handleRotateClick}>
        <img src="/assets/navi/Rotate cw (1).png" alt="Rotate" className="nav-icon" /> {/* Rotate 아이콘 */}
      </div>
    </div>
  );
};

export default Navbar;