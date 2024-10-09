import React from 'react';
import { useNavigate } from 'react-router-dom'; // useNavigate 훅을 import
import '../styles/navibar.css'; // CSS 파일 import

const Navbar = () => {
  const navigate = useNavigate(); // navigate 함수 생성

  return (
    <div className="navbar">
      <div className="nav-item" onClick={() => navigate('/')}>
        <img src="/assets/navi/Icon.png" alt="Home" className="nav-icon" /> {/* Home 아이콘 */}
      </div>
      <div className="nav-item" onClick={() => navigate('/step1')}>
        <img src="/assets/navi/Settings.png" alt="Settings" className="nav-icon" /> {/* Settings 아이콘 */}
      </div>
      <div className="nav-item" onClick={() => navigate('/image')}>
        <img src="/assets/navi/Mail.png" alt="Mail" className="nav-icon" /> {/* Mail 아이콘 */}
      </div>
      <div className="nav-item" onClick={() => navigate('/rotate')}>
        <img src="/assets/navi/Rotate cw (1).png" alt="Rotate" className="nav-icon" /> {/* Rotate 아이콘 */}
      </div>
    </div>
  );
};

export default Navbar;
