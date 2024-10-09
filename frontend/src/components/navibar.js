import React from 'react';
import '../styles/navibar.css'; // CSS 파일 import

const Navbar = () => {
  return (
    <div className="navbar">
      <div className="nav-item">
        <div className="nav-icon">🏠</div> {/* Home 아이콘 */}
      </div>
      <div className="nav-item">
        <div className="nav-icon">⚙️</div> {/* Settings 아이콘 */}
      </div>
      <div className="nav-item">
        <div className="nav-icon">📁</div> {/* Folder 아이콘 */}
      </div>
      <div className="nav-item">
        <div className="nav-icon">📸</div> {/* Instagram 아이콘 */}
      </div>
      <div className="nav-item">
        <div className="nav-icon">✉️</div> {/* Mail 아이콘 */}
      </div>
    </div>
  );
};

export default Navbar;
