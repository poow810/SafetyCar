import React, { useState } from "react";
import "../styles/Input.css"; // CSS 파일 임포트

function InputForm({ type, label }) {
  const [isFocused, setIsFocused] = useState(false); // focus 상태 관리

  return (
    <div className={`input-container ${isFocused ? "focused" : ""}`}>
      {/* <label>{label}</label> */}
      <input
        type={type}
        placeholder={label}
        onFocus={() => setIsFocused(true)} // input에 focus가 되었을 때
        onBlur={() => setIsFocused(false)} // input에서 focus가 벗어났을 때
      />
    </div>
  );
}

export default InputForm;
