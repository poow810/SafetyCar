import React, { useEffect, useState } from "react";
import { v4 as uuidv4 } from "uuid"; // UUID를 가져옵니다.
import "./App.css";
import { BrowserRouter as Router, Route, Routes } from "react-router-dom";
import LoginPage from "./pages/LoginPage";
import HomePage from "./pages/MainPage";
import LibraryPage from "./pages/LibraryPage";
import "./styles/global.css";

function App() {
  const [messages, setMessages] = useState([]);
  const [messageCount, setMessageCount] = useState(0); // 메시지 수 상태 추가
  const clientId = uuidv4(); // 고유한 클라이언트 ID를 생성합니다.

  useEffect(() => {
    let eventSource;

    const connectEventSource = () => {
      eventSource = new EventSource(
        `http://localhost:8080/api/sse?clientId=${clientId}`
      ); // 클라이언트 ID 포함

      eventSource.onmessage = (event) => {
        const newMessage = event.data;
        console.log("수신한 메시지:", newMessage); // 수신한 메시지 로그 출력
        setMessages((prevMessages) => [...prevMessages, newMessage]);
        setMessageCount((prevCount) => prevCount + 1); // 메시지 수 업데이트
      };

      eventSource.onerror = (error) => {
        console.error("SSE error:", error);
        eventSource.close();
        // 재연결 시도
        setTimeout(connectEventSource, 1000); // 1초 후 재연결
      };
    };

    connectEventSource(); // 초기 연결

    return () => {
      if (eventSource) {
        eventSource.close(); // 컴포넌트 언마운트 시 연결 종료
      }
    };
  }, []);

  return (
    <Router>
      <Routes>
        <Route path="/login" element={<LoginPage />} />
        <Route path="/home" element={<HomePage />} />
        <Route path="/library" element={<LibraryPage />} />
      </Routes>
    </Router>
  );
}

export default App;
