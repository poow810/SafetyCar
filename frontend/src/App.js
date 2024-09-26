import React, { useEffect, useState } from 'react';
import { v4 as uuidv4 } from 'uuid'; // UUID를 가져옵니다.
import { BrowserRouter as Router, Route, Link, Routes } from 'react-router-dom';
import NewPage from './Newpage';
import './App.css';

function App() {
  const [messages, setMessages] = useState([]);
  const [messageCount, setMessageCount] = useState(0); // 메시지 수 상태 추가
  const clientId = uuidv4(); // 고유한 클라이언트 ID를 생성합니다.

  useEffect(() => {
    let eventSource;

    const connectEventSource = () => {
      eventSource = new EventSource(`http://localhost:8080/api/sse?clientId=${clientId}`); // 클라이언트 ID 포함

      eventSource.onmessage = (event) => {
        const newMessage = event.data;
        console.log("수신한 메시지:", newMessage); // 수신한 메시지 로그 출력
        setMessages((prevMessages) => [...prevMessages, newMessage]);
        setMessageCount((prevCount) => prevCount + 1); // 메시지 수 업데이트
      };

      eventSource.onerror = (error) => {
        console.error('SSE error:', error);
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
    <Router> {/* Router로 감싸기 */}
        <header className="App-header">
          <h1>알림</h1>
          <p>수신한 메시지 수: {messageCount}</p> {/* 메시지 수 표시 */}
          <ul>
            {messages.map((msg, index) => (
              <li key={index} style={{ color: msg.includes("경고") ? 'red' : 'black' }}>
                {msg}
              </li>             
            ))}
          </ul>
          <nav>
            <Link to="/">홈</Link>
            <Link to="/new">새로운 페이지</Link>
          </nav>
        </header>
      <div className="App">
        <Routes>
          <Route path="/" element={<h2>홈 페이지</h2>} />
          <Route path="/new" element={<NewPage />} /> {/* NewPage 컴포넌트 */}
        </Routes>
      </div>
    </Router>
  );
}

export default App;
