import ReactDOM from 'react-dom/client';
import './index.css';
import App from './App';
// import Frame from './ReadFrame';
import reportWebVitals from './reportWebVitals';
import axios from 'axios';
import { io } from 'socket.io-client'
import React, { useState, useEffect } from 'react';
import { Manager } from "socket.io-client";
// function Axios() {
//   const instance = axios.create({
//     baseURL: "http://localhost:8080",
//     // headers: {
//     //   'Content-Type': 'application/json;charset=utf-8',
//     // },
//   });

//   return instance;
// }

// let imgurl;

// async function readFrame() {
//   const res = await Axios().get('/0');
//   imgurl = URL.createObjectURL(res.blob());

// }

// function InfiniteFrameRenderer() {
//   const [frameKey, setFrameKey] = useState(0);

//   useEffect(() => {
//     // 1초마다 key를 변경하여 Frame을 다시 렌더링
//     const interval = setInterval(() => {
//       setFrameKey(prevKey => prevKey + 1);
//     }, 10); // 1000ms = 1초

//     // 컴포넌트 언마운트 시 타이머 정리
//     return () => clearInterval(interval);
//   }, []);

//   return (
//     <div>
//       <Frame key={frameKey} /> {/* key를 변경하면 컴포넌트가 리렌더링됨 */}
//     </div>
//   );
// }

const ws = new WebSocket("wss://j11b209.p.ssafy.io/api/socket");
function ShowCCTV() {
  const [frameSrc, setFrameSrc] = useState(null);
 
  ws.onmessage = function(msg) {
    setFrameSrc(URL.createObjectURL(msg.data));
    // console.log(imageSrc);
  }

  // useEffect(() => {
  //   return <img src={frameSrc} alt="CCTV" />
  // })

  return (
    <img src={frameSrc} alt="CCTV" />
  );
}


const root = ReactDOM.createRoot(document.getElementById("root"));
root.render(
  <React.StrictMode>
    {/* <App /> */}
    {/* <button onClick={readFrame}>frame</button> */}
    {/* <InfiniteFrameRenderer/> */}
    <ShowCCTV/>
  </React.StrictMode>
);

// If you want to start measuring performance in your app, pass a function
// to log results (for example: reportWebVitals(console.log))
// or send to an analytics endpoint. Learn more: https://bit.ly/CRA-vitals
