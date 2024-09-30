import ReactDOM from "react-dom/client";
// import './index.css';
import App from "./App";
// import Frame from './ReadFrame';
// import reportWebVitals from './reportWebVitals';
// import axios from 'axios';
// import { io } from 'socket.io-client'
import React, { useState, useEffect } from "react";

const WEBSOCKET_URL = process.env.REACT_APP_WEBSOCKET_URL;
// import { Manager } from "socket.io-client";
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

const ws = new WebSocket(WEBSOCKET_URL);

function ShowCCTV() {
  const [frameSrc, setFrameSrc] = useState(null);

  ws.onmessage = function (msg) {
    setFrameSrc(URL.createObjectURL(msg.data));
    // console.log(imageSrc);
  };

  // useEffect(() => {
  //   return <img src={frameSrc} alt="CCTV" />
  // })

  return <img src={frameSrc} alt="CCTV" />;
}

function ShowCCTV2() {
  const [frameSrcArr, setFrameSrcArr] = useState([null,null,null,null]);
  
  ws.onmessage = function(msg) {
    let newArr = [...frameSrcArr];
    // const idx = msg.data.slice(0,1).arrayBuffer().getInt8(0);
    const int8Array = new Int8Array(msg.data.slice(0,1).arrayBuffer());
    const idx = int8Array[0];
    
    newArr[idx] = URL.createObjectURL(msg.data.slice(1));
    setFrameSrcArr(newArr);
  };
  

  return <div>
    <img src={frameSrcArr[0]} alt="CCTV 0"></img>
    <img src={frameSrcArr[1]} alt="CCTV 1"></img>
    </div> 

}

const root = ReactDOM.createRoot(document.getElementById("root"));
root.render(
  <React.StrictMode>
    <App />
    {/* <button onClick={readFrame}>frame</button> */}
    {/* <InfiniteFrameRenderer/> */}
    {/* <ShowCCTV /> */}
    <ShowCCTV2/>
  </React.StrictMode>
);

// If you want to start measuring performance in your app, pass a function
// to log results (for example: reportWebVitals(console.log))
// or send to an analytics endpoint. Learn more: https://bit.ly/CRA-vitals
