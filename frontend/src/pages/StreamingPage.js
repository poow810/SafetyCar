// import ReactDOM from "react-dom/client";

// import React, { useState, useEffect } from "react";

// const WEBSOCKET_URL = process.env.REACT_APP_WEBSOCKET_URL;
// // import { Manager } from "socket.io-client";
// // function Axios() {
// //   const instance = axios.create({
// //     baseURL: "http://localhost:8080",
// //     // headers: {
// //     //   'Content-Type': 'application/json;charset=utf-8',
// //     // },
// //   });

// //   return instance;
// // }

// // let imgurl;

// // async function readFrame() {
// //   const res = await Axios().get('/0');
// //   imgurl = URL.createObjectURL(res.blob());

// // }

// // function InfiniteFrameRenderer() {
// //   const [frameKey, setFrameKey] = useState(0);

// //   useEffect(() => {
// //     // 1초마다 key를 변경하여 Frame을 다시 렌더링
// //     const interval = setInterval(() => {
// //       setFrameKey(prevKey => prevKey + 1);
// //     }, 10); // 1000ms = 1초

// //     // 컴포넌트 언마운트 시 타이머 정리
// //     return () => clearInterval(interval);
// //   }, []);

// //   return (
// //     <div>
// //       <Frame key={frameKey} /> {/* key를 변경하면 컴포넌트가 리렌더링됨 */}
// //     </div>
// //   );
// // }

// const ws = new WebSocket(WEBSOCKET_URL);

// function ShowCCTV() {
//   const [frameSrc, setFrameSrc] = useState(null);

//   ws.onmessage = function (msg) {
//     setFrameSrc(URL.createObjectURL(msg.data));
//     // console.log(imageSrc);
//   };

//   // useEffect(() => {
//   //   return <img src={frameSrc} alt="CCTV" />
//   // })

//   return <img src={frameSrc} alt="CCTV" />;
// }

// function StreamingPage() {
//   const [frameSrcArr, setFrameSrcArr] = useState([null, null, null, null]);

//   // 이미지 저장 함수 (Blob -> Base64로 변환 후 Local Storage에 저장)
//   const saveFrameToLocalStorage = (blob, cameraIndex) => {
//     const reader = new FileReader();
//     reader.readAsDataURL(blob); // Blob을 Base64로 변환
//     reader.onloadend = function () {
//       const base64Data = reader.result;
//       localStorage.setItem(`savedImageCamera${cameraIndex}`, base64Data); // 카메라 인덱스에 맞게 저장
//     };
//   };

//   ws.onmessage = async function (msg) {
//     let newArr = [...frameSrcArr];
//     const int8Array = new Int8Array(await msg.data.slice(0, 1).arrayBuffer());
//     const idx = int8Array[0];
//     newArr[idx] = URL.createObjectURL(msg.data.slice(1));
//     setFrameSrcArr(newArr);
//     const blob = new Blob([msg.data.slice(1)], { type: "image/jpeg" });
//     const blobUrl = URL.createObjectURL(blob);
//     newArr[idx] = blobUrl;

//     // 각 카메라 프레임을 Local Storage에 저장
//     if (idx === 0) {
//       saveFrameToLocalStorage(blob, 0); // 카메라 0번의 프레임을 저장
//     } else if (idx === 1) {
//       saveFrameToLocalStorage(blob, 1); // 카메라 1번의 프레임을 저장
//     }
//   };

//   return (
//     <div>
//       <img src={frameSrcArr[0]} alt="CCTV 0"></img>
//       <img src={frameSrcArr[1]} alt="CCTV 1"></img>
//     </div>
//   );
// }

// export default StreamingPage;
