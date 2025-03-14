This project is based on optitrack, ROS-Noetic, ROS-Humble and Franka Panda Robot. Aim of it is trying to track a thrown ball with optitrack camera system and moving robot arm to predicted falling position and catch the ball.

This work is contributed by [Yuzhe Ding](https://github.com/xdd0225), [Shengping Yu](https://github.com/ShengpingY) and [Yuedong Zhai](https://github.com/kervinzhai) under supervision from Philipp Holzmann, Alexander Rose at [Control and Cyber-Physical Systems Laboratory (CCPS)](https://www.ccps.tu-darmstadt.de/ccps/index.en.jsp) of TU Darmstadt.


Overall there are three parts of the this project:
  1. [Parabolic prediction of the ball.](https://github.com/ShengpingY/Catching-Flying-Object-with-Franka-robot/blob/curvefit/test_with_publisher_and_iterative_curvfitting3D.m)
  2. [Coordinate Tranformation from Motioncapture system to robot system.](https://github.com/ShengpingY/Catching-Flying-Object-with-Franka-robot/tree/Master/src/camera_robot_tf)
  3. [Robot arm trajectory plan.](https://github.com/ShengpingY/Catching-Flying-Object-with-Franka-robot/tree/Master/src/trajectoryplan)
  4. [Motion controller based on trajectory.](https://github.com/ShengpingY/Catching-Flying-Object-with-Franka-robot/tree/Master/src/robotmotion)

![Gif](https://github.com/user-attachments/assets/2dd094c9-e89b-47ca-9866-7caa66b4d123)

Final Catching result looklike this:
  [Click to watch video](https://private-user-images.githubusercontent.com/47857328/422783083-3bd93af9-8194-4de9-be99-b6a0fbd48d07.mp4?jwt=eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9.eyJpc3MiOiJnaXRodWIuY29tIiwiYXVkIjoicmF3LmdpdGh1YnVzZXJjb250ZW50LmNvbSIsImtleSI6ImtleTUiLCJleHAiOjE3NDE5NTU0MzQsIm5iZiI6MTc0MTk1NTEzNCwicGF0aCI6Ii80Nzg1NzMyOC80MjI3ODMwODMtM2JkOTNhZjktODE5NC00ZGU5LWJlOTktYjZhMGZiZDQ4ZDA3Lm1wND9YLUFtei1BbGdvcml0aG09QVdTNC1ITUFDLVNIQTI1NiZYLUFtei1DcmVkZW50aWFsPUFLSUFWQ09EWUxTQTUzUFFLNFpBJTJGMjAyNTAzMTQlMkZ1cy1lYXN0LTElMkZzMyUyRmF3czRfcmVxdWVzdCZYLUFtei1EYXRlPTIwMjUwMzE0VDEyMjUzNFomWC1BbXotRXhwaXJlcz0zMDAmWC1BbXotU2lnbmF0dXJlPWZkZWZmMWJlOWEyOTU5OTZjMzA1MGJkODZmODkxOTUyMzViMDUwMTMxODE2YjdhODVlN2ZlNDRlYjdkN2U1ZTUmWC1BbXotU2lnbmVkSGVhZGVycz1ob3N0In0.bPz2PEhUYdh8nphtek6p6Ki0UQC6BFiHL4lzCT2m_E8)

