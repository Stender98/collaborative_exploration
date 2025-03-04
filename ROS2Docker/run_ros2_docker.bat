@echo off
REM Start VcXsrv if not already running
tasklist | find /i "vcxsrv.exe" > nul || start "" "C:\Program Files\VcXsrv\vcxsrv.exe" -multiwindow -ac

REM Wait a bit to ensure X Server starts
timeout /t 3 /nobreak > nul

REM Set DISPLAY environment variable for Docker
set DISPLAY=host.docker.internal:0.0

REM Run ROS 2 Jazzy Docker container with X11 forwarding and networking
docker run -it -p 5005:5005 --net=host --name ros2_slam ^
    -e DISPLAY=%DISPLAY% ^
    -e QT_X11_NO_MITSHM=1 ^
    -v %CD%:/workspace ^
    ros2-slam-container
