1. 가상 환경 만들기
   ```conda create -n ADROIT python=3.10 ```
2. 가상 환경 실행
   ```conda activate ADROIT```
3. 깃 클론 하기
   ```git clone https://github.com/samuelp4065/FR_lerobot.git && cd FR_lerobot```
4. lerobot 다운 받기
   ```cd /lerobot && pip install -e .```
5. 

6. 르로봇 포트 사용 허가 해주기
   ``` sudo usermod -a -G dialout $USER && sudo chmod 666 /dev/ttyACM*```
#. teleoperation 실행
  ```cd /FR_lerobot/lerobot```
```
lerobot-teleoperate
--robot.type=fairino_follower
--robot.controller_ip=192.168.58.2
--robot.velocity=20
--robot.acceleration=5
--teleop.use_degree=True
--teleop.type=so101_leader
--teleop.id=leader
--teleop.port=/dev/ttyACM0
--teleop.use_degrees=True
```



