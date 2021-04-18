# Хакатон UavProf AquilaUAV 

[UavProf](https://uavprof.ru/) - Крупный Всероссийский хакатон по управлению мультиагентной системой БПЛА.



### Установка

```bash
cd ~/catkin_ws/src

git clone https://github.com/AquilaUAV/drone-games_ws.git

cd ~/catkin_ws/src/drone-games_ws

bash install.sh

cd ~/catkin_ws

catkin build
```



### Запуск



**Решение Рой**: swarm_race_1.rc

```bash
sim="race"
type="mc"

world="race_1.world"
num=12

team="AquilaUAV"
u_cmd="roslaunch drone-games_ws step_line_trajectory_flight.launch"
```

