RRT* on the F1/10 autonomous racing car.  

Implementation of RRT* on the ROS F1/10 autonomous racing car.  


GUIDA PER ESEGUIRE :

Una volta lanciata la scena su Unity dare i seguenti comandi da terminale

```bash 
 cd path/to/rrt_star/map
```
```bash
 rosrun map_server map_server map.yaml
```


Nella dir principale, lanciare il seguente comando da terminale :

```bash
  roslaunch rrt rrt_node.launch
```

Recarsi ora nella dir del PurePursuit per avviarlo :

```bash
  cd path/to/rrt_star/purepursuit
```

```bash
  python ros_pp_local.py
```

```bash
  rostopic pub /commands/stop std_msgs/Bool "data: false"
```



NB : il path del file trj contenente il global planner va modificato all'interno del launch file presente in ../launch/rrt_node.launch
