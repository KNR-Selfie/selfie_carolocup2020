to change src folder on selfie:
```
./deploy.sh
```
to also clean build and devel: 
```
./deploy.sh -d
```

next, build the workspace on selfie
```
catkin_make
```

to run the program on selfie, go to home directory (/home/selfie), and run:
```
./startup.sh
```

the program will be ran in a screen session. to connect to it, run:
```
screen -r 
```
if there are multiple sessions running you can kill them using:
```
killall screen
```
to minimize the session press ctrl+a, and then d, or type:
```
screen -d
```
to show running screen sessions type:
```
screen -ls
```
