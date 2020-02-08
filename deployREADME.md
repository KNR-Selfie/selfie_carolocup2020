to change src folder on selfie:
```
./deploy.sh
```
to clean build and devel and change src folder:
```
./deploy.sh -d
```

next, build the workspace on selfie

to run the program on selfie, go to home directory (/home/selfie), and run:
```
startup.sh
```

the program will be ran in a screen session. to connect to it, run:
```
screen -r (selfie_screen)
```
if there are multiple sessions running you can kill them using:
```
killall screen
```
to minimize the session press ctrl+a, and then d, or type:
```
detach
```
to show running screen sessions type:
```
screen -ls
```
