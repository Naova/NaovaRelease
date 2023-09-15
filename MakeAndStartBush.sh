# Reset
Color_Off='\033[0m'       # Text Reset

# Regular Colors
Red='\033[0;31m'          # Red
Green='\033[0;32m'        # Green

# Bold
BRed='\033[1;31m'         # Red
BGreen='\033[1;32m'       # Green


cd Make/Linux
make
if [ $? -eq 0 ]; then
    echo ${BGreen}Make ${Green}successful. Starting SimRobot.${Color_Off}
else
    echo ${Red}Failed to ${BRed}Make${Red}. Exitting.${Color_Off}
    exit
fi
cd ../..
cd Build/Linux/bush/Develop
export DISPLAY=:0
./bush
