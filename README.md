# Instrukcja instalacji systemu sterowania manipulatora _Dobot Magician_ na bazie framework'a ROS 2 w laboratorium P109 

## Instalacja ROS Jazzy

Na początku należy zainstalować odpowiednią dystrybucję frameworka ROS, zgodnie z instrukcją:
[https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html)

## Aktualizacja i instalacja pakietów z repozytorium _apt_: 
```
su administrator
sudo apt update
sudo apt upgrade
sudo apt install ros-jazzy-diagnostic-aggregator ros-jazzy-rqt-robot-monitor ros-jazzy-tf-transformations ros-jazzy-urdf-tutorial ros-jazzy-python-qt-binding python3-pykdl python3-pip python3-colcon-common-extensions
exit
```
## Utworzenie przestrzeni roboczej oraz pobranie kodów źródłowych systemu z _GitHub'a_: 
```
source /opt/ros/jazzy/setup.bash
mkdir -p ~/dobot_anro_system/src
cd dobot_anro_system/
git clone -b fszyszko2 https://github.com/GroupOfRobots/magician_ros2.git src/
su administrator
sudo pip3 install --break-system-packages -r src/requirements.txt
exit
rosdep install -i --from-path src --rosdistro jazzy -y
```
:warning: Jeśli po wykonaniu ostatniej z powyższych komend wyświetli się komunikat `ERROR: your rosdep installation has not been initialized yet.`, wpisz kolejno w konsoli poniższe komendy: 
```
su administrator
sudo rosdep init
rosdep update
exit
```

## Budowanie systemu oraz skasowanie plików źródłowych oraz przeniesienie systemu sterowania do katalogu _/opt_: 
```
rosdep update
rosdep install -i --from-path src --rosdistro jazzy -y
colcon build
su administrator
sudo rm -rf build/ log/ src/
cd .. && sudo mv dobot_anro_system/ /opt/dobot_anro_system/
exit
```

## Dodanie użytkownika do grupy _dialout_: 
```
su administrator
sudo adduser student dialout 
exit
```
## :exclamation: Zrestartuj komputer :exclamation:
:warning: Restart jest konieczny, aby odświeżyć uprawnienia użytkownika tzn. możliwość korzystania z portów szeregowych. 

## Odświeżenie pamięci _cache_ RQT: 
```
source /opt/dobot_anro_system/install/setup.bash
rqt --force-discover
```
## Utworzenie skryptu konfigurującego środowisko do pracy z robotem:
```
su administrator
cd /opt/
sudo mkdir p109
cd p109/
sudo touch anro.sh
sudo nano anro.sh
```
Skopiuj do pliku _anro.sh_ poniższy skrypt _bash'owy_, zapisz zmiany i zamknij plik, a następnie wyjdź z powłoki administratora komendą `exit`.
```
#!/bin/bash
echo "source /opt/dobot_anro_system/install/setup.bash" >> ~/.bashrc
echo "export ROS_AUTOMATIC_RANGE_DISCOVERY=LOCALHOST" >> ~/.bashrc
```
