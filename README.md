# Instrukcja instalacji systemu sterowania manipulatora _Dobot Magician_ na bazie framework'a ROS 2 w laboratorium P109 

## Aktualizacja i instalacja pakietów z repozytorium _apt_: 
```
su administrator
sudo apt update
sudo apt upgrade
sudo apt install ros-humble-diagnostic-aggregator ros-humble-rqt-robot-monitor ros-humble-tf-transformations python3-pykdl python3-pip
exit
```
## Utworzenie przestrzeni roboczej oraz pobranie kodów źródłowych systemu z _GitHub'a_: 
```
source /opt/ros/humble/setup.bash
mkdir -p ~/dobot_anro_system/src
cd dobot_anro_system/
git clone <link_to_system_repo> src/
pip3 install -r src/requirements.txt
rosdep install -i --from-path src --rosdistro humble -y
```
:warning: Jeśli po wykonaniu ostatniej z powyższych komend wyświetli się komunikat `ERROR: your rosdep installation has not been initialized yet.`, wpisz kolejno w konsoli poniższe komendy: 
```
su administrator
sudo rosdep init
rosdep update
exit
```

## Budowanie systemu oraz skasowanie plików źródłowych: 
```
rosdep update
rosdep install -i --from-path src --rosdistro humble -y
colcon build
rm -r build/ log/ 
python3 src/cleaner.py
su administrator
sudo rm -r src/
exit
```
## Przeniesienie systemu sterowania do katalogu _/opt_:
Należy otworzyć dwa okna menedżera plików. W jednym z nich należy się zalogować jako `administrator`. Można to osiągnąć poprzez próbę otwarcia katalogu, do którego użytkownik `student` nie ma dostępu. Przy próbie otwarcia takiego katalogu należy dwukrotnie podać hasło użytkownika `administrator`. Następnie należy przenieść katalog `/dobot_anro_system` do katalogu `/opt` i usunąć katalog `/dobot_anro_system`, który pozostał w katalogu `home` - będąc w tej samej konsoli wpisz w tym celu dwa poniższe polecenia: 
```
cd
rm -r dobot_anro_system/
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
echo "export ROS_LOCALHOST_ONLY=1" >> ~/.bashrc
```
