As bibliotecas instaladas nesse tutorial são de vital importância para o funcionamento das câmeras Intel, porém é necessario muito cuidado ao instalar esse arquivos,
as câmeras são produtos como celulares, e são descontinuadas ao longo do tempo, por isso é necessário cuidar para instalar uma versão que respeite as configurações do computador, opere as câmeras desejadas e funcione na versão do ROS instalada.

# Pre-requisitos
Tenha em mãos as câmeras Intel;
ROS instalado;
Verifique a versão do kernel do computador (núcleo do sistema operacional), usando o comando ```uname -r```;

# Realsense SDK
# Install SDK
1. Baixar a release correta, dentro do repositório da Intel no git (https://github.com/IntelRealSense/librealsense/releases) procure a release que correponde a
suas configurações.
![image](https://github.com/julioGSabka/IEEE_OPEN_Tutorials/assets/99331176/459b52ab-eb1e-47fb-a46e-06dcac425c2b)

Nessa foto da release 2.50 fique atento as versões do kernel, as câmeras disponiveis, e a versão suportada do Ubuntu, mais para baixo, baixe o arquivo .zip da release
![image](https://github.com/julioGSabka/IEEE_OPEN_Tutorials/assets/99331176/2e68eafd-a868-4cea-baea-c505324f4129)

* OBS: Caso não encontre uma versão com o kernel compativel, é possivel alterar a versão do kernel utilizada para bootar o computador.
* Se o seu dispositivo for uma placa Nvidia Jetson, siga o tutorial para Jetson Nano (-link-)
2. Instalação do SDK, para um tutorial mais detalhado veja: https://github.com/IntelRealSense/librealsense/blob/master/doc/installation.md

Atualize a distribuição do Ubuntu, incluindo a obtenção do kernel estável mais recente:
```
sudo apt-get update && sudo apt-get upgrade && sudo apt-get dist-upgrade
```
Extraia o arquivo a release baixada, em uma pasta de preferência (recomendo a pasta home)

Navegue até o diretório raiz librealsense para executar os scripts a seguir.
Desconecte qualquer câmera Intel RealSense conectada.

Instale os pacotes principais necessários para construir binários librealsense e os módulos do kernel afetados:

```sudo apt-get install libssl-dev libusb-1.0-0-dev libudev-dev pkg-config libgtk-3-dev cmake```

Pacotes específicos de distribuição:

  * Ubuntu 14 ou ao executar o Ubuntu 16.04 live-disk:
    ```./scripts/install_glfw3.sh```

  * Ubuntu 16:
    ```sudo apt-get install libglfw3-dev```

  * Ubuntu 18/20/22:
    ```sudo apt-get install libglfw3-dev libgl1-mesa-dev libglu1-mesa-dev at```
    
Execute o script de permissões Intel Realsense no diretório raiz librealsense:
```./scripts/setup_udev_rules.sh```

Crie e aplique módulos de kernel para:

 * Ubuntu 20/22 (focal/jammy) com LTS kernel 5.13, 5.15
    ```./scripts/patch-realsense-ubuntu-lts-hwe.sh```

 * Ubuntu 14/16/18/20 com LTS kernel (< 5.13)
    ```./scripts/patch-realsense-ubuntu-lts.sh```
# Build SDK
* Navegue até o diretório raiz do librealsense e execute
```mkdir build && cd build```
* Execute o CMake:
```cmake ../ -DBUILD_EXAMPLES=true``` - Constrói librealsense junto com demos e tutoriais
* Recompilar e instalar binários librealsense:

```sudo make uninstall && make clean && make && sudo make install```
# Lib realsense-ros
Após ter instalado o SDK, é necessário baixar a biblioteca realsense-ros
1. Baixe a release correspondente a sua situação, analisando, o ROS suportado, a versão do SDK, e os dispositivos suportados.
Página de downloads(https://github.com/IntelRealSense/realsense-ros/releases)
2. Para um tutorial mais detalhado siga: https://github.com/IntelRealSense/realsense-ros/tree/ros1-legacy
*Lembrando que esse tutorial é para ROS, para ROS2 siga: https://github.com/IntelRealSense/realsense-ros/tree/ros2-development
3. Crie um espaço de trabalho catkin Ubuntu, não faça isso no seu workspace
```
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src/
```
Especificamente, certifique-se de que o pacote ros ddynamic_reconfigure esteja instalado(Senão instale com: ```sudo apt-get install ros-$ROS-DISTRO-ddynamic-reconfigure```)
```
catkin_init_workspace
cd ..
catkin_make clean
catkin_make -DCATKIN_ENABLE_TESTING=False -DCMAKE_BUILD_TYPE=Release
catkin_make install
echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
source ~/.bashrc
```
