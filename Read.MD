# Estrutura de ROS para Lisa

## Visão Geral
Esta branch contém a implementação da estrutura de comunicação do ROS utilizada no projeto Lisa, focando na interação entre os nós através de publishers, subscribers, services e clients. O código aqui presente estabelece como diferentes componentes do sistema comunicam-se, permitindo um fluxo de dados eficiente e organizado. 

## Estrutura de Comunicação
A estrutura de comunicação é baseada em tópicos e nós do ROS, onde diferentes nós publicam e se inscrevem em tópicos específicos para trocar informações, além da utilização de serviços e clientes, onde o cliente principal controla e aciona os serviços. Isso inclui:

- **Publicadores (Publishers):** Nós que enviam dados, como imagens de vídeo ou informações sensoriais.
- **Subscritores (Subscribers):** Nós que recebem dados de outros nós para processamento ou ação subsequente.
- **Servidores (Servers):** Nós que oferecem serviços que podem ser acionados por outros nós.
- **Clientes (Clients):** Nós que solicitam a execução de serviços a partir dos servidores.

Esses componentes são essenciais para o funcionamento autônomo e a interatividade da Lisa, permitindo que o sistema responda de maneira dinâmica ao ambiente e aos comandos do usuário.

### Descrição dos Componentes
- **cam_pub**: Publica imagens capturadas pela câmera no tópico `imagens`.
- **controlador_client**: Cliente principal que aciona diferentes serviços com base nas imagens recebidas e publica resultados no tópico `resultados`.
- **gesto_service**: Serviço que reconhece gestos a partir das imagens.
- **famosos_service**: Serviço que reconhece rostos de pessoas famosas a partir das imagens.
- **seguir_service**: Serviço que detecta e segue rostos nas imagens.
- **contador_service**: Serviço que conta o número de dedos mostrados nas imagens.
- **rostos_service**: Serviço que reconhece rostos genéricos nas imagens.
- **Serial**: Componente que envia resultados processados para o dispositivo ESP32 via comunicação ROSserial.

## Fluxograma da Estrutura
Incluímos um fluxograma detalhado para ilustrar como os componentes interagem dentro da rede ROS. Para visualizar este fluxograma, veja abaixo:

![Fluxograma do Projeto](Images/fluxograma.png)

## Como Usar

### Passo 1: Copiar o Código (Ctrl-C)

```bash
#!/bin/bash

# Função para verificar se um comando existe
command_exists() {
    command -v "$1" >/dev/null 2>&1
}

# Instalação de dependências do sistema
echo "Instalando dependências do sistema..."
sudo apt-get update
sudo apt-get install -y build-essential cmake git libopencv-dev python3-pip

# Instalar ROS Noetic
if ! command_exists roscore; then
    echo "Instalando ROS Noetic..."
    sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
    sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
    sudo apt-get update
    sudo apt-get install -y ros-noetic-desktop-full
    echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
    source /opt/ros/noetic/setup.bash
fi

# Inicializar rosdep
if ! command_exists rosdep; then
    echo "Inicializando rosdep..."
    sudo rosdep init
    rosdep update
fi

# Instalar dependências Python
echo "Instalando dependências Python..."
pip3 install -U rosdep rosinstall_generator wstool rosinstall six vcstools
pip3 install mediapipe pyserial

# Configuração inicial do workspace
echo "Criando o workspace e o pacote..."
mkdir -p ~/lisa_ws/src
cd ~/lisa_ws/
catkin_make

source devel/setup.bash
echo "source ~/lisa_ws/devel/setup.bash" >> ~/.bashrc
source ~/.bashrc

cd ~/lisa_ws/src

catkin_create_pkg estrutura std_msgs rospy roscpp
cd ~/lisa_ws

cd ~/lisa_ws/src/estrutura

cd ~/lisa_ws/src/estrutura/scripts
git clone --depth 1 https://github.com/MHC-CodeSmith/LISA_MHC.git .
rm -rf .git
chmod +x *.py

# Atualizar CMakeLists.txt
echo "Atualizando CMakeLists.txt..."
cat <<EOL > ~/lisa_ws/src/estrutura/CMakeLists.txt
cmake_minimum_required(VERSION 3.0.2)
project(estrutura)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
)

catkin_package()

include_directories(
  \${catkin_INCLUDE_DIRS}
)

install(PROGRAMS
    scripts/cam_pub.py
    scripts/cam_sub.py
    scripts/contador_service.py
    scripts/controlador.py
    scripts/display_node.py
    scripts/gesto_service.py
    scripts/reconhecimento_rosto_service.py
    scripts/serial_node.py
    scripts/show_faces.py
    DESTINATION \${CATKIN_PACKAGE_BIN_DESTINATION}
)
EOL

# Compilar o workspace
echo "Compilando o workspace..."
cd ~/lisa_ws
catkin_make

source devel/setup.bash
echo "Configuração concluída com sucesso."
```

### Passo 2: Salvar o Script: Salve o script acima em um arquivo chamado setup_workspace.sh. (crtl-V dentro do nano e depois crtl-O Enter e ctrl-X)
```bash
touch setup_workspace.sh && nano setup_workspace.sh
```

### Passo 3: Tornar o Script Executável: Torne o script executável.
```bash
chmod +x setup_workspace.sh
```

### Passo 4: Executar o Script: Execute o script.
```bash
./setup_workspace.sh
```
