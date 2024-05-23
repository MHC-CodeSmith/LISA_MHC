# Estrutura de ROS para Lisa

## Visão Geral
Esta branch contém a implementação da estrutura de comunicação do ROS utilizada no projeto Lisa, focando na interação entre os nós através de publishers, subscribers, services e clients. O código aqui presente estabelece como diferentes componentes do sistema comunicam-se, permitindo um fluxo de dados eficiente e organizado.

## Estrutura de Comunicação
A estrutura de comunicação é baseada em tópicos do ROS, onde diferentes nós publicam e se inscrevem em tópicos específicos para trocar informações. Isso inclui:

- **Publicadores (Publishers):** Nós que enviam dados, como imagens de vídeo ou informações sensoriais.
- **Subscritores (Subscribers):** Nós que recebem dados de outros nós para processamento ou ação subsequente.

Esses componentes são essenciais para o funcionamento autônomo e a interatividade da Lisa, permitindo que o sistema responda de maneira dinâmica ao ambiente e aos comandos do usuário.

## Fluxograma da Estrutura
Incluímos um fluxograma detalhado para ilustrar como os componentes interagem dentro da rede ROS. Para visualizar este fluxograma, navegue até a pasta de imagens neste repositório e abra o arquivo `fluxograma.png`.

## Como Usar

### Passo 1: Copiar o Código

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
mkdir -p src include scripts

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

add_executable(example_node src/example_node.cpp)
target_link_libraries(example_node \${catkin_LIBRARIES})

install(PROGRAMS
    scripts/cam_pub.py
    scripts/cam_sub.py
    scripts/classificar_famoso_rosto_pub-sub.py
    scripts/contador_pub-sub.py
    scripts/gestos_pub-sub.py
    scripts/reconhecer_rostos_pub.py
    scripts/reconhecer_rostos_pub_messages.py
    scripts/reconhecer_rostos_sub_message.py
    scripts/reconhecer_rostos_sub.py
    scripts/seguir_rosto_pub-sub.py
    scripts/serial_node.py
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
