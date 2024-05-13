# BurguerTeleop

Este repositório contém um projeto em ROS 2 que move o robô `Turtlebot3 Burger` através de teleoperação utilizando o teclado.

## Requisitos

Para executar este projeto, é necessário ter o **ROS 2** e o **Turtlebot3** instalados além de possuir linux como sistema operacional (também é possível utiliar o WSL). Caso a execução do código seja feita através de um simulador como _**Webots**_ ou _**Gazebo**_, é necessário realizar sua instalação no terminal. <br>

Por fim, é necessário clonar o repositório [ros2_tutorials](https://github.com/ros/ros_tutorials.git) no caminho `turtle_workspace/src`. Para isso, execute o comando abaixo:

```bash
git clone https://github.com/ros/ros_tutorials.git -b humble 
```

## Como executar

Para executar o projeto, siga os passos abaixo:

1. Clone este repositório;
2. Abra um terminal e execute o comando `ros2 launch webots_ros2_turtlebot robot_launch.py`, para iniciar o simulador do robô através do Webots;
3. Abra outro terminal e navegue até a pasta _turtle_workspace_ utilizando o comando `cd <caminho>/turtle_workspace`;
4. Execute o comando `colcon build`, para compilar o projeto;
5. Execute o comando `source install/setup.bash`, para carregar as variáveis de ambiente;
6. Execute o comando `ros2 run burguer_controller release_node`, para executar o projeto.

Caso sua execução seja com o robô real, substitua o comando `ros2 launch webots_ros2_turtlebot robot_launch.py` pelo comando `ros2 launch turtlebot3_bringup robot_launch.py`.

## Vídeo de demonstração

Clique na imagem abaixo para assistir ao vídeo de demonstração do projeto:

[![Vídeo de demonstração](https://arminlab.com/wp-content/uploads/2022/09/icons8-youtube-play-button-2048-300x300.png)]()