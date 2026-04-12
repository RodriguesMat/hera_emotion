# hera_emotion

Pacote ROS 2 responsável pela **expressão emocional e renderização facial** do robô social HERA, desenvolvido no âmbito do projeto de Iniciação Científica do time [RoboFEI](https://github.com/RoboFEI-Home).

A interface facial opera como o "atuador visual" da arquitetura: ao receber um comando simbólico publicado em um tópico ROS 2, altera em tempo real as propriedades SVG do rosto (boca, sobrancelhas, olhos, bochechas), produzindo expressões coerentes com o estado interno do robô.

---

## Visão geral

O repositório está organizado em três camadas:

| Camada | Tecnologia | Responsabilidade |
|---|---|---|
| **Backend ROS 2** | Python / rclpy | Nó subscriber que recebe comandos de emoção e os repassa ao front-end |
| **Front-end web** | HTML / JavaScript / SVG | Renderização e animação do rosto em tempo real |
| **Lançamento** | CMake / launch | Configuração e inicialização do pacote no workspace |

A comunicação entre o núcleo de controle (FlexBE / scripts Python) e a interface web utiliza o **Rosbridge Suite** via WebSocket, com **roslibjs** no lado cliente.

---

## Emoções implementadas

O pacote conta com dois protótipos de rosto:

**Versão cartoon** — design minimalista com formas geométricas simples, estratégia deliberada para evitar o *Uncanny Valley*:
- Feliz
- Triste

**Versão humanizada** — gradientes, sombras, pálpebras, cílios e bochechas com rubor animado:
- Alegria
- Surpresa
- Triste
- Pensativa
- Neutra

Cada emoção corresponde a um conjunto de parâmetros visuais (curvatura da boca, inclinação das sobrancelhas, direção do olhar, micro-movimentos) que são aplicados ao SVG quando o tópico `/emotion` recebe a string correspondente.

---

## Pré-requisitos

- ROS 2 (Humble ou superior)
- Python 3
- [`rosbridge_suite`](https://github.com/RobotWebTools/rosbridge_suite)
- Navegador web moderno (para exibição da interface facial)
- Workspace HERA (`hera_ws`) configurado

---

## Instalação

Clone o repositório dentro do `src` do workspace:

```bash
cd ~/hera_ws/src/hera_robot
git clone https://github.com/RodriguesMat/hera_emotion.git
cd ~/hera_ws
colcon build --symlink-install
source install/setup.zsh
```

---

## Execução

**1. Inicie o servidor Rosbridge:**

```bash
ros2 launch rosbridge_server rosbridge_websocket_launch.xml
```

**2. Lance o nó hera_emotion:**

```bash
ros2 launch hera_emotion hera_emotion.launch.py
```

**3. Abra a interface web** no navegador apontando para o arquivo HTML do front-end (ou via servidor HTTP local).

**4. Publique uma emoção** para acionar uma expressão:

```bash
ros2 topic pub /emotion std_msgs/String "data: 'feliz'" --once
```

---

## Tópicos ROS 2

| Tópico | Tipo | Direção | Descrição |
|---|---|---|---|
| `/emotion` | `std_msgs/String` | Subscriber | Recebe o nome da emoção a ser exibida |

---

## Estrutura do repositório

```
hera_emotion/
├── include/hera_emotion/   # Headers C++ (reservado para extensões futuras)
├── launch/                 # Arquivos de lançamento ROS 2
├── src/                    # Nó Python + interface HTML/JS/SVG
├── CMakeLists.txt
└── package.xml
```

---

## Arquitetura do sistema

Este pacote integra a **camada de apresentação facial** de uma arquitetura de três camadas desenvolvida para a HERA:

```
FlexBE (orquestração comportamental)
        │
        │  publica /emotion
        ▼
  hera_emotion (ROS 2 node)
        │
        │  WebSocket (Rosbridge)
        ▼
  Interface HTML/SVG (navegador)
```

O sistema suporta **execução distribuída**: o nó ROS 2 e a interface web podem rodar em máquinas diferentes na mesma rede Wi-Fi, desde que ambas alcancem o servidor Rosbridge.

---

## Contexto do projeto

Este pacote foi desenvolvido como parte de uma Iniciação Científica no **Centro Universitário FEI**, com o objetivo de integrar expressões faciais animadas à plataforma robótica social HERA para pesquisa em interação humano-robô (HRI).

**Time:** RoboFEI  
**Autor:** Matheus Rodrigues  
**Licença:** Apache-2.0
