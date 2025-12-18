# Novo Rosto com Variações de Emoções e Estados para o Robô HERA 

O pacote hera_emotion é um pacote ROS 2 responsável por renderizar e controlar uma interface facial animada (baseada em SVG) para o robô HERA. 

O rosto é exibido num navegador web e comunica com o sistema ROS2, permitindo alterações dinâmicas de expressão.

----

## Funcionalidades

- **Interface Web:** Renderiza um rosto vetorizado (SVG) acessível via navegador.
- **Expressões Faciais:** Suporta diversas emoções (ex: `alegria`, `surpresa`, `nojo`, `angústia`, `pensativa` e `neutra`) e estados durante uma task (ex: `procurando`, `navegando`, `obstáculo` e `encontrado`).
- **Animação "Idle":** O rosto possui movimentos autónomos (piscar, oscilação da cabeça) para parecer vivo quando ocioso.
- **Lip-sync:** Sincronização da abertura da boca baseada na intensidade do som (via tópico ROS).
- **Serviço ROS:** Disponibiliza um serviço para alterar a emoção de forma programática.

----

## Dependências

Para utilizar esse pacote, é necessário ter instalado:

- **ROS2 Humble.**
- **rosbridge_suite:** Necessário para a comunicação entre o navegador e o ROS.

```bash
sudo apt install ros-humble-rosbridge-server
```
- **hera_msgs:** Um pacote especializado que deve conter a definição do serviço `setEmotion`.

```python
string emotion
---
bool success
string message
```
- **speak.py:** Um arquivo específico para cuidar do TTS (Text To Speech) do robô, com uma personalização que mande as palavras para que tenha a movimentação da boca.

----

## Como Executar

O pacote possui um arquivo de launch que inicia todos os nós necessários (o servidor bridge, o servidor HTTP para a página e o nó de lógica).

```bash
ros2 launch hera_emotion emotion.launch.py
```
Isto irá iniciar:
- **rosbridge_websocket** na porta `9090`.
- **Servidor HTTP Python** na porta `8000` (servindo a pasta `include/hera_emotion`).
- **Nó** `emotion_node` (`setEmotion.py`).

----

## Abrir Interface

Abra o navegador, na própria máquina em que está rodando ou qualquer outro aparelho conectado a mesma rede wifi, e no campo de pesquisa digite `http://192.168.0.227:9090`.

**Nota Importante sobre o IP:** 
No arquivo `include/hera_emotion/face.html` tem o IP do websocket definido estaticamente na linha 271: `const ROSBRIDGE_URL = 'ws://192.168.0.227:9090';`.  
Quando for rodar esse arquivo, tem que trocar o endereço de IP `192.168.0.227` para o correspondente da sua máquina.

----

# Tópicos e Serviços

**Serviços**
- **/setEmotion** (`hera_msgs/srv/SetEmotion`)
  - Altera a expressão facial do robô.
  - Exemplo de chamada via terminal:
    ```bash
    ros2 service call setEmotion hera_msgs/srv/SetEmotion "{emotion: 'navegando'}"  
    ```
**Tópicos Subscritos**
- **/hera/emotion** (`std_msgs/String`)
   - Recebe a string da emoção a ser exibida. Publicado pelo nó `setEmotion.py`.
- **/hera/mouth_open** (`std_msgs/Float32`)
   - Recebe um valor entre `0.0` e `1.0` para controlar a abertura da boca (lip-sync).
 
----

> Nota: Se for utilizar este rosto sem o ROS 2, para conseguir mudar as emoções manualmente, é necessário acessar o arquivo  `include/hera_emotion/face.html` e descomentar as linhas 179 a 191. Isso fará com que os botões de controle apareçam na tela.
