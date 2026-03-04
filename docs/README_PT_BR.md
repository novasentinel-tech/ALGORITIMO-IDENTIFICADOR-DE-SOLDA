# Sistema Avançado de Identificação e Planejamento de Soldagem

Um sistema completo de visão computacional e planejamento de movimento para identificação automática de juntas de solda e planejamento de processo de soldagem com ROS e Gazebo.

## Características Principais

✨ **Detecção Inteligente de Juntas**
- Processamento de imagens com OpenCV
- Detecção automática de tipos de junta (aberta/fechada)
- Cálculo de profundidade e dimensões
- Estimativa de confiança da detecção

🔧 **Classificação de Processo**
- Classificação automática de tipo de passe (raiz, enchimento, acabamento)
- Seleção inteligente de processo de soldagem:
  - **SMAW**: Eletrodo revestido (campo)
  - **GTAW**: TIG (alta precisão)
  - **GMAW**: MIG/MAG (produção)
  - **FCAW**: Arame tubular
- Cálculo de parâmetros ótimos (tensão, corrente, velocidade)

🤖 **Integração com ROS**
- Arquitetura modular com nós ROS
- Suporte a MoveIt para planejamento de trajetória
- Publicação de planos em tópicos ROS
- Services para controle remoto

📊 **Análise Completa**
- Geometria 3D da junta
- Ângulo da tocha
- Número de passes necessários
- Parâmetros de soldagem por processo

## Estrutura do Projeto

```
welding_system/
├── src/
│   ├── joint_detector.py          # Módulo principal de detecção
│   └── __init__.py
├── scripts/
│   ├── image_publisher_node.py    # Nó de publicação de imagens
│   ├── welding_planner_node.py    # Nó de planejamento (principal)
│   └── movement_executor_node.py  # Nó de execução com MoveIt
├── launch/
│   ├── welding_system.launch      # Launch file principal
│   └── gazebo_welding.launch      # Launch file com Gazebo
├── config/
│   └── welding_params.yaml        # Parâmetros de configuração
├── test/
│   └── test_joint_detector.py     # Testes unitários
├── CMakeLists.txt
└── package.xml
```

## Dependências

### Dependências ROS
```bash
ros-<distro>-rospy
ros-<distro>-cv-bridge
ros-<distro>-sensor-msgs
ros-<distro>-geometry-msgs
ros-<distro>-moveit
ros-<distro>-gazebo-ros
```

### Dependências Python
```bash
opencv-python
numpy
opencv-contrib-python
```

## Instalação

1. **Clone o repositório**
```bash
cd ~/catkin_ws/src
git clone https://github.com/novasentinel-tech/ALGORITIMO-IDENTIFICADOR-DE-SOLDA.git
cd ..
```

2. **Instale as dependências**
```bash
rosdep install --from-paths src --ignore-src -r -y
pip install opencv-python numpy
```

3. **Compile o workspace**
```bash
catkin build
source devel/setup.bash
```

## Uso

### Modo Simulado (Recomendado para Testes)

```bash
# Terminal 1: Inicia todos os nós
roslaunch welding_system welding_system.launch

# Terminal 2: Monitora os tópicos
rostopic echo /welding/pass_type
rostopic echo /welding/process
rostopic echo /welding/confidence

# Terminal 3: Para resetar contador
rosservice call /welding/reset_counter

# Terminal 4: Para ver status
rosservice call /welding/get_status
```

### Com Gazebo

```bash
roslaunch welding_system gazebo_welding.launch
```

### Com Câmera Real

Edite o launch file para mudar a fonte de imagem:
```xml
<param name="image_source" value="camera"/>
<param name="camera_id" value="0"/>
```

### Com Arquivo de Imagem

```xml
<param name="image_source" value="file"/>
<param name="image_file" value="/path/to/weld_joint.jpg"/>
```

## Tópicos ROS

### Publishers

| Tópico | Tipo | Descrição |
|--------|------|-----------|
| `/camera/image_raw` | `sensor_msgs/Image` | Imagem da câmera |
| `/welding/plan` | `std_msgs/String` | Plano completo de soldagem |
| `/welding/pass_type` | `std_msgs/String` | Tipo de passe (root/fill/cap) |
| `/welding/process` | `std_msgs/String` | Processo (SMAW/GTAW/GMAW/FCAW) |
| `/welding/confidence` | `std_msgs/Float32` | Confiança da detecção (0-1) |
| `/welding/num_passes` | `std_msgs/Int32` | Número de passes necessários |
| `/welding/visualized_image` | `sensor_msgs/Image` | Imagem com anotações |
| `/welding/planned_trajectory` | `trajectory_msgs/JointTrajectory` | Trajetória planejada |
| `/welding/movement_status` | `std_msgs/String` | Status do movimento |

### Services

| Service | Tipo | Descrição |
|---------|------|-----------|
| `/welding/reset_counter` | `std_srvs/Trigger` | Reseta contador de juntas |
| `/welding/get_status` | `std_srvs/Trigger` | Obtém status do nó |
| `/welding/execute_plan` | `std_srvs/Trigger` | Executa plano no robô |
| `/welding/get_robot_state` | `std_srvs/Trigger` | Obtém estado do robô |
| `/welding/add_obstacles` | `std_srvs/Trigger` | Adiciona obstáculos à cena |

## Exemplos de Uso Programático

### Python - Usar o detector diretamente

```python
from joint_detector import WeldingJointDetector, WeldingClassifier
import cv2

# Inicializa detector
detector = WeldingJointDetector()
classifier = WeldingClassifier()

# Carrega imagem
image = cv2.imread("junta_solda.jpg")

# Detecta junta
joint_info = detector.detect_joint(image)

if joint_info:
    print(f"Junta detectada!")
    print(f"  Área: {joint_info.area} pixels²")
    print(f"  Profundidade: {joint_info.depth} pixels")
    print(f"  Confiança: {joint_info.confidence:.2%}")
    
    # Planeja soldagem
    plan = classifier.plan_welding(joint_info)
    print(f"\nPlano de Soldagem:")
    print(f"  Tipo de Passe: {plan.pass_type.value}")
    print(f"  Processo: {plan.process.value}")
    print(f"  Passes Necessários: {plan.num_passes}")
    print(f"  Tensão: {plan.parameters['voltage']} V")
    print(f"  Corrente: {plan.parameters['current']} A")
```

### ROS - Usar via Tópicos

```python
#!/usr/bin/env python3
import rospy
from std_msgs.msg import String

def callback(msg):
    rospy.loginfo(f"Plano recebido:\n{msg.data}")

rospy.init_node('welding_monitor')
rospy.Subscriber('/welding/plan', String, callback)
rospy.spin()
```

## Fluxo de Processamento

```
┌─────────────────────┐
│  Câmera/Dataset     │
└──────────┬──────────┘
           │
           ▼
┌──────────────────────────────────┐
│  Imagem ROS Topic                │
│  (/camera/image_raw)             │
└──────────┬───────────────────────┘
           │
           ▼
┌──────────────────────────────────┐
│  Detecção de Junta               │
│  - OpenCV Edge Detection         │
│  - Contour Analysis              │
│  - Feature Extraction            │
└──────────┬───────────────────────┘
           │
           ▼
┌──────────────────────────────────┐
│  Classificação de Passe          │
│  - Root / Fill / Cap             │
│  - Junta Aberta/Fechada          │
│  - Profundidade Estimada         │
└──────────┬───────────────────────┘
           │
           ▼
┌──────────────────────────────────┐
│  Seleção de Processo             │
│  - SMAW / GTAW / GMAW / FCAW     │
│  - Parâmetros Ótimos             │
└──────────┬───────────────────────┘
           │
           ▼
┌──────────────────────────────────┐
│  Planejamento de Trajetória      │
│  - MoveIt Motion Planning        │
│  - Cálculo de Torcha             │
│  - Pontos de Waypoint            │
└──────────┬───────────────────────┘
           │
           ▼
┌──────────────────────────────────┐
│  Execução no Robô                │
│  - Gazebo Simulation             │
│  - Hardware Real (opcional)      │
└──────────────────────────────────┘
```

## Parâmetros de Configuração

Veja `config/welding_params.yaml` para configuração completa. Parâmetros principais:

```yaml
# Detecção
joint_detection:
  min_confidence: 0.3
  canny_threshold1: 50
  canny_threshold2: 150

# Materiais
materials:
  steel: { ... }
  stainless_steel: { ... }
  aluminum: { ... }

# Processos
welding_processes:
  SMAW: { voltage: [18-22], current: [70-150], ... }
  GTAW: { voltage: [12-18], current: [80-200], ... }
  GMAW: { voltage: [18-28], current: [150-300], ... }
  FCAW: { voltage: [22-28], current: [180-350], ... }
```

## Testes

Execute os testes unitários:

```bash
cd src/welding_system
python3 -m pytest test/test_joint_detector.py -v

# Ou com unittest
python3 test/test_joint_detector.py
```

## Algoritmo de Detecção

1. **Pré-processamento**
   - Conversão para escala de cinza
   - Desfoque Gaussiano (5x5)
   - Redução de ruído

2. **Detecção de Bordas**
   - Canny Edge Detection (thresholds: 50, 150)
   - Encontra contornos

3. **Análise de Features**
   - Calcula área, perímetro, centroide
   - Detecta forma (aberta/fechada)
   - Estima profundidade

4. **Confiança**
   - Baseada em circularidade do contorno
   - Intervalo: 0 a 1

## Algoritmo de Classificação

1. **Ordem de Passes**: sempre ROOT → FILL → CAP
2. **Número de Passes**: baseado em profundidade
   - até 5mm: 1 passe
   - 5-15mm: 2 passes
   - acima de 15mm: múltiplos (depth/8)

3. **Seleção de Processo**:
   - ROOT: GTAW (precisão) ou SMAW (campo)
   - FILL: GMAW ou FCAW (produçao)
   - CAP: GMAW (acabamento)

4. **Parâmetros**: procurados na biblioteca de conhecimento

## Troubleshooting

### Nenhuma junta detectada
- Verificar iluminação da câmera
- Ajustar thresholds de Canny em `config/welding_params.yaml`
- Verifique se a junta é visível na imagem

### Confiança muito baixa
- A junta pode ter forma irregular
- Tente melhorar contraste da imagem
- Ajuste `min_confidence` nos parâmetros

### Processo incorreto selecionado
- Verifique se material está correto no launch file
- Ajuste as regras de seleção em `WeldingClassifier.select_process()`

## Contribuindo

Para submeter melhorias:
1. Fork o repositório
2. Crie uma branch (`git checkout -b feature/melhoria`)
3. Commit suas mudanças (`git commit -am 'Add melhoria'`)
4. Push para a branch (`git push origin feature/melhoria`)
5. Abra um Pull Request

## Créditos

Desenvolvido por **Nova Sentinel Tech**

Se você usar ou modificar este código, **por favor mencione este repositório**.

## Licença

Este projeto é licenciado sob a Licença GPL-3.0 - veja [LICENSE](LICENSE) para detalhes.

## Referências Técnicas

- [OpenCV Documentation](https://docs.opencv.org/)
- [ROS Documentation](http://wiki.ros.org/)
- [MoveIt! Documentation](https://moveit.ros.org/)
- [Gazebo Simulation](http://gazebosim.org/)

## Roadmap

- [ ] Suporte a YOLO/YOLOv8 para detecção neural
- [ ] Integração com Fast-SAM
- [ ] Dashboard web para monitoramento
- [ ] Suporte a múltiplas câmeras
- [ ] Calibração automática de câmera
- [ ] Banco de dados de padrões de junta
- [ ] Geração automática de URDF
- [ ] Simulação em tempo real com Gazebo

## Suporte

Para questões, abra uma [Issue](https://github.com/novasentinel-tech/ALGORITIMO-IDENTIFICADOR-DE-SOLDA/issues)

---

**Desenvolvido com ❤️ para soldagem inteligente**
