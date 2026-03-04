# Arquitetura do Sistema de Soldagem Inteligente

## 📐 Visão Geral da Arquitetura

```
┌─────────────────────────────────────────────────────────────────┐
│                     SISTEMA DE SOLDAGEM INTELIGENTE              │
└─────────────────────────────────────────────────────────────────┘

                          CAMADA DE APRESENTAÇÃO
        ┌────────────────────────────────────────────────────┐
        │  RViz Visualization │ RQT Dashboard │ ROS Topics    │
        └────────────────────────────────────────────────────┘
                                    ▲
                                    │
        ┌───────────────────────────┴────────────────────────┐
        │                MIDDLEWARE ROS 2 / ROS 1            │
        │ (Message Passing, Service Call, Parameter Server)  │
        └───────────────────────────┬────────────────────────┘
                                    │
        ┌───────────────────────────┴────────────────────────┐
        │            CAMADA DE PROCESSAMENTO INTELIGENTE     │
        │  ┌──────────────────────────────────────────────┐  │
        │  │  Welding Planner Node (Principal)             │  │
        │  │  - Orquestra todo o sistema                   │  │
        │  │  - Integra detecção e classificação           │  │
        │  │  - Publica planos de soldagem                 │  │
        │  └──────────────────────────────────────────────┘  │
        │           │                          │               │
        │           ▼                          ▼               │
        │  ┌─────────────────────┐  ┌──────────────────────┐  │
        │  │ Joint Detector      │  │ Welding Classifier   │  │
        │  │                     │  │                      │  │
        │  │ - Canny Edge Det.   │  │ - Classify Pass Type │  │
        │  │ - Contour Analysis  │  │ - Select Process     │  │
        │  │ - Feature Extract   │  │ - Calculate Params   │  │
        │  │ - Confidence Check  │  │ - Plan Trajectory    │  │
        │  └─────────────────────┘  └──────────────────────┘  │
        │                                                       │
        └───────────────────────────┬────────────────────────┘
                                    │
        ┌───────────────────────────┴────────────────────────┐
        │            CAMADA DE AQUISIÇÃO E EXECUÇÃO          │
        │  ┌──────────────────────────────────────────────┐  │
        │  │  Image Publisher (Câmera/Dataset)            │  │
        │  │  - Captura de câmera em tempo real           │  │
        │  │  - Suporte a dataset estático                │  │
        │  │  - Publicação em tópico ROS                  │  │
        │  └──────────────────────────────────────────────┘  │
        │                        │                             │
        │  ┌─────────────────────────────────────────────┐    │
        │  │  Movement Executor (MoveIt Integration)      │    │
        │  │  - Planejamento de trajetória (MoveIt)       │    │
        │  │  - Execução em Gazebo/Hardware Real          │    │
        │  │  - Colisão Avoidance                         │    │
        │  └─────────────────────────────────────────────┘    │
        │                                                       │
        └───────────────────────────┬────────────────────────┘
                                    │
        ┌───────────────────────────┴────────────────────────┐
        │         INTERFACES E PERIFÉRICOS (Opcional)        │
        │  - Hardware: Câmera Gazebo/Real, Robô Manipulador  │
        │  - Database: Parâmetros de Soldagem                │
        │  - Simulação: Gazebo World + Models                │
        └─────────────────────────────────────────────────────┘
```

## 🔄 Fluxo de Dados

```
┌─────────────────┐
│ ENTRADA: Imagem │
└────────┬────────┘
         │
         ▼
    ┌─────────────────────────────────────────┐
    │    ETAPA 1: AQUISIÇÃO                   │
    │    ImagePublisherNode                   │
    │                                         │
    │  - Lê câmera ou imagem estática      │
    │  - Publica em /camera/image_raw      │
    └────────────┬────────────────────────────┘
                 │ (ROS Topic)
                 │
                 ▼
    ┌─────────────────────────────────────────┐
    │    ETAPA 2: PRÉ-PROCESSAMENTO            │
    │    (em WeldingPlannerNode)              │
    │                                         │
    │  - Converte ROS Image → OpenCV       │
    │  - Cinza + Desfoque                  │
    │  - Aplicar Canny Edge Detection      │
    │  - Encontrar Contornos                │
    └────────────┬────────────────────────────┘
                 │
                 ▼
    ┌─────────────────────────────────────────┐
    │    ETAPA 3: DETECÇÃO DE JUNTA            │
    │    WeldingJointDetector                │
    │                                         │
    │  - Filtra contornos válidos          │
    │  - Extrai features:                   │
    │    • Área, Perímetro                │
    │    • Centro, Dimensões               │
    │    • Tipo (aberta/fechada)           │
    │    • Profundidade estimada           │
    │  - Calcula confiança                 │
    │                                         │
    │  Saída: JointInfo object              │
    └────────────┬────────────────────────────┘
                 │
                 ▼
    ┌─────────────────────────────────────────┐
    │    ETAPA 4: CLASSIFICAÇÃO                │
    │    WeldingClassifier                   │
    │                                         │
    │  4.1 - Classificar tipo de passe:    │
    │        Root (1º) → Fill (2º) → Cap   │
    │                                         │
    │  4.2 - Selecionar processo:             │
    │        SMAW/GTAW/GMAW/FCAW            │
    │        (baseado em passe + junta)     │
    │                                         │
    │  4.3 - Obter parâmetros:                │
    │        Tensão, Corrente, Velocidade  │
    │        (de lookup table)              │
    │                                         │
    │  4.4 - Calcular geometria:              │
    │        Ângulo da tocha, pontos,       │
    │        velocidade de avanço           │
    │                                         │
    │  Saída: WeldingPlan object             │
    └────────────┬────────────────────────────┘
                 │
                 ▼
    ┌─────────────────────────────────────────┐
    │    ETAPA 5: PUBLICAÇÃO DE PLANO          │
    │    (WeldingPlannerNode publishers)     │
    │                                         │
    │  Publica em tópicos:                   │
    │  - /welding/plan (completo)          │
    │  - /welding/pass_type                 │
    │  - /welding/process                   │
    │  - /welding/confidence                │
    │  - /welding/num_passes                │
    │  - /welding/visualized_image          │
    └────────────┬────────────────────────────┘
                 │
                 ▼
    ┌─────────────────────────────────────────┐
    │    ETAPA 6: PLANEJAMENTO DE MOVIMENTO   │
    │    MovementExecutorNode                │
    │                                         │
    │  6.1 - Parse do plano                  │
    │  6.2 - Planejamento com MoveIt         │
    │        (ou simulação se não disponível)│
    │  6.3 - Publica trajetória              │
    │        em /welding/planned_trajectory │
    └────────────┬────────────────────────────┘
                 │
                 ▼
    ┌─────────────────────────────────────────┐
    │    ETAPA 7: EXECUÇÃO (Opcional)         │
    │    via Service /welding/execute_plan   │
    │                                         │
    │  - Executa em Gazebo simulado         │
    │  - ou encaminha para HW real           │
    │  - Feedback em movimento_status       │
    └────────────┬────────────────────────────┘
                 │
                 ▼
            [ SOLDA EXECUTADA ]
```

## 🏗️ Componentes Principais

### 1. **WeldingJointDetector**

**Responsabilidade**: Detectar automaticamente juntas de solda em imagens

```
Entrada: np.ndarray (imagem OpenCV BGR)
         ↓
Processamento:
  • cvtColor → BGR a Gray
  • GaussianBlur(5x5)
  • Canny(50, 150)
  • findContours
  
Filtragem:
  • min_area < contour < max_area
  • Seleciona maior válido
  
Extração de Features:
  • area = cv2.contourArea
  • perimeter = cv2.arcLength
  • moments (M10, M01, M00) → centroidé
  • minAreaRect → dimensões
  • Razão de aspecto → aberta/fechada
  • Confiança = circularidade
         ↓
Saída: JointInfo | None
       ├─ contour
       ├─ area, perimeter
       ├─ center (cx, cy)
       ├─ width, depth, angle
       ├─ is_open (bool)
       └─ confidence (0-1)
```

**Classes Enumeradas**:
- `PassType`: ROOT, FILL, CAP
- `WeldProcess`: SMAW, GTAW, GMAW, FCAW

### 2. **WeldingClassifier**

**Responsabilidade**: Classificar tipo de passe e selecionar processo apropriado

```
Métodos:
├─ classify_pass(joint_info, pass_number)
│  └─ Retorna PassType baseado no número do passe
│
├─ select_process(pass_type, joint_info, material)
│  └─ Seleciona WeldProcess ideal
│     Lógica:
│     • ROOT: GTAW (precisão) ou SMAW (campo)
│     • FILL: GMAW ou FCAW (profundidade)
│     • CAP: GMAW (acabamento)
│
├─ plan_welding(joint_info, pass_number, material)
│  └─ Cria WeldingPlan completo
│
└─ Lookup Tables:
   └─ welding_parameters[(pass_type, process)]
      ├─ voltage
      ├─ current
      ├─ travel_speed
      ├─ wire_feed_speed
      └─ gas_flow
```

### 3. **WeldingPlannerNode (ROS)**

**Responsabilidade**: Orquestrar todo o processamento em tempo real

```
Subscribers:
├─ /camera/image_raw (sensor_msgs/Image)
└─ Callback: image_callback

Processamento no Callback:
├─ Converte ROS Image → OpenCV
├─ Chama detector.detect_joint()
├─ Se detectado:
│  ├─ Chama classifier.plan_welding()
│  ├─ Publica em tópicos
│  └─ Log de informações
└─ Se não detectado:
   └─ Log throttled

Publishers:
├─ /welding/plan (String completo)
├─ /welding/pass_type (String)
├─ /welding/process (String)
├─ /welding/confidence (Float32)
├─ /welding/num_passes (Int32)
└─ /welding/visualized_image (Image)

Services:
├─ /welding/reset_counter
└─ /welding/get_status
```

### 4. **MovementExecutorNode (ROS)**

**Responsabilidade**: Planejar e executar movimento do robô

```
Subscribers:
├─ /welding/plan (String)
└─ Callback: plan_callback

Processamento:
├─ Parse do texto do plano
├─ Extrai parâmetros
├─ Se MoveIt disponível:
│  ├─ Cria PoseStamped
│  ├─ Planeja com move_group.plan()
│  └─ Publica JointTrajectory
└─ Se simulação:
   └─ Gera trajetória interpolada

Publishers:
├─ /welding/planned_trajectory
├─ /welding/movement_status
└─ /welding/execution_feedback

Services:
├─ /welding/execute_plan
├─ /welding/get_robot_state
└─ /welding/add_obstacles
```

### 5. **ImagePublisherNode (ROS)**

**Responsabilidade**: Capturar e publicar imagens

```
Modos:
├─ 'simulated': Gera junta artificial
├─ 'file': Lê de arquivo estático
└─ 'camera': Captura de câmera real (via OpenCV)

Publicação:
└─ /camera/image_raw (sensor_msgs/Image)
   Frequência: configurável (default: 10 Hz)
```

## 📊 Estrutura de Dados

### JointInfo
```python
@dataclass
class JointInfo:
    contour: np.ndarray              # Contorno do OpenCV
    area: float                      # Pixels²
    perimeter: float
    center: Tuple[float, float]      # (cx, cy)
    width: float
    depth: float
    angle: float                     # Graus
    is_open: bool                    # V-shape vs I-shape
    confidence: float                # 0-1
```

### WeldingPlan
```python
@dataclass
class WeldingPlan:
    pass_type: PassType
    process: WeldProcess
    num_passes: int
    parameters: dict
        ├─ voltage: float (volts)
        ├─ current: float (amps)
        ├─ travel_speed: float (mm/min)
        ├─ wire_feed_speed: float (mm/min)
        └─ gas_flow: float (L/min)
    torch_angle: float               # Graus
    start_point: Tuple[float, float, float]  # (x, y, z)
    end_point: Tuple[float, float, float]
    travel_speed: float
    confidence: float
```

## 🔌 Interfaces Externas

### ROS Topics

```
Entrada:
  /camera/image_raw (sensor_msgs/Image)
    ↓ [50-150 Hz típico]
    
Saída:
  /welding/pass_type (std_msgs/String)
  /welding/process (std_msgs/String)
  /welding/confidence (std_msgs/Float32)
  /welding/num_passes (std_msgs/Int32)
  /welding/plan (std_msgs/String)
  /welding/visualized_image (sensor_msgs/Image)
  /welding/planned_trajectory (trajectory_msgs/JointTrajectory)
  
Feedback:
  /welding/movement_status (std_msgs/String)
  /welding/execution_feedback (std_msgs/String)
```

### ROS Services

```
Entrada:
  POST /welding/reset_counter → TriggerResponse
  POST /welding/get_status → TriggerResponse
  POST /welding/execute_plan → TriggerResponse
  POST /welding/get_robot_state → TriggerResponse
  POST /welding/add_obstacles → TriggerResponse
```

## 🔄 Padrões de Design

### 1. **Separation of Concerns**
- Joint Detector: apenas detecção
- Classifier: apenas classificação
- Nodes: integração com ROS

### 2. **Factory Pattern**
- `WeldingClassifier` é uma factory para planos

### 3. **Observer Pattern**
- ROS Subscribers como observers
- Topics como eventos publicados

### 4. **Strategy Pattern**
- Diferentes estratégias de fonte de imagem (camera, file, simulated)

## 🚀 Escalabilidade

```
v1.0 (Atual):
├─ Detecção single-threaded
├─ Uma câmera
└─ Um robô

v2.0 (Planejado):
├─ Processamento paralelo
├─ Multi-câmera
├─ Rede neural (YOLO/FastSAM)
└─ Database remoto

v3.0 (Futuro):
├─ Aprendizado contínuo
├─ Múltiplos robôs coordenados
├─ Integração ERP
└─ Dashboard web em tempo real
```

## 📋 Checklist de Implementação

- ✅ Detecção de juntas (OpenCV)
- ✅ Classificação de passe e processo
- ✅ Cálculo de parâmetros
- ✅ Nodes ROS (3x)
- ✅ Launch files
- ✅ Configuração YAML
- ✅ Testes unitários
- ✅ Documentação
- ⏳ Integração MoveIt (parcial)
- ⏳ Gazebo world files
- ⏳ Dashboard web
- ⏳ Rede neural avançada

## 🔍 Considerações de Desempenho

1. **Taxa de Processamento**: ~30-50ms por frame (10 Hz)
2. **Memória**: ~500MB por nó (baseline)
3. **CPU**: ~1 core para processamento, 1 para ROS
4. **Latência E2E**: ~100-200ms (detecção + classificação)

---

**Desenvolvido com arquitectura modular para flexibilidade máxima** 🏗️
