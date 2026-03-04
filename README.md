# ALGORITMO INTELIGENTE IDENTIFICADOR DE SOLDA 🤖⚙️

[![License: GPL-3.0](https://img.shields.io/badge/License-GPL--3.0-blue.svg)](LICENSE)
[![Language: Python](https://img.shields.io/badge/Language-Python%203.8%2B-blue.svg)](https://www.python.org/)
[![Framework: ROS](https://img.shields.io/badge/Framework-ROS%201.0-green.svg)](http://www.ros.org/)
[![Status: Active Development](https://img.shields.io/badge/Status-Active%20Development-brightgreen.svg)]()

Um sistema avançado e modular para identificação automática de juntas de solda, classificação de tipo de passe (raiz, enchimento, acabamento) e seleção inteligente de processo de soldagem (**SMAW, GTAW, GMAW, FCAW**) usando visão computacional e ROS.

## 🎯 Objetivos

✨ **Detecção Automática**: Identificar juntas de solda em imagens com alta precisão  
🔧 **Classificação Inteligente**: Determinar o tipo de passe necessário  
📊 **Seleção de Processo**: Escolher o melhor processo baseado em engenharia de soldagem  
🤖 **Integração ROS**: Arquitetura completamente modular com ROS  
🎓 **Bem Documentado**: Documentação técnica completa e exemplos práticos  

## 🚀 Início Rápido

```bash
# Clone e compile
cd ~/catkin_ws/src
git clone https://github.com/novasentinel-tech/ALGORITIMO-IDENTIFICADOR-DE-SOLDA.git
cd ..
catkin build
source devel/setup.bash

# Execute
roslaunch welding_system welding_system.launch
```

**Próximo passo?** Veja [⚡ QUICK START](docs/QUICK_START.md)

## 📚 Documentação Completa

| Documento | Descrição |
|-----------|-----------|
| [**QUICK START**](docs/QUICK_START.md) | ⚡ Guia de 5 minutos para começar |
| [**README Completo**](docs/README_PT_BR.md) | 📖 Documentação técnica detalhada |
| [**ARQUITETURA**](docs/ARCHITECTURE.md) | 🏗️ Design e fluxo do sistema |

## 🌟 Características Principais

### 🔍 Detecção Inteligente de Juntas
- Processamento com **OpenCV Canny Edge Detection**
- Análise de contornos e extração de features
- Detecção automática de tipo (aberta/fechada)  
- Cálculo de profundidade estimada
- Confiança da detecção (0-100%)

### 🔧 Classificação Avançada
- Classificação de passe: **Root** (raiz) → **Fill** (enchimento) → **Cap** (acabamento)
- Seleção automática de processo:
  - **SMAW**: Eletrodo revestido
  - **GTAW**: TIG (alta precisão)
  - **GMAW**: MIG/MAG (produtivo)
  - **FCAW**: Arame tubular
- Cálculo automático de parâmetros

### 🤖 Integração ROS Completa
- 3 nós ROS coordenados
- 19 tópicos para publicação de dados
- 5 serviços para controle remoto
- Suporte a **MoveIt!** para planejamento
- Compatível com **Gazebo** para simulação

## 📦 Estrutura do Projeto

```
welding_system/
├── src/
│   ├── joint_detector.py
│   └── examples.py
├── scripts/
│   ├── image_publisher_node.py
│   ├── welding_planner_node.py
│   └── movement_executor_node.py
├── launch/
│   ├── welding_system.launch
│   └── gazebo_welding.launch
├── config/
│   └── welding_params.yaml
├── test/
│   └── test_joint_detector.py
└── docs/
    ├── QUICK_START.md
    ├── README_PT_BR.md
    └── ARCHITECTURE.md
```

## 🔄 Fluxo de Processamento

```
Câmera → Detecção de Junta → Classificação → Seleção de Processo → 
Planejamento de Movimento → Execução no Robô
```

## 💻 Exemplos Rápidos

**Usar o módulo diretamente:**

```python
from joint_detector import WeldingJointDetector, WeldingClassifier
import cv2

image = cv2.imread("junta.jpg")
detector = WeldingJointDetector()
joint = detector.detect_joint(image)

if joint:
    classifier = WeldingClassifier()
    plan = classifier.plan_welding(joint)
    print(f"Processo: {plan.process.value}")
```

**Usar via ROS:**

```bash
roslaunch welding_system welding_system.launch
rostopic echo /welding/process
```

## 🧪 Testes

```bash
python3 src/welding_system/test/test_joint_detector.py
python3 src/welding_system/examples.py
```

## 📋 Dependências

- **ROS** (Noetic ou Humble)
- **Python 3.8+**
- **OpenCV**: `pip install opencv-python`
- **NumPy**: `pip install numpy`
- **MoveIt** (opcional): `sudo apt install ros-noetic-moveit`

## 🛠️ Instalação

```bash
cd ~/catkin_ws/src
git clone https://github.com/novasentinel-tech/ALGORITIMO-IDENTIFICADOR-DE-SOLDA.git
cd ..
rosdep install --from-paths src --ignore-src -r -y
catkin build welding_system
source devel/setup.bash
```

## 📈 Roadmap

- [ ] Suporte a YOLO/YOLOv8
- [ ] Dashboard web
- [ ] Múltiplas câmeras
- [ ] Banco de dados de padrões
- [ ] Integração ERP/MES

## 👏 Créditos

Desenvolvido por **João Pedro Rodrigues Viana** - GPL-3.0

🙏 Se usar ou modificar, **por favor mencione este repositório**

---

**Desenvolvido com ❤️ para tornar a soldagem mais inteligente** 🤖
