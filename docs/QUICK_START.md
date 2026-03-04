# Guia de Início Rápido

## ⚡ Instalação em 5 Minutos

### Pré-requisitos

```bash
# Ubuntu 20.04 LTS ou 22.04 LTS
# ROS Noetic ou Humble já instalado
# Python 3.8+
```

### 1. Clone e Build

```bash
# Clone o repositório
cd ~/catkin_ws/src
git clone https://github.com/novasentinel-tech/ALGORITIMO-IDENTIFICADOR-DE-SOLDA.git
cd ..

# Instale dependências
rosdep install --from-paths src --ignore-src -r -y

# Compile
catkin build welding_system
source devel/setup.bash
```

### 2. Teste Rápido

```bash
# Terminal 1: Inicia o sistema completo
roslaunch welding_system welding_system.launch

# Terminal 2: Verifica se está funcionando
rostopic list | grep welding

# Terminal 3: Vê os planos sendo gerados
rostopic echo /welding/process

# Terminal 4: Reseta contador
rosservice call /welding/reset_counter
```

## 📊 Primeiro Uso

### Entenda o Fluxo

```
Câmera (simulada)
    ↓
[Imagem ROS] → /camera/image_raw
    ↓
Detector detecta junta
    ↓
[Plan gerado] → /welding/plan
    ↓
Status publicado:
  - /welding/pass_type (root/fill/cap)
  - /welding/process (SMAW/GTAW/GMAW/FCAW)
  - /welding/confidence (0-1)
```

### Monitore em Tempo Real

```bash
# Em 3 terminais diferentes:

# Terminal A: Imagens com detecção
rosrun image_view image_view image:=/welding/visualized_image

# Terminal B: Informações em tempo real
watch -n 0.5 'rostopic echo -n 1 /welding/pass_type & rostopic echo -n 1 /welding/process'

# Terminal C: Estatísticas
rostopic hz /welding/plan
```

## 🔧 Personalização Rápida

### Mudar o Material

```bash
roslaunch welding_system welding_system.launch material:=aluminum
```

Opções: `steel`, `stainless_steel`, `aluminum`

### Usar Câmera Real

**Arquivo: `src/welding_system/launch/welding_system.launch`**

Encontre:
```xml
<param name="image_source" value="simulated"/>
```

Mude para:
```xml
<param name="image_source" value="camera"/>
<param name="camera_id" value="0"/>  <!-- ou 1, 2, etc -->
```

Depois relance:
```bash
roslaunch welding_system welding_system.launch
```

### Usar Arquivo de Imagem

```bash
# Crie uma imagem de teste
# ou use: src/welding_system/launch/welding_system.launch

# Edite:
<param name="image_source" value="file"/>
<param name="image_file" value="/caminho/para/sua/imagem.jpg"/>
```

## 🧪 Testes Rápidos

### Teste Unitário

```bash
cd ~/catkin_ws/src/ALGORITIMO-IDENTIFICADOR-DE-SOLDA/src/welding_system
python3 test/test_joint_detector.py -v
```

### Teste o Detector Diretamente

```python
python3 << 'EOF'
import sys
sys.path.insert(0, '/path/to/welding_system/src')

from joint_detector import WeldingJointDetector, WeldingClassifier
import cv2

# Cria imagem de teste
img = cv2.imread('test_image.jpg')
if img is None:
    print("Imagem não encontrada, criando junta simulada...")
    img = cv2.ones((480, 640, 3), dtype='uint8') * 200
    cv2.line(img, (200, 200), (400, 200), (0, 0, 0), 5)
    cv2.line(img, (200, 200), (300, 350), (0, 0, 0), 5)
    cv2.line(img, (400, 200), (300, 350), (0, 0, 0), 5)

# Detecta
detector = WeldingJointDetector()
joint = detector.detect_joint(img)

if joint:
    print(f"✓ Junta detectada")
    print(f"  Área: {joint.area:.0f} px²")
    print(f"  Confiança: {joint.confidence:.1%}")
    
    # Classifica
    classifier = WeldingClassifier()
    plan = classifier.plan_welding(joint)
    print(f"\n✓ Plano gerado")
    print(f"  Passe: {plan.pass_type.value}")
    print(f"  Processo: {plan.process.value}")
    print(f"  Passes necessários: {plan.num_passes}")
else:
    print("✗ Junta não detectada")

EOF
```

## 📱 Interfaces Visuais

### RQT - Monitor de Tópicos

```bash
# Em um terminal
rqt

# Vá para Plugins → Topics → Topic Monitor
# Selecione /welding/* para monitorar
```

### RViz - Visualização 3D

```bash
# Se você tiver um robô URDF configurado
roslaunch welding_system gazebo_welding.launch
rviz -d src/welding_system/config/welding.rviz
```

## 🔍 Diagnóstico

### Verifique se os nós estão rodando

```bash
rosnode list

# Esperado:
# /image_publisher_node
# /welding_planner_node
# /movement_executor_node
# /rosout
```

### Verifique os tópicos

```bash
rostopic list | grep -E "camera|welding"

# Esperado:
# /camera/image_raw
# /welding/confidence
# /welding/num_passes
# /welding/pass_type
# /welding/plan
# /welding/process
# /welding/visualized_image
# /welding/movement_status
# ...
```

### Teste um tópico

```bash
# Uma vez
rostopic echo -n 1 /welding/process

# Continuamente
rostopic echo /welding/pass_type

# Com frequência
rostopic hz /welding/plan
```

## ⚙️ Parâmetros Comuns

### Ajustar Taxa de Publicação

```bash
roslaunch welding_system welding_system.launch publish_rate:=5
```

### Desabilitar Visualização

```bash
roslaunch welding_system welding_system.launch visualize:=false
```

### Modo Debug

```bash
# Aumenta saída de log
rosparam set /welding_planner_node/enable_logging true
rosparam set /welding_planner_node/enable_logging false
```

## 🚀 Próximos Passos

1. **Entenda o código**: Leia `src/joint_detector.py`
2. **Modifique parâmetros**: Edite `config/welding_params.yaml`
3. **Adicione suas imagens**: Use fonte 'file' com seus dados
4. **Integre com seu robô**: Implemente `MoveGroupCommander`
5. **Implante em produção**: Use launch files para ambiente real

## 💡 Dicas

- Sempre rode em um terminal ROS com `source devel/setup.bash`
- Use `rosclean purge` se tiver problemas com cache
- Verifique logs com `roslaunch -v`
- Teste partes individuais antes de integrar

## 🐛 Problemas Comuns

### "No module named rospy"
```bash
# Instale ROS Python
sudo apt install python3-rospy
```

### "Cannot import cv_bridge"
```bash
# Instale CV Bridge
sudo apt install ros-noetic-cv-bridge  # ou humble, focal, etc
```

### Câmera não funciona
```bash
# Teste sua câmera
v4l2-ctl --list-devices
ls -la /dev/video*
```

### MoveIt não encontrado
```bash
# Instale MoveIt
sudo apt install ros-noetic-moveit
```

## 📚 Documentação Completa

Veja `docs/README_PT_BR.md` para documentação detalhada.

## 🤝 Contribua

Sugira melhorias em: https://github.com/novasentinel-tech/ALGORITIMO-IDENTIFICADOR-DE-SOLDA/issues

---

**Pronto?** Rode agora mesmo:

```bash
cd ~/catkin_ws
source devel/setup.bash
roslaunch welding_system welding_system.launch
```

🎉 Boa sorte!
