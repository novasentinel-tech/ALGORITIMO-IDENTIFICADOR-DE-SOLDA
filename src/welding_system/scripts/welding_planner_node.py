#!/usr/bin/env python3
"""
Nó de Planejamento de Soldagem (Welding Planner)
Nó principal que:
1. Recebe imagens de uma câmera
2. Detecta juntas de solda
3. Classifica o tipo de passe necessário
4. Seleciona processo apropriado
5. Publica plano de soldagem
"""

import rospy
import cv2
import sys
import os
from sensor_msgs.msg import Image
from std_msgs.msg import String, Float32, Int32
from geometry_msgs.msg import Pose, Point, Quaternion
from cv_bridge import CvBridge
from std_srvs.srv import Trigger, TriggerResponse

# Adiciona o path para importar o módulo de detecção
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'src'))

from joint_detector import (
    WeldingJointDetector, WeldingClassifier, JointInfo, 
    PassType, WeldProcess, visualize_joint_detection
)


class WeldingPlannerNode:
    """Node principal de planejamento de soldagem"""
    
    def __init__(self):
        """Inicializa o nó de planejamento"""
        rospy.init_node('welding_planner_node', anonymous=True)
        
        # Parâmetros
        self.material = rospy.get_param('~material', 'steel')
        self.visualize = rospy.get_param('~visualize', True)
        self.enable_logging = rospy.get_param('~enable_logging', True)
        
        # Detectores e classificadores
        self.joint_detector = WeldingJointDetector()
        self.welding_classifier = WeldingClassifier()
        
        # ROS Components
        self.bridge = CvBridge()
        
        # Subscribers
        self.image_sub = rospy.Subscriber(
            '/camera/image_raw', Image, 
            self.image_callback, queue_size=1
        )
        
        # Publishers
        self.plan_pub = rospy.Publisher(
            '/welding/plan', String, queue_size=10
        )
        self.pass_type_pub = rospy.Publisher(
            '/welding/pass_type', String, queue_size=10
        )
        self.process_pub = rospy.Publisher(
            '/welding/process', String, queue_size=10
        )
        self.confidence_pub = rospy.Publisher(
            '/welding/confidence', Float32, queue_size=10
        )
        self.num_passes_pub = rospy.Publisher(
            '/welding/num_passes', Int32, queue_size=10
        )
        
        # Visualization Publisher
        if self.visualize:
            self.vis_pub = rospy.Publisher(
                '/welding/visualized_image', Image, queue_size=1
            )
        
        # Services
        rospy.Service('/welding/reset_counter', Trigger, self.handle_reset_counter)
        rospy.Service('/welding/get_status', Trigger, self.handle_get_status)
        
        # State variables
        self.joint_counter = 0
        self.last_joint_info = None
        self.last_plan = None
        self.frames_processed = 0
        
        rospy.loginfo("Welding Planner Node iniciado")
        rospy.loginfo(f"Material: {self.material}")
        rospy.loginfo(f"Visualização: {self.visualize}")
    
    def image_callback(self, msg: Image):
        """
        Callback chamado quando uma nova imagem é recebida
        
        Args:
            msg: Mensagem ROS com a imagem
        """
        try:
            # Converte ROS Image para OpenCV
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            
            self.frames_processed += 1
            
            # Detecta junta
            joint_info = self.joint_detector.detect_joint(cv_image)
            
            if joint_info is not None:
                self.joint_counter += 1
                self.last_joint_info = joint_info
                
                # Classifica e planeja soldagem
                self.process_joint(joint_info)
                
                # Visualização
                if self.visualize:
                    self.publish_visualization(cv_image, joint_info)
            
            else:
                if self.enable_logging and self.frames_processed % 30 == 0:
                    rospy.logwarn_throttle(5, "Nenhuma junta detectada")
        
        except Exception as e:
            rospy.logerr(f"Erro no processamento de imagem: {e}")
    
    def process_joint(self, joint_info: JointInfo):
        """
        Processa uma junta detectada e gera plano de soldagem
        
        Args:
            joint_info: Informações da junta detectada
        """
        try:
            # Classifica passe (sempre começa com passe raiz)
            pass_number = 1
            
            # Planeja a soldagem
            plan = self.welding_classifier.plan_welding(
                joint_info, 
                pass_number=pass_number,
                material=self.material
            )
            
            self.last_plan = plan
            
            # Publica informações
            self.publish_welding_plan(plan, joint_info)
            
            if self.enable_logging:
                self.log_welding_plan(plan, joint_info)
        
        except Exception as e:
            rospy.logerr(f"Erro ao processar junta: {e}")
    
    def publish_welding_plan(self, plan, joint_info: JointInfo):
        """
        Publica o plano de soldagem em tópicos ROS
        
        Args:
            plan: Plano de soldagem
            joint_info: Informações da junta
        """
        # Publica tipo de passe
        self.pass_type_pub.publish(plan.pass_type.value)
        
        # Publica processo
        self.process_pub.publish(plan.process.value)
        
        # Publica confiança
        self.confidence_pub.publish(Float32(plan.confidence))
        
        # Publica número de passes
        self.num_passes_pub.publish(Int32(plan.num_passes))
        
        # Publica plano completo em formato texto
        plan_text = self.format_plan_as_string(plan, joint_info)
        self.plan_pub.publish(plan_text)
    
    def format_plan_as_string(self, plan, joint_info: JointInfo) -> str:
        """
        Formata o plano de soldagem como texto
        
        Args:
            plan: Plano de soldagem
            joint_info: Informações da junta
            
        Returns:
            String formatada com o plano
        """
        plan_str = f"""
=== PLANO DE SOLDAGEM ===
Junta #{self.joint_counter}:
  Tipo de Passe: {plan.pass_type.value}
  Processo: {plan.process.value}
  Material: {self.material}
  
Características da Junta:
  Área: {joint_info.area:.2f} px²
  Largura: {joint_info.width:.2f} px
  Profundidade: {joint_info.depth:.2f} px
  Ângulo: {joint_info.angle:.2f}°
  Junta Aberta: {"Sim" if joint_info.is_open else "Não"}
  Confiança: {joint_info.confidence:.2%}

Parâmetros de Soldagem:
  Número de Passes: {plan.num_passes}
  Tensão: {plan.parameters.get('voltage', 'N/A')} V
  Corrente: {plan.parameters.get('current', 'N/A')} A
  Velocidade de Avanço: {plan.parameters.get('travel_speed', 'N/A')} mm/min
  Velocidade do Arame: {plan.parameters.get('wire_feed_speed', 'N/A')} mm/min
  Vazão de Gás: {plan.parameters.get('gas_flow', 'N/A')} L/min

Geometria da Tocha:
  Ângulo: {plan.torch_angle:.2f}°
  Ponto de Início: {plan.start_point}
  Ponto de Fim: {plan.end_point}
  Velocidade de Deslocamento: {plan.travel_speed} mm/min

Confiança do Plano: {plan.confidence:.2%}
========================
"""
        return plan_str
    
    def publish_visualization(self, cv_image, joint_info: JointInfo):
        """
        Publica uma imagem com visualização da detecção
        
        Args:
            cv_image: Imagem em OpenCV
            joint_info: Informações da junta
        """
        try:
            # Visualiza detecção
            vis_image = visualize_joint_detection(cv_image, joint_info)
            
            # Adiciona informações do plano
            if self.last_plan:
                text = f"{self.last_plan.pass_type.value.upper()} | {self.last_plan.process.value}"
                cv2.putText(vis_image, text, (10, 60), cv2.FONT_HERSHEY_SIMPLEX,
                           0.7, (0, 255, 0), 2)
            
            # Publica
            ros_image = self.bridge.cv2_to_imgmsg(vis_image, encoding="bgr8")
            self.vis_pub.publish(ros_image)
        
        except Exception as e:
            rospy.logwarn(f"Erro ao publicar visualização: {e}")
    
    def log_welding_plan(self, plan, joint_info: JointInfo):
        """
        Registra informações do plano em log
        
        Args:
            plan: Plano de soldagem
            joint_info: Informações da junta
        """
        rospy.loginfo(
            f"Junta detectada #{self.joint_counter}: "
            f"Passe={plan.pass_type.value}, "
            f"Processo={plan.process.value}, "
            f"Passes={plan.num_passes}, "
            f"Conf={plan.confidence:.2%}"
        )
    
    def handle_reset_counter(self, req):
        """
        Service para resetar o contador de juntas
        
        Args:
            req: Requisição do serviço
            
        Returns:
            Resposta do serviço
        """
        self.joint_counter = 0
        rospy.loginfo("Contador de juntas resetado")
        return TriggerResponse(success=True, message="Contador resetado")
    
    def handle_get_status(self, req):
        """
        Service para obter status do nó
        
        Args:
            req: Requisição do serviço
            
        Returns:
            Resposta com status
        """
        status = f"""
Status do Welding Planner:
  Juntas Detectadas: {self.joint_counter}
  Frames Processados: {self.frames_processed}
  Última Junta Detectada: {self.last_joint_info is not None}
  Plano Ativo: {self.last_plan is not None}
"""
        return TriggerResponse(success=True, message=status)
    
    def run(self):
        """Loop principal do nó"""
        rospy.loginfo("Welding Planner Node rodando")
        
        try:
            rospy.spin()
        except KeyboardInterrupt:
            rospy.loginfo("Interrupção do usuário")
        finally:
            rospy.loginfo("Welding Planner Node finalizado")
            rospy.loginfo(f"Total de juntas detectadas: {self.joint_counter}")
            rospy.loginfo(f"Total de frames processados: {self.frames_processed}")


if __name__ == '__main__':
    try:
        node = WeldingPlannerNode()
        node.run()
    except rospy.ROSInterruptException:
        pass
    except Exception as e:
        rospy.logerr(f"Erro no Welding Planner Node: {e}")
