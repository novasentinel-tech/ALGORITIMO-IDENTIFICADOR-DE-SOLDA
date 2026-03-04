#!/usr/bin/env python3
"""
Nó de Execução de Movimento (Movement Executor)
Recebe o plano de soldagem e executa a trajetória do robô usando MoveIt
"""

import rospy
import sys
import os
from std_msgs.msg import String, Bool, Float32MultiArray
from geometry_msgs.msg import Pose, Point, Quaternion, PoseStamped
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from std_srvs.srv import Trigger, TriggerResponse
from rospy.exceptions import ROSException

# Adiciona o path para importar módulos locais
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'src'))

# Tenta importar MoveIt (pode não estar instalado em development)
try:
    import moveit_commander
    import moveit_msgs.msg
    MOVEIT_AVAILABLE = True
except ImportError:
    rospy.logwarn("MoveIt não disponível - usando simulação")
    MOVEIT_AVAILABLE = False


class MovementExecutorNode:
    """Node para executar movimento do robô com planejamento MoveIt"""
    
    def __init__(self):
        """Inicializa o nó executor de movimento"""
        rospy.init_node('movement_executor_node', anonymous=True)
        
        # Parâmetros
        self.robot_name = rospy.get_param('~robot_name', 'manipulator')
        self.simulation_mode = rospy.get_param('~simulation_mode', not MOVEIT_AVAILABLE)
        self.execute_movement = rospy.get_param('~execute_movement', False)
        self.planning_time = rospy.get_param('~planning_time', 5.0)
        
        # MoveIt configuration (se disponível)
        self.move_group = None
        self.scene = None
        self.robot = None
        
        if MOVEIT_AVAILABLE and not self.simulation_mode:
            try:
                moveit_commander.roscpp_initialize(sys.argv)
                self.robot = moveit_commander.RobotCommander()
                self.scene = moveit_commander.PlanningSceneInterface()
                self.move_group = moveit_commander.MoveGroupCommander(self.robot_name)
                self.move_group.set_planning_time(self.planning_time)
                rospy.loginfo("MoveIt inicializado com sucesso")
                MOVEIT_AVAILABLE = True
            except Exception as e:
                rospy.logwarn(f"Erro ao inicializar MoveIt: {e}")
                self.simulation_mode = True
        else:
            self.simulation_mode = True
        
        # ROS Components
        # Subscribers
        self.plan_sub = rospy.Subscriber(
            '/welding/plan', String,
            self.plan_callback, queue_size=10
        )
        
        # Publishers
        self.trajectory_pub = rospy.Publisher(
            '/welding/planned_trajectory', JointTrajectory, queue_size=10
        )
        self.status_pub = rospy.Publisher(
            '/welding/movement_status', String, queue_size=10
        )
        self.execution_pub = rospy.Publisher(
            '/welding/execution_feedback', String, queue_size=10
        )
        
        # Services
        rospy.Service('/welding/execute_plan', Trigger, self.handle_execute_plan)
        rospy.Service('/welding/get_robot_state', Trigger, self.handle_get_robot_state)
        rospy.Service('/welding/add_obstacles', Trigger, self.handle_add_obstacles)
        
        # State
        self.current_plan = None
        self.current_plan_text = ""
        self.execution_in_progress = False
        
        rospy.loginfo("Movement Executor Node inicializado")
        rospy.loginfo(f"Modo Simulação: {self.simulation_mode}")
        if self.move_group:
            rospy.loginfo(f"Grupo de Movimento: {self.robot_name}")
    
    def plan_callback(self, msg: String):
        """
        Callback para receber o plano de soldagem
        
        Args:
            msg: Mensagem com o plano
        """
        try:
            self.current_plan_text = msg.data
            
            # Extrai informações do plano
            self.parse_plan(msg.data)
            
            # Planeja movimento automaticamente
            if not self.execution_in_progress:
                self.plan_welding_path()
        
        except Exception as e:
            rospy.logerr(f"Erro ao processar plano: {e}")
    
    def parse_plan(self, plan_text: str):
        """
        Parse do texto do plano para extrair informações
        
        Args:
            plan_text: Texto do plano de soldagem
        """
        # Extrai informações chave do plano (método simples)
        lines = plan_text.split('\n')
        self.current_plan = {
            'pass_type': None,
            'process': None,
            'torch_angle': 0,
            'start_point': [0, 0, 0],
            'end_point': [0, 0, 0],
            'travel_speed': 300
        }
        
        for line in lines:
            if 'Tipo de Passe' in line:
                self.current_plan['pass_type'] = line.split(':')[1].strip()
            elif 'Processo' in line and 'Processo:' in line:
                self.current_plan['process'] = line.split(':')[1].strip()
            elif 'Ângulo' in line and 'Ângulo:' in line:
                try:
                    self.current_plan['torch_angle'] = float(
                        line.split(':')[1].strip().split('°')[0]
                    )
                except:
                    pass
    
    def plan_welding_path(self):
        """
        Planeja a trajetória de soldagem
        """
        try:
            if self.current_plan is None:
                rospy.logwarn("Nenhum plano disponível")
                return
            
            rospy.loginfo(f"Planejando trajetória para {self.current_plan['process']}")
            
            if self.simulation_mode:
                self.simulate_trajectory()
            else:
                self.plan_with_moveit()
        
        except Exception as e:
            rospy.logerr(f"Erro ao planejar trajetória: {e}")
    
    def simulate_trajectory(self):
        """
        Simula planejamento de trajetória (modo desenvolvimento)
        """
        try:
            # Cria trajetória simulada
            trajectory = JointTrajectory()
            trajectory.header.frame_id = "base_link"
            trajectory.joint_names = [
                'joint1', 'joint2', 'joint3',
                'joint4', 'joint5', 'joint6'
            ]
            
            # Simula pontos de waypoint
            num_points = 5
            for i in range(num_points):
                point = JointTrajectoryPoint()
                
                # Cria uma trajetória interpolada simples
                t = i / max(1, num_points - 1)
                start = [0.0, -1.57, 1.57, 0.0, 0.0, 0.0]
                end = [0.0, -1.2, 1.57, 0.5, 0.0, 0.0]
                
                point.positions = [
                    s + (e - s) * t for s, e in zip(start, end)
                ]
                point.velocities = [0.5] * 6
                point.time_from_start = rospy.Duration(i * 2.0)
                
                trajectory.points.append(point)
            
            # Publica trajetória
            self.trajectory_pub.publish(trajectory)
            
            msg = f"Trajetória simulada planejada para {self.current_plan['process']}"
            rospy.loginfo(msg)
            self.status_pub.publish(msg)
        
        except Exception as e:
            rospy.logerr(f"Erro ao simular trajetória: {e}")
    
    def plan_with_moveit(self):
        """
        Planeja trajetória usando MoveIt
        """
        if not self.move_group:
            rospy.logwarn("MoveIt não disponível")
            self.simulate_trajectory()
            return
        
        try:
            rospy.loginfo("Usando MoveIt para planejar trajetória")
            
            # Define pose alvo (exemplo simplificado)
            target_pose = PoseStamped()
            target_pose.header.frame_id = "base_link"
            target_pose.pose.position.x = 0.5
            target_pose.pose.position.y = 0.0
            target_pose.pose.position.z = 0.5
            target_pose.pose.orientation.w = 1.0
            
            # Planeja
            self.move_group.set_pose_target(target_pose)
            plan = self.move_group.plan()
            
            if plan and len(plan.joint_trajectory.points) > 0:
                # Publica trajectory
                self.trajectory_pub.publish(plan.joint_trajectory)
                
                msg = "Trajetória planejada com sucesso usando MoveIt"
                rospy.loginfo(msg)
                self.status_pub.publish(msg)
            else:
                rospy.logwarn("Falha ao planejar trajetória com MoveIt")
                self.simulate_trajectory()
        
        except Exception as e:
            rospy.logwarn(f"Erro no planejamento com MoveIt: {e}")
            self.simulate_trajectory()
    
    def handle_execute_plan(self, req):
        """
        Service para executar o plano planejado
        
        Args:
            req: Requisição do serviço
            
        Returns:
            Resposta do serviço
        """
        if self.execution_in_progress:
            return TriggerResponse(
                success=False,
                message="Execução já em progresso"
            )
        
        try:
            self.execution_in_progress = True
            msg = "Iniciando execução de movimento"
            rospy.loginfo(msg)
            self.execution_pub.publish(msg)
            
            if self.execute_movement and self.move_group:
                # Executa no robô real/simulado
                self.move_group.go(wait=True)
                msg = "Movimento executado com sucesso"
            else:
                # Caminha por uma trajectory apenas (para simulação)
                msg = "Trajetória enviada para execução"
            
            rospy.loginfo(msg)
            self.execution_pub.publish(msg)
            self.execution_in_progress = False
            
            return TriggerResponse(success=True, message=msg)
        
        except Exception as e:
            self.execution_in_progress = False
            msg = f"Erro ao executar movimento: {e}"
            rospy.logerr(msg)
            return TriggerResponse(success=False, message=msg)
    
    def handle_get_robot_state(self, req):
        """
        Service para obter estado do robô
        
        Args:
            req: Requisição do serviço
            
        Returns:
            Resposta com estado
        """
        try:
            if self.move_group:
                current_joints = self.move_group.get_current_joint_values()
                current_pose = self.move_group.get_current_pose()
                
                state = f"""
Estado do Robô:
  Configuração Atual: {[f"{j:.3f}" for j in current_joints]}
  Pose Atual:
    X: {current_pose.pose.position.x:.3f}
    Y: {current_pose.pose.position.y:.3f}
    Z: {current_pose.pose.position.z:.3f}
  Modo Simulação: {self.simulation_mode}
"""
            else:
                state = "Robô em modo simulação - sem estado real"
            
            return TriggerResponse(success=True, message=state)
        
        except Exception as e:
            return TriggerResponse(
                success=False,
                message=f"Erro ao obter estado: {e}"
            )
    
    def handle_add_obstacles(self, req):
        """
        Service para adicionar obstáculos à cena
        
        Args:
            req: Requisição do serviço
            
        Returns:
            Resposta do serviço
        """
        if not self.scene:
            return TriggerResponse(
                success=False,
                message="Planning scene não disponível"
            )
        
        try:
            # Adiciona uma bancada de trabalho (exemplo)
            box_pose = PoseStamped()
            box_pose.header.frame_id = "base_link"
            box_pose.pose.position.x = 0.5
            box_pose.pose.position.y = 0.0
            box_pose.pose.position.z = -0.1
            
            self.scene.add_box(
                "work_table",
                box_pose,
                size=(1.0, 1.0, 0.1)
            )
            
            msg = "Obstáculos adicionados à cena"
            rospy.loginfo(msg)
            return TriggerResponse(success=True, message=msg)
        
        except Exception as e:
            msg = f"Erro ao adicionar obstáculos: {e}"
            rospy.logerr(msg)
            return TriggerResponse(success=False, message=msg)
    
    def run(self):
        """Loop principal do nó"""
        rospy.loginfo("Movement Executor Node rodando")
        
        try:
            rospy.spin()
        except KeyboardInterrupt:
            rospy.loginfo("Interrupção do usuário")
        finally:
            if self.move_group:
                moveit_commander.roscpp_shutdown()
            rospy.loginfo("Movement Executor Node finalizado")


if __name__ == '__main__':
    try:
        node = MovementExecutorNode()
        node.run()
    except rospy.ROSInterruptException:
        pass
    except Exception as e:
        rospy.logerr(f"Erro no Movement Executor Node: {e}")
