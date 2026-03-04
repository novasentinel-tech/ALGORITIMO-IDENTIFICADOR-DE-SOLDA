#!/usr/bin/env python3
"""
Nó de Publicação de Imagens
Publica imagens de câmera (simulada ou real) em um tópico ROS
Pode simular uma câmera capturando juntas de solda
"""

import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import os
import time


class ImagePublisherNode:
    """Node para publicar imagens da câmera"""
    
    def __init__(self):
        """Inicializa o nó publicador de imagens"""
        rospy.init_node('image_publisher_node', anonymous=True)
        
        # Parâmetros
        self.publish_rate = rospy.get_param('~publish_rate', 10)  # Hz
        self.image_source = rospy.get_param('~image_source', 'simulated')  # 'simulated', 'camera', or 'file'
        self.image_file = rospy.get_param('~image_file', None)
        self.camera_id = rospy.get_param('~camera_id', 0)
        
        # Publisher
        self.image_pub = rospy.Publisher('/camera/image_raw', Image, queue_size=10)
        self.info_pub = rospy.Publisher('/camera/camera_info', Image, queue_size=10)
        
        # OpenCV bridge
        self.bridge = CvBridge()
        
        # Variáveis de estado
        self.frame_count = 0
        self.last_frame = None
        self.video_capture = None
        
        rospy.loginfo("Image Publisher Node iniciado")
        rospy.loginfo(f"Fonte de imagem: {self.image_source}")
    
    def _generate_simulated_image(self) -> np.ndarray:
        """
        Gera uma imagem simulada de uma junta de solda
        
        Returns:
            Imagem em formato BGR
        """
        # Cria imagem branca
        image = np.ones((480, 640, 3), dtype=np.uint8) * 200
        
        # Adiciona uma junta de solda simulada (linha V-shaped)
        center_x = 320
        center_y = 240
        
        # Desenha uma junta em V
        pts = np.array([
            [center_x - 50, center_y - 80],
            [center_x, center_y + 50],
            [center_x + 50, center_y - 80]
        ], np.int32)
        
        cv2.polylines(image, [pts], False, (0, 0, 0), 3)
        
        # Adiciona um pouco de ruído para tornar mais realista
        noise = np.random.normal(0, 10, image.shape).astype(np.uint8)
        image = cv2.add(image, noise)
        
        # Adiciona timestamp
        timestamp = f"Frame: {self.frame_count}"
        cv2.putText(image, timestamp, (10, 30), cv2.FONT_HERSHEY_SIMPLEX,
                    1, (0, 0, 0), 2)
        
        return image
    
    def _read_image_from_file(self) -> np.ndarray:
        """
        Lê imagem de um arquivo
        
        Returns:
            Imagem em formato BGR, ou None se erro
        """
        if self.image_file is None or not os.path.exists(self.image_file):
            rospy.logwarn(f"Arquivo de imagem não encontrado: {self.image_file}")
            return None
        
        image = cv2.imread(self.image_file)
        
        if image is None:
            rospy.logerr(f"Erro ao ler imagem: {self.image_file}")
            return None
        
        # Redimensiona se necessário
        if image.shape[0] > 1080 or image.shape[1] > 1920:
            scale = min(1080 / image.shape[0], 1920 / image.shape[1])
            image = cv2.resize(image, None, fx=scale, fy=scale)
        
        return image
    
    def _read_image_from_camera(self) -> np.ndarray:
        """
        Lê imagem de uma câmera conectada
        
        Returns:
            Imagem em formato BGR, ou None se erro
        """
        if self.video_capture is None:
            self.video_capture = cv2.VideoCapture(self.camera_id)
            
            if not self.video_capture.isOpened():
                rospy.logerr(f"Não foi possível abrir câmera {self.camera_id}")
                return None
            
            # Define resolução
            self.video_capture.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
            self.video_capture.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
            self.video_capture.set(cv2.CAP_PROP_FPS, 30)
        
        ret, frame = self.video_capture.read()
        
        if not ret:
            rospy.logwarn("Erro ao capturar frame da câmera")
            return None
        
        return frame
    
    def get_current_image(self) -> np.ndarray:
        """
        Obtém uma imagem baseada na fonte configurada
        
        Returns:
            Imagem em formato BGR
        """
        if self.image_source == 'simulated':
            return self._generate_simulated_image()
        
        elif self.image_source == 'file':
            image = self._read_image_from_file()
            return image if image is not None else self._generate_simulated_image()
        
        elif self.image_source == 'camera':
            image = self._read_image_from_camera()
            return image if image is not None else self._generate_simulated_image()
        
        else:
            rospy.logwarn(f"Fonte de imagem desconhecida: {self.image_source}")
            return self._generate_simulated_image()
    
    def publish_image(self):
        """Publica uma imagem no tópico"""
        try:
            # Obtém imagem
            cv_image = self.get_current_image()
            
            if cv_image is None:
                rospy.logwarn("Falha ao obter imagem")
                return
            
            # Converte para mensagem ROS
            ros_image = self.bridge.cv2_to_imgmsg(cv_image, encoding="bgr8")
            
            # Publica
            self.image_pub.publish(ros_image)
            
            self.frame_count += 1
            
            if self.frame_count % 30 == 0:
                rospy.logdebug(f"Publicados {self.frame_count} frames")
        
        except Exception as e:
            rospy.logerr(f"Erro ao publicar imagem: {e}")
    
    def run(self):
        """Loop principal do nó"""
        rospy.loginfo(f"Iniciando publicação a {self.publish_rate} Hz")
        
        rate = rospy.Rate(self.publish_rate)
        
        try:
            while not rospy.is_shutdown():
                self.publish_image()
                rate.sleep()
        
        except KeyboardInterrupt:
            rospy.loginfo("Interrupção do usuário")
        
        finally:
            if self.video_capture is not None:
                self.video_capture.release()
            rospy.loginfo("Image Publisher Node finalizado")


if __name__ == '__main__':
    try:
        node = ImagePublisherNode()
        node.run()
    except rospy.ROSInterruptException:
        pass
    except Exception as e:
        rospy.logerr(f"Erro no Image Publisher Node: {e}")
