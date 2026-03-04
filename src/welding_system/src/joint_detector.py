"""
Módulo de Detecção de Juntas de Solda
Utiliza OpenCV para detectar e classificar juntas de soldagem em imagens
"""

import cv2
import numpy as np
from enum import Enum
from dataclasses import dataclass
from typing import Tuple, List, Optional


class PassType(Enum):
    """Tipos de passe de soldagem"""
    ROOT = "root"          # Passe raiz
    FILL = "fill"          # Passe de enchimento
    CAP = "cap"            # Passe de acabamento


class WeldProcess(Enum):
    """Processos de soldagem suportados"""
    SMAW = "SMAW"          # Eletrodo revestido
    GTAW = "GTAW"          # TIG - Alta precisão
    GMAW = "GMAW"          # MIG/MAG - Produtivo
    FCAW = "FCAW"          # Arame tubular


@dataclass
class JointInfo:
    """Informações sobre uma junta de solda detectada"""
    contour: np.ndarray                    # Contorno da junta
    area: float                            # Área da junta em pixels²
    perimeter: float                       # Perímetro da junta
    center: Tuple[float, float]           # Centro (x, y)
    width: float                          # Largura estimada
    depth: float                          # Profundidade estimada
    angle: float                          # Ângulo da junta em graus
    is_open: bool                         # Se é uma junta aberta (V shape)
    confidence: float                     # Confiança da detecção (0-1)


@dataclass
class WeldingPlan:
    """Plano de soldagem para uma junta"""
    pass_type: PassType                   # Tipo de passe
    process: WeldProcess                  # Processo de soldagem
    num_passes: int                       # Número de passes necessários
    parameters: dict                      # Parâmetros de soldagem
    torch_angle: float                    # Ângulo da tocha em graus
    start_point: Tuple[float, float, float]    # Ponto de início (x, y, z)
    end_point: Tuple[float, float, float]      # Ponto de fim (x, y, z)
    travel_speed: float                   # Velocidade de deslocamento (mm/min)
    confidence: float                     # Confiança do plano


class WeldingJointDetector:
    """
    Detector de juntas de soldagem usando visão computacional
    """
    
    def __init__(self):
        """Inicializa o detector com parâmetros padrão"""
        self.min_contour_area = 100
        self.max_contour_area = 100000
        self.canny_threshold1 = 50
        self.canny_threshold2 = 150
        
    def detect_joint(self, image: np.ndarray) -> Optional[JointInfo]:
        """
        Detecta uma junta de solda na imagem
        
        Args:
            image: Imagem em formato BGR (OpenCV)
            
        Returns:
            JointInfo com os dados da junta detectada, ou None se não detectada
        """
        if image is None:
            return None
            
        # Converte para escala de cinza
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        
        # Aplica desfoque para reduzir ruído
        blurred = cv2.GaussianBlur(gray, (5, 5), 1.5)
        
        # Detecta bordas com Canny
        edges = cv2.Canny(blurred, self.canny_threshold1, self.canny_threshold2)
        
        # Encontra contornos
        contours, _ = cv2.findContours(edges, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        
        if not contours:
            return None
        
        # Filtra e seleciona o maior contorno válido
        valid_contours = [
            c for c in contours
            if self.min_contour_area < cv2.contourArea(c) < self.max_contour_area
        ]
        
        if not valid_contours:
            return None
        
        # Seleciona o contorno com maior área
        joint_contour = max(valid_contours, key=cv2.contourArea)
        
        # Calcula informações da junta
        return self._extract_joint_features(joint_contour, image)
    
    def _extract_joint_features(self, contour: np.ndarray, image: np.ndarray) -> JointInfo:
        """
        Extrai características da junta a partir do contorno
        
        Args:
            contour: Contorno do OpenCV
            image: Imagem original para contexto de dimensões
            
        Returns:
            JointInfo com as características extraídas
        """
        area = cv2.contourArea(contour)
        perimeter = cv2.arcLength(contour, True)
        
        # Calcula o centroide
        M = cv2.moments(contour)
        if M["m00"] == 0:
            cx, cy = 0, 0
        else:
            cx = int(M["m10"] / M["m00"])
            cy = int(M["m01"] / M["m00"])
        
        # Ajusta círculo mínimo para estimar dimensões
        epsilon = 0.02 * perimeter
        approx = cv2.approxPolyDP(contour, epsilon, True)
        
        # Calcula o retângulo envolvente rotacionado
        rect = cv2.minAreaRect(contour)
        (cx, cy), (width, height), angle = rect
        
        # Calcula profundidade estimada (baseada na área e largura)
        estimated_depth = area / width if width > 0 else 0
        
        # Determina se a junta é aberta (V-shaped)
        is_open = self._detect_open_joint(contour, width, height)
        
        # Calcula confiança (baseada na circularidade do contorno)
        confidence = self._calculate_detection_confidence(area, perimeter)
        
        return JointInfo(
            contour=contour,
            area=area,
            perimeter=perimeter,
            center=(cx, cy),
            width=width,
            depth=estimated_depth,
            angle=angle,
            is_open=is_open,
            confidence=confidence
        )
    
    def _detect_open_joint(self, contour: np.ndarray, width: float, height: float) -> bool:
        """
        Detecta se a junta é uma V-shaped (aberta) ou I-shaped (fechada)
        
        Args:
            contour: Contorno da junta
            width: Largura da junta
            height: Altura da junta
            
        Returns:
            True se for junta aberta (V-shaped)
        """
        # Razão de aspecto
        aspect_ratio = height / width if width > 0 else 0
        
        # V-shaped tem altura > largura
        # I-shaped tem altura ≈ largura
        return aspect_ratio > 1.2
    
    def _calculate_detection_confidence(self, area: float, perimeter: float) -> float:
        """
        Calcula a confiança da detecção baseada em características do contorno
        
        Args:
            area: Área do contorno
            perimeter: Perímetro do contorno
            
        Returns:
            Confiança entre 0 e 1
        """
        # Calcula a circularidade (quanto mais próximo de um círculo, maior)
        # Circulatura ideal = 4π * area / perimeter²
        if perimeter == 0:
            return 0.0
        
        circularity = 4 * np.pi * area / (perimeter ** 2)
        
        # Limita entre 0 e 1
        confidence = min(1.0, circularity * 1.5)
        
        return confidence


class WeldingClassifier:
    """
    Classifica tipos de passes e seleciona processos apropriados
    """
    
    def __init__(self):
        """Inicializa o classificador"""
        self.pass_sequence = [PassType.ROOT, PassType.FILL, PassType.CAP]
        
        # Biblioteca de parâmetros de soldagem
        self.welding_parameters = self._load_welding_parameters()
    
    def classify_pass(self, joint_info: JointInfo, pass_number: int = 1) -> PassType:
        """
        Classifica qual tipo de passe é necessário
        
        Args:
            joint_info: Informações da junta
            pass_number: Número do passe (começa em 1)
            
        Returns:
            Tipo de passe recomendado
        """
        if pass_number < len(self.pass_sequence):
            return self.pass_sequence[pass_number - 1]
        else:
            return PassType.CAP
    
    def select_process(self, pass_type: PassType, joint_info: JointInfo, 
                      material: str = "steel") -> WeldProcess:
        """
        Seleciona o processo de soldagem mais apropriado
        
        Args:
            pass_type: Tipo de passe
            joint_info: Informações da junta
            material: Tipo de material
            
        Returns:
            Processo de soldagem recomendado
        """
        if pass_type == PassType.ROOT:
            # Passe raiz requer alta qualidade - prefere GTAW ou SMAW
            return WeldProcess.GTAW if joint_info.is_open else WeldProcess.GTAW
        
        elif pass_type == PassType.FILL:
            # Enchimento requer produtividade - prefere GMAW/FCAW
            if joint_info.depth > 10:
                return WeldProcess.FCAW  # Mais produtivo para juntas profundas
            return WeldProcess.GMAW
        
        else:  # CAP
            # Acabamento requer boa aparência - prefere GMAW
            return WeldProcess.GMAW
    
    def plan_welding(self, joint_info: JointInfo, pass_number: int = 1,
                     material: str = "steel") -> WeldingPlan:
        """
        Cria um plano de soldagem completo para a junta
        
        Args:
            joint_info: Informações da junta
            pass_number: Número do passe
            material: Tipo de material
            
        Returns:
            Plano de soldagem com todos os parâmetros
        """
        pass_type = self.classify_pass(joint_info, pass_number)
        process = self.select_process(pass_type, joint_info, material)
        
        # Calcula número de passes necessários
        num_passes = self._calculate_num_passes(joint_info)
        
        # Obtém parâmetros de soldagem
        parameters = self.welding_parameters.get(
            (pass_type.value, process.value),
            {}
        )
        
        # Calcula geometria da tocha
        torch_angle = self._calculate_torch_angle(joint_info, pass_type)
        
        # Calcula pontos de soldagem (simplificado - apenas 2D)
        cx, cy = joint_info.center
        start_point = (cx - joint_info.width/2, cy, 0)
        end_point = (cx + joint_info.width/2, cy, 0)
        
        # Velocidade de deslocamento
        travel_speed = parameters.get('travel_speed', 300)
        
        return WeldingPlan(
            pass_type=pass_type,
            process=process,
            num_passes=num_passes,
            parameters=parameters,
            torch_angle=torch_angle,
            start_point=start_point,
            end_point=end_point,
            travel_speed=travel_speed,
            confidence=joint_info.confidence
        )
    
    def _calculate_num_passes(self, joint_info: JointInfo) -> int:
        """
        Calcula número de passes necessários baseado na profundidade
        
        Args:
            joint_info: Informações da junta
            
        Returns:
            Número de passes recomendado
        """
        depth = joint_info.depth
        
        if depth < 5:
            return 1  # Um passe é suficiente
        elif depth < 15:
            return 2  # Dois passes
        else:
            return max(3, int(depth / 8))  # Mais passes para juntas profundas
    
    def _calculate_torch_angle(self, joint_info: JointInfo, pass_type: PassType) -> float:
        """
        Calcula o ângulo ideal da tocha
        
        Args:
            joint_info: Informações da junta
            pass_type: Tipo de passe
            
        Returns:
            Ângulo em graus (0-180)
        """
        # Ângulo base da junta
        base_angle = joint_info.angle
        
        if pass_type == PassType.ROOT:
            # Passe raiz requer ângulo mais perpendicular
            return base_angle + 90
        else:
            # Enchimento e acabamento com ângulo tradicional
            return base_angle + 45
    
    def _load_welding_parameters(self) -> dict:
        """
        Carrega a biblioteca de parâmetros de soldagem
        
        Returns:
            Dicionário com parâmetros por (pass_type, process)
        """
        return {
            # Passe Raiz
            ('root', 'SMAW'): {
                'voltage': 20,
                'current': 80,
                'travel_speed': 150,
                'wire_feed_speed': 0,  # Eletrodo revestido
                'gas_flow': 0
            },
            ('root', 'GTAW'): {
                'voltage': 15,
                'current': 120,
                'travel_speed': 100,
                'wire_feed_speed': 80,
                'gas_flow': 15  # Argônio
            },
            
            # Passe de Enchimento
            ('fill', 'GMAW'): {
                'voltage': 22,
                'current': 200,
                'travel_speed': 300,
                'wire_feed_speed': 400,
                'gas_flow': 20  # CO2/Argônio mix
            },
            ('fill', 'FCAW'): {
                'voltage': 24,
                'current': 220,
                'travel_speed': 350,
                'wire_feed_speed': 450,
                'gas_flow': 0  # FCAW não requer gás (self-shielded)
            },
            
            # Passe de Acabamento
            ('cap', 'GMAW'): {
                'voltage': 20,
                'current': 180,
                'travel_speed': 250,
                'wire_feed_speed': 350,
                'gas_flow': 20
            }
        }


def visualize_joint_detection(image: np.ndarray, joint_info: Optional[JointInfo]) -> np.ndarray:
    """
    Visualiza a detecção da junta na imagem
    
    Args:
        image: Imagem original
        joint_info: Informações da junta detectada
        
    Returns:
        Imagem com anotações
    """
    vis_image = image.copy()
    
    if joint_info is None:
        return vis_image
    
    # Desenha contorno
    cv2.drawContours(vis_image, [joint_info.contour], 0, (0, 255, 0), 2)
    
    # Desenha centroide
    cv2.circle(vis_image, (int(joint_info.center[0]), int(joint_info.center[1])), 
               5, (0, 0, 255), -1)
    
    # Adiciona text com informações
    text = f"Area: {joint_info.area:.0f} | Conf: {joint_info.confidence:.2f}"
    cv2.putText(vis_image, text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 
                0.7, (255, 255, 255), 2)
    
    return vis_image
