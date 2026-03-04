"""
Testes unitários para o módulo de detecção de juntas
"""

import unittest
import numpy as np
import cv2
import sys
import os

# Adiciona path para importar o módulo
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'src'))

from joint_detector import (
    WeldingJointDetector, WeldingClassifier, PassType, WeldProcess
)


class TestWeldingJointDetector(unittest.TestCase):
    """Testes para o detectador de juntas"""
    
    def setUp(self):
        """Configuração pré-teste"""
        self.detector = WeldingJointDetector()
    
    def test_detect_joint_valid_image(self):
        """Testa detecção em imagem válida"""
        # Cria imagem com contorno visível
        image = np.ones((480, 640, 3), dtype=np.uint8) * 200
        
        # Desenha uma linha preta (possível junta)
        cv2.line(image, (100, 100), (500, 100), (0, 0, 0), 3)
        cv2.line(image, (100, 100), (300, 300), (0, 0, 0), 3)
        cv2.line(image, (500, 100), (300, 300), (0, 0, 0), 3)
        
        result = self.detector.detect_joint(image)
        
        # Se detectado, deve retornar JointInfo
        if result is not None:
            self.assertTrue(hasattr(result, 'area'))
            self.assertTrue(hasattr(result, 'contour'))
            self.assertGreater(result.confidence, 0)
    
    def test_detect_joint_empty_image(self):
        """Testa detecção em imagem vazia"""
        image = np.ones((480, 640, 3), dtype=np.uint8) * 200
        
        result = self.detector.detect_joint(image)
        
        # Deve retornar None para imagem sem juntas
        self.assertIsNone(result)
    
    def test_detect_joint_none_input(self):
        """Testa detecção com entrada None"""
        result = self.detector.detect_joint(None)
        self.assertIsNone(result)
    
    def test_confidence_calculation(self):
        """Testa cálculo de confiança"""
        image = np.ones((480, 640, 3), dtype=np.uint8) * 200
        
        # Desenha um círculo (forma regular)
        cv2.circle(image, (320, 240), 100, (0, 0, 0), -1)
        
        result = self.detector.detect_joint(image)
        
        if result is not None:
            self.assertGreaterEqual(result.confidence, 0)
            self.assertLessEqual(result.confidence, 1)


class TestWeldingClassifier(unittest.TestCase):
    """Testes para o classificador de processo"""
    
    def setUp(self):
        """Configuração pré-teste"""
        self.classifier = WeldingClassifier()
    
    def test_classify_pass_root(self):
        """Testa classificação de passe raiz"""
        pass_type = self.classifier.classify_pass(None, pass_number=1)
        self.assertEqual(pass_type, PassType.ROOT)
    
    def test_classify_pass_fill(self):
        """Testa classificação de passe de enchimento"""
        pass_type = self.classifier.classify_pass(None, pass_number=2)
        self.assertEqual(pass_type, PassType.FILL)
    
    def test_classify_pass_cap(self):
        """Testa classificação de passe de acabamento"""
        pass_type = self.classifier.classify_pass(None, pass_number=3)
        self.assertEqual(pass_type, PassType.CAP)
    
    def test_classify_pass_beyond_sequence(self):
        """Testa classificação além da sequência"""
        pass_type = self.classifier.classify_pass(None, pass_number=100)
        # Deve retornar CAP para números muito altos
        self.assertEqual(pass_type, PassType.CAP)
    
    def test_select_process_root_open_joint(self):
        """Testa seleção de processo para passe raiz em junta aberta"""
        from joint_detector import JointInfo
        
        # Cria info de junta aberta
        joint_info = JointInfo(
            contour=None,
            area=500,
            perimeter=100,
            center=(0, 0),
            width=50,
            depth=10,
            angle=90,
            is_open=True,
            confidence=0.8
        )
        
        process = self.classifier.select_process(PassType.ROOT, joint_info)
        
        # Passe raiz em junta aberta deve usar GTAW (alta qualidade)
        self.assertIn(process, [WeldProcess.GTAW, WeldProcess.SMAW])
    
    def test_select_process_fill(self):
        """Testa seleção de processo para passe de enchimento"""
        from joint_detector import JointInfo
        
        joint_info = JointInfo(
            contour=None,
            area=500,
            perimeter=100,
            center=(0, 0),
            width=50,
            depth=15,
            angle=90,
            is_open=True,
            confidence=0.8
        )
        
        process = self.classifier.select_process(PassType.FILL, joint_info)
        
        # Enchimento deve preferir FCAW para juntas profundas
        self.assertEqual(process, WeldProcess.FCAW)
    
    def test_num_passes_calculation(self):
        """Testa cálculo do número de passes"""
        from joint_detector import JointInfo
        
        # Junta rasa
        shallow_joint = JointInfo(
            contour=None, area=500, perimeter=100, center=(0, 0),
            width=50, depth=3, angle=90, is_open=False, confidence=0.8
        )
        
        passes = self.classifier._calculate_num_passes(shallow_joint)
        self.assertEqual(passes, 1)
        
        # Junta profunda
        deep_joint = JointInfo(
            contour=None, area=500, perimeter=100, center=(0, 0),
            width=50, depth=25, angle=90, is_open=False, confidence=0.8
        )
        
        passes = self.classifier._calculate_num_passes(deep_joint)
        self.assertGreaterEqual(passes, 3)


class TestWeldingParameterRanges(unittest.TestCase):
    """Testes para os parâmetros de soldagem"""
    
    def setUp(self):
        """Configuração pré-teste"""
        self.classifier = WeldingClassifier()
    
    def test_parameters_loaded(self):
        """Testa se padrões foram carregados"""
        self.assertIsNotNone(self.classifier.welding_parameters)
        self.assertGreater(len(self.classifier.welding_parameters), 0)
    
    def test_voltage_in_range(self):
        """Testa se tensão está em faixa válida"""
        for process_params in self.classifier.welding_parameters.values():
            voltage = process_params.get('voltage', 0)
            self.assertGreater(voltage, 0)
            self.assertLess(voltage, 50)  # Soldagem típica < 50V
    
    def test_current_in_range(self):
        """Testa se corrente está em faixa válida"""
        for process_params in self.classifier.welding_parameters.values():
            current = process_params.get('current', 0)
            self.assertGreater(current, 0)
            self.assertLess(current, 400)  # Soldagem típica < 400A


class TestWeldingPlan(unittest.TestCase):
    """Testes para geração de planos de soldagem"""
    
    def setUp(self):
        """Configuração pré-teste"""
        from joint_detector import JointInfo
        
        self.classifier = WeldingClassifier()
        self.joint_info = JointInfo(
            contour=None,
            area=5000,
            perimeter=300,
            center=(320, 240),
            width=100,
            depth=12,
            angle=45,
            is_open=True,
            confidence=0.9
        )
    
    def test_plan_generation(self):
        """Testa geração de plano completo"""
        plan = self.classifier.plan_welding(self.joint_info, pass_number=1)
        
        self.assertIsNotNone(plan)
        self.assertIsNotNone(plan.pass_type)
        self.assertIsNotNone(plan.process)
        self.assertGreater(plan.num_passes, 0)
        self.assertTrue(hasattr(plan, 'parameters'))
        self.assertTrue(hasattr(plan, 'torch_angle'))
    
    def test_plan_torch_angle(self):
        """Testa cálculo do ângulo da tocha"""
        plan = self.classifier.plan_welding(self.joint_info, pass_number=1)
        
        self.assertGreaterEqual(plan.torch_angle, 0)
        self.assertLessEqual(plan.torch_angle, 180)


def run_tests():
    """Executa todos os testes"""
    # Cria suite de testes
    loader = unittest.TestLoader()
    suite = unittest.TestSuite()
    
    # Adiciona testes
    suite.addTests(loader.loadTestsFromTestCase(TestWeldingJointDetector))
    suite.addTests(loader.loadTestsFromTestCase(TestWeldingClassifier))
    suite.addTests(loader.loadTestsFromTestCase(TestWeldingParameterRanges))
    suite.addTests(loader.loadTestsFromTestCase(TestWeldingPlan))
    
    # Executa
    runner = unittest.TextTestRunner(verbosity=2)
    result = runner.run(suite)
    
    return result.wasSuccessful()


if __name__ == '__main__':
    success = run_tests()
    sys.exit(0 if success else 1)
