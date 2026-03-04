#!/usr/bin/env python3
"""
Exemplo de uso do módulo de detecção sem ROS
Demonstra como usar as classes diretamente em Python
"""

import cv2
import numpy as np
from pathlib import Path
import sys

# Adiciona path para importar o módulo
sys.path.insert(0, str(Path(__file__).parent.parent / 'src'))

from joint_detector import (
    WeldingJointDetector, 
    WeldingClassifier,
    visualize_joint_detection
)


def example_1_detect_from_file():
    """
    Exemplo 1: Detectar junta de um arquivo de imagem
    """
    print("\n" + "="*60)
    print("EXEMPLO 1: Detectar Junta de um Arquivo")
    print("="*60)
    
    # Para este exemplo, criamos uma imagem simulada
    # Em uso real, você carregaria: img = cv2.imread('sua_junta.jpg')
    
    print("\n[1] Criando imagem de teste com junta simulada...")
    image = np.ones((480, 640, 3), dtype=np.uint8) * 220  # Fundo cinza
    
    # Desenha uma junta em V (como seria real)
    pts = np.array([
        [200, 150],   # Topo esquerdo
        [320, 350],   # Fundo (apex)
        [440, 150]    # Topo direito
    ], np.int32)
    
    cv2.polylines(image, [pts], False, (50, 50, 50), 4)  # Preta
    
    # Adiciona detalhe
    cv2.line(image, (200, 150), (320, 200), (80, 80, 80), 2)
    
    # Simula dano superficial (ruído)
    noise = np.random.normal(0, 5, image.shape).astype(np.uint8)
    image = cv2.add(image, noise)
    
    print("  ✓ Imagem criada (640x480)")
    
    # Detecção
    print("\n[2] Iniciando detecção...")
    detector = WeldingJointDetector()
    joint_info = detector.detect_joint(image)
    
    if joint_info:
        print("  ✓ Junta detectada com sucesso!")
        print(f"\n    Características da Junta:")
        print(f"      • Área: {joint_info.area:.1f} pixels²")
        print(f"      • Perímetro: {joint_info.perimeter:.1f} pixels")
        print(f"      • Centro: ({joint_info.center[0]:.1f}, {joint_info.center[1]:.1f})")
        print(f"      • Largura: {joint_info.width:.1f} px")
        print(f"      • Profundidade: {joint_info.depth:.1f} px")
        print(f"      • Ângulo: {joint_info.angle:.1f}°")
        print(f"      • Junta Aberta: {'Sim (V-shape)' if joint_info.is_open else 'Não (I-shape)'}")
        print(f"      • Confiança: {joint_info.confidence:.2%}")
    else:
        print("  ✗ Nenhuma junta detectada")
        return None
    
    return image, joint_info


def example_2_classify_welding():
    """
    Exemplo 2: Classificar tipo de passe e processo
    """
    print("\n" + "="*60)
    print("EXEMPLO 2: Classificar Passe e Processo")
    print("="*60)
    
    # Usa a junta do exemplo 1
    image, joint_info = example_1_detect_from_file()
    
    if joint_info is None:
        return
    
    print("\n[1] Inicializando classificador...")
    classifier = WeldingClassifier()
    print("  ✓ Classificador pronto")
    
    # Teste com diferentes números de passa
    print("\n[2] Classificando tipos de passe...")
    
    from joint_detector import PassType
    
    for pass_num in range(1, 4):
        pass_type = classifier.classify_pass(joint_info, pass_number=pass_num)
        process = classifier.select_process(pass_type, joint_info, material="steel")
        
        print(f"\n    Passe #{pass_num}:")
        print(f"      • Tipo: {pass_type.value.upper()}")
        print(f"      • Processo: {process.value}")
        
        # Mostra parâmetros
        params_key = (pass_type.value, process.value)
        params = classifier.welding_parameters.get(params_key, {})
        
        if params:
            print(f"      • Parâmetros:")
            print(f"          - Tensão: {params.get('voltage', 'N/A')} V")
            print(f"          - Corrente: {params.get('current', 'N/A')} A")
            print(f"          - Vel. Avanço: {params.get('travel_speed', 'N/A')} mm/min")


def example_3_complete_welding_plan():
    """
    Exemplo 3: Gerar plano de soldagem completo
    """
    print("\n" + "="*60)
    print("EXEMPLO 3: Plano de Soldagem Completo")
    print("="*60)
    
    # Usa a junta do exemplo 1
    image, joint_info = example_1_detect_from_file()
    
    if joint_info is None:
        return
    
    print("\n[1] Gerando plano de soldagem...")
    classifier = WeldingClassifier()
    plan = classifier.plan_welding(joint_info, pass_number=1, material="steel")
    
    print("  ✓ Plano gerado")
    
    print(f"\n[2] Detalhes do Plano:")
    print(f"\n    IDENTIFICAÇÃO:")
    print(f"      • Tipo de Passe: {plan.pass_type.value.upper()}")
    print(f"      • Processo: {plan.process.value}")
    print(f"      • Material: steel")
    print(f"      • Confiança: {plan.confidence:.2%}")
    
    print(f"\n    GEOMETRIA:")
    print(f"      • Ponto de Início: {plan.start_point}")
    print(f"      • Ponto de Fim: {plan.end_point}")
    print(f"      • Ângulo da Tocha: {plan.torch_angle:.1f}°")
    
    print(f"\n    PLANEJAMENTO DE PASSES:")
    print(f"      • Passes Necessários: {plan.num_passes}")
    print(f"      • Velocidade de Deslocamento: {plan.travel_speed} mm/min")
    
    print(f"\n    PARÂMETROS DE SOLDAGEM:")
    print(f"      • Tensão: {plan.parameters.get('voltage', 'N/A')} V")
    print(f"      • Corrente: {plan.parameters.get('current', 'N/A')} A")
    print(f"      • Vel. Avanço: {plan.parameters.get('travel_speed', 'N/A')} mm/min")
    print(f"      • Vel. Arame: {plan.parameters.get('wire_feed_speed', 'N/A')} mm/min")
    print(f"      • Vazão Gás: {plan.parameters.get('gas_flow', 'N/A')} L/min")
    
    return plan


def example_4_visualization():
    """
    Exemplo 4: Visualizar detecção na imagem
    """
    print("\n" + "="*60)
    print("EXEMPLO 4: Visualizar Detecção")
    print("="*60)
    
    image, joint_info = example_1_detect_from_file()
    
    if joint_info is None:
        return
    
    print("\n[1] Criando imagem com anotações...")
    vis_image = visualize_joint_detection(image, joint_info)
    print("  ✓ Imagem com anotações criada")
    
    # Salva a imagem
    output_file = "/tmp/welding_detection_result.jpg"
    cv2.imwrite(output_file, vis_image)
    print(f"\n[2] Imagem salva em: {output_file}")
    
    # Se em ambiente gráfico, tenta mostrar
    try:
        cv2.imshow("Detecção de Junta", vis_image)
        print("\n[3] Pressione qualquer tecla para continuar...")
        cv2.waitKey(0)
        cv2.destroyAllWindows()
    except Exception as e:
        print(f"\n[3] Não foi possível exibir imagem (modo headless): {e}")


def example_5_batch_processing():
    """
    Exemplo 5: Processar múltiplas juntas em batch
    """
    print("\n" + "="*60)
    print("EXEMPLO 5: Processamento em Batch")
    print("="*60)
    
    print("\n[1] Gerando 5 juntas de teste...")
    
    detector = WeldingClassifier()
    results = []
    
    for i in range(1, 6):
        # Cria junta com profundidade variada
        image = np.ones((480, 640, 3), dtype=np.uint8) * 220
        
        depth = i * 8  # 8, 16, 24, 32, 40 mm
        
        # Desenha junta com profundidade proporcional
        height = int(100 + depth * 2)
        pts = np.array([
            [200, 150],
            [320, 150 + height],
            [440, 150]
        ], np.int32)
        
        cv2.polylines(image, [pts], False, (50, 50, 50), 4)
        
        # Detecta
        detector_obj = WeldingJointDetector()
        joint = detector_obj.detect_joint(image)
        
        if joint:
            # Planeja
            plan = detector.plan_welding(joint)
            results.append({
                'index': i,
                'depth': joint.depth,
                'process': plan.process.value,
                'num_passes': plan.num_passes
            })
    
    # Exibe resultados
    print(f"\n[2] Resultados:")
    print(f"\n{'ID':<5} {'Profundidade':<15} {'Processo':<10} {'Passes':<8}")
    print("-" * 40)
    
    for result in results:
        print(f"{result['index']:<5} {result['depth']:<15.1f} {result['process']:<10} {result['num_passes']:<8}")
    
    print(f"\n  ✓ Processados {len(results)} juntas com sucesso")


def main():
    """
    Main: Executa todos os exemplos
    """
    print("\n")
    print("╔════════════════════════════════════════════════════════╗")
    print("║  EXEMPLOS DE USO DO MÓDULO DE DETECÇÃO DE SOLDAGEM    ║")
    print("╚════════════════════════════════════════════════════════╝")
    
    try:
        # Exemplo 1
        example_1_detect_from_file()
        
        # Exemplo 2
        example_2_classify_welding()
        
        # Exemplo 3
        example_3_complete_welding_plan()
        
        # Exemplo 4
        example_4_visualization()
        
        # Exemplo 5
        example_5_batch_processing()
        
        print("\n" + "="*60)
        print("✓ TODOS OS EXEMPLOS EXECUTADOS COM SUCESSO")
        print("="*60 + "\n")
        
    except KeyboardInterrupt:
        print("\n\n✗ Interrompido pelo usuário")
    except Exception as e:
        print(f"\n✗ Erro: {e}")
        import traceback
        traceback.print_exc()


if __name__ == '__main__':
    main()
