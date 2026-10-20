# Kart-VCU — Fórmula Tesla UFMG

Firmware da Unidade de Controle Veicular (VCU) para kart elétrico com tração traseira independente (2x motores BLDC/PMSM). Desenvolvido em C99 para a equipe Fórmula Tesla UFMG.

## Funcionalidades

- **Diferencial Eletrônico**: cinemática de Ackermann para distribuição de rotação entre as rodas traseiras em curvas.
- **Vetorização de Torque (TV)**: controle em malha fechada (PD com filtro IIR de 1ª ordem) para rastreamento de taxa de guinada e compensação de subesterço.
- **Launch Control**: controle de arrancada com estimativa de transferência dinâmica de carga traseira e malha PI para manter o escorregamento próximo ao pico de aderência (~12%).
- **Segurança Funcional**:
  - Validação redundante de curso de acelerador (APPS1/APPS2 com corte em discrepância > 10% por 100 ms).
  - Plausibilidade de freio e acelerador (BSPD: freio > 15 bar e pedal > 25%, com travamento até redução < 5%).
  - Monitoramento de heartbeat de barramento CAN com timeout de 100 ms.
- **Gestão Térmica e de Bateria**:
  - Derating térmico de motores (95 a 125 °C) e inversores (65 a 85 °C).
  - Derating por subtensão da bateria (64 a 58 V).
  - Limitador de potência elétrica total (13.6 kW) e corrente de descarga (220 A).
  - Frenagem regenerativa proporcional integrada ao pedal de freio com proteção anti-bloqueio.

## Parâmetros Principais

| Parâmetro | Valor | Unidade |
| :--- | :--- | :--- |
| Massa total (kart + piloto) | 250 | kg |
| Distância entre eixos (L) | 0.955 | m |
| Bitola traseira (B) | 1.100 | m |
| Relação de transmissão (59T/16T) | 3.6875 | - |
| Raio dinâmico do pneu traseiro | 0.132 | m |
| Potência de pico combinada | 13.6 | kW |
| Torque máximo por motor | 25.0 | N.m |
| Rotação máxima dos motores | 6000 | RPM |
| Velocidade máxima | 80 | km/h |
| Tensão nominal do barramento | 72 | V |

## Estrutura do Código

- `vehicle_control.h` / `vehicle_control.c`: lógica central, segurança funcional (APPS/BSPD), estimador inercial de velocidade, derating térmico/subtensão e regeneração.
- `electronic_differential.h` / `electronic_differential.c`: diferencial de Ackermann e torque vectoring.
- `launch_control.h` / `launch_control.c`: máquina de estados de largada e controle PI de slip ratio.
- `main_simulation.c`: bancada de testes em malha fechada e simulação Monte Carlo (1000 iterações estocásticas).

## Compilação e Execução

Requisitos: compilador C99 (GCC ou Clang).

Compilar:
```bash
gcc -Wall -Wextra -Werror -pedantic -std=c99 -O2 \
    main_simulation.c vehicle_control.c electronic_differential.c launch_control.c \
    -lm -o sim_teste
```

Executar (semente aleatória pelo relógio):
```bash
./sim_teste
```

Executar com semente fixa para reprodutibilidade:
```bash
./sim_teste 14707
```

## Autores

- João Paulo Silva Plöger
- Fórmula Tesla UFMG — Universidade Federal de Minas Gerais
