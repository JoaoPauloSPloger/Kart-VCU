# Documento de Referência Técnica (`reference.md`)

---

## 1. Identificação e Descrição do Projeto

* **Nome Oficial do Projeto:** Kart Elétrico de Alta Performance — Projeto Trainee de Engenharia 2026/2 (Fórmula Tesla UFMG)
* **Subsistema Principal:** Tração, Potência, Transmissão Mecânica e Controle Dinâmico Veicular (VCU)
* **Instituição:** Universidade Federal de Minas Gerais (UFMG) — Escola de Engenharia
* **Equipe:** Fórmula Tesla UFMG
* **Autores do Subsistema de Powertrain & VCU:** João Paulo Silva Plöger
* **Descrição Geral:**
  Desenvolvimento integral do *powertrain* elétrico traseiro bipartido, sistema de transmissão mecânica por corrente e *firmware* embarcado da Unidade de Controle Veicular (**VCU**) para um superkart elétrico projetado para substituir frotas a combustão em kartódromos. O sistema integra dois motores BLDC/PMSM independentes no eixo traseiro, permitindo **Diferencial Eletrônico por Software (Cinemática de Ackermann)**, **Vetorização Ativa de Torque (*Torque Vectoring* PD com filtro IIR)**, **Controle de Largada (*Launch Control* PI por *slip ratio* com transferência dinâmica de carga)**, **Frenagem Regenerativa Proporcional (*Brake Blending*)** e supervisão de segurança funcional crítica em conformidade com as normas **ISO 26262**, **SAE J670 / ISO 8855**, **FSAE Electric Vehicle** e **MISRA-C:2012**.

---

## 2. Equipe e Divisão de Subsistemas

| Integrante(s) | Subsistema / Responsabilidade Principal |
| :--- | :--- |
| **João Paulo Silva Plöger** | Tração, Potência, Transmissão Mecânica, Sensores, Diferencial Eletrônico, *Launch Control*, Frenagem Regenerativa e Dinâmica/Pneus |
| **Tomás & Miguel** | Sistema de Freios Hidráulicos, Dimensionamento de Cilindro Mestre/Pinças/Discos e Integração de *Brake Blending* |
| **Gabriel Fúrfuro & Guilherme** | Estrutura Tubular do Chassi, Proteções de Impacto (Frontal, Lateral, Traseira), Suportes de Mancais e Simulação Estrutural FEA (*Ansys*) |
| **Guilherme** | Aerodinâmica, Carenagem em Materiais Compósitos, Dutos de Refrigeração NACA e Mapeamento de Fluxo |

---

## 3. Constantes Físicas e Parâmetros Geométricos / Dinâmicos do Veículo

### 3.1. Constantes Físicas Universais (Imutáveis nos Ensaios)
| Constante | Macro no Código | Símbolo | Valor | Unidade |
| :--- | :--- | :---: | :---: | :---: |
| Aceleração da gravidade | `GRAVITY_MSS` | $g$ | $9{,}81$ | $\text{m/s}^2$ |
| Massa específica do ar (ISA ao nível do mar) | `AIR_DENSITY_KG_M3` | $\rho_{\text{ar}}$ | $1{,}225$ | $\text{kg/m}^3$ |

### 3.2. Massa, Geometria e Aerodinâmica do Kart
| Parâmetro | Macro no Código | Símbolo | Valor Nominal | Unidade |
| :--- | :--- | :---: | :---: | :---: |
| Massa total máxima regulamentar (kart + piloto + bateria) | `TOTAL_VEHICLE_MASS_KG` | $m$ | $250{,}0$ | $\text{kg}$ |
| Massa e dimensões do pack de baterias | — | $m_{\text{bat}}$ | $40{,}0\text{ kg}$ ($400 \times 200 \times 200\text{ mm}$) | — |
| Distância entre eixos (*Wheelbase*) | `WHEELBASE_M` | $L$ | $0{,}955$ | $\text{m}$ |
| Bitola traseira (*Rear Track*) | `REAR_TRACK_M` | $B_r$ | $1{,}100$ | $\text{m}$ |
| Bitola dianteira (*Front Track*) | `FRONT_TRACK_M` | $B_f$ | $0{,}950$ | $\text{m}$ |
| Altura do centro de gravidade | `CG_HEIGHT_M` | $h_{\text{CG}}$ | $0{,}220$ | $\text{m}$ |
| Distribuição estática de peso (Traseira / Dianteira) | `WEIGHT_DIST_REAR` / `FRONT` | $w_r \ / \ w_f$ | $60\% \ / \ 40\%$ | — |
| Área frontal projetada (CAD kart + piloto) | `FRONTAL_AREA_M2` | $A$ | $0{,}420$ | $\text{m}^2$ |
| Coeficiente de arrasto aerodinâmico | `DRAG_COEFFICIENT_CD` | $C_d$ | $0{,}700$ | — |
| Coeficiente de resistência ao rolamento | `ROLLING_RESISTANCE_CRR` | $C_{rr}$ | $0{,}022$ | — |
| Limite mecânico máximo de esterçamento | `MAX_STEERING_ANGLE_DEG` | $\delta_{\max}$ | $\pm 60{,}0^\circ$ ($\pm 1{,}0472\text{ rad}$) | graus / rad |

### 3.3. Rodas e Pneus (Dianteiros $10 \times 4.50\text{-}5$ / Traseiros $11 \times 7.10\text{-}5$)
| Parâmetro | Macro no Código | Símbolo | Valor Nominal | Unidade |
| :--- | :--- | :---: | :---: | :---: |
| Raio dinâmico do pneu dianteiro sob carga | `FRONT_TIRE_RADIUS_DYNAMIC_M` | $r_{\text{din},f}$ | $0{,}125$ | $\text{m}$ |
| Raio nominal do pneu traseiro descarregado | `TIRE_RADIUS_NOMINAL_M` | $r_{\text{nom},r}$ | $0{,}135$ | $\text{m}$ |
| Raio dinâmico do pneu traseiro sob deformação | `TIRE_RADIUS_DYNAMIC_M` | $r_{\text{din},r}$ | $0{,}132$ | $\text{m}$ |
| Coeficiente de atrito longitudinal de pico (seco) | `TIRE_PEAK_FRICTION_COEFF` | $\mu_{\text{peak}}$ | $1{,}150$ | — |
| Escorregamento longitudinal ótimo de pico | `target_slip` | $\lambda^*$ | $0{,}12$ ($12\%$) | — |
| Inércia rotacional equivalente refletida por roda traseira | `REAR_WHEEL_EQUIV_INERTIA_KGM2` | $J_{\text{eq},w}$ | $0{,}110$ | $\text{kg}\cdot\text{m}^2$ |

---

## 4. Especificações do Powertrain, Transmissão e Sistema Elétrico

### 4.1. Motores e Transmissão Mecânica Independente (Semieixos Bipartidos)
| Parâmetro | Macro no Código | Valor Nominal | Unidade |
| :--- | :--- | :---: | :---: |
| Número de motores de tração traseira | `NUM_MOTORS` | $2$ (independentes, 1 por semieixo) | unid. |
| Relação de redução por corrente (Coroa $59\text{T}$ / Pinhão $16\text{T}$) | `GEAR_RATIO` | $3{,}700$ ($3{,}6875:1$) | — |
| Rendimento mecânico da transmissão por corrente | `TRANSMISSION_EFFICIENCY` | $0{,}950$ ($95\%$) | — |
| Rendimento eletromecânico combinado ($\eta_{\text{inv}} \cdot \eta_{\text{mot}}$) | `INVERTER_MOTOR_EFFICIENCY` | $0{,}880$ ($88\%$) | — |
| Rotação máxima regulamentar dos motores | `MOTOR_MAX_RPM` | $6000{,}0$ | $\text{RPM}$ |
| Velocidade máxima regulamentar do veículo | `MAX_SPEED_KMH` / `MAX_SPEED_MS` | $80{,}0\text{ km/h}$ ($22{,}22\text{ m/s}$) | $\text{km/h}$ / $\text{m/s}$ |
| Potência nominal contínua (por motor / total) | `MOTOR_RATED_POWER_KW` | $2{,}5\text{ kW}$ / $5{,}0\text{ kW}$ | $\text{kW}$ |
| Potência elétrica de pico (por motor / combinada) | `MOTOR_PEAK_POWER_KW` / `SYSTEM_MAX_POWER_KW` | $6{,}8\text{ kW}$ / $13{,}6\text{ kW}$ | $\text{kW}$ |
| Torque máximo de pico (por motor / total de entrada) | `MOTOR_MAX_TORQUE_NM` / `SYSTEM_MAX_TORQUE_NM` | $25{,}0\text{ N}\cdot\text{m}$ / $50{,}0\text{ N}\cdot\text{m}$ | $\text{N}\cdot\text{m}$ |
| Torque máximo trativo combinado no eixo traseiro | — | $175{,}75$ ($50 \times 3{,}70 \times 0{,}95$) | $\text{N}\cdot\text{m}$ |

### 4.2. Barramento DC, Correntes e Limites Térmicos
| Parâmetro | Macro no Código | Valor Nominal | Unidade |
| :--- | :--- | :---: | :---: |
| Tensão nominal do barramento DC ($\text{LiFePO}_4$ / NMC) | `DC_BUS_NOMINAL_VOLTAGE_V` | $72{,}0$ | $\text{V}$ |
| Corrente máxima de descarga da bateria | `BATTERY_MAX_DISCHARGE_A` | $220{,}0$ | $\text{A}$ |
| Corrente máxima de recarga regenerativa | `BATTERY_MAX_REGEN_A` | $60{,}0$ | $\text{A}$ |
| Subtensão: início de *derating* linear $\to$ corte total | `BATTERY_UNDERVOLTAGE_START_V` $\to$ `CUTOFF_V` | $64{,}0\text{ V} \to 58{,}0\text{ V}$ | $\text{V}$ |
| Tensão máxima admissível para regeneração ativa | `BATTERY_MAX_REGEN_VOLTAGE_V` | $82{,}0$ | $\text{V}$ |
| Velocidade mínima do chassi para regeneração | `REGEN_MIN_SPEED_MS` | $1{,}5$ ($5{,}4\text{ km/h}$) | $\text{m/s}$ |
| Temperatura do motor: início de *derating* $\to$ corte (*Cutoff*) | `MOTOR_TEMP_DERATING_START_C` $\to$ `CUTOFF_C` | $95{,}0^\circ\text{C} \to 125{,}0^\circ\text{C}$ | $^\circ\text{C}$ |
| Temperatura do inversor: início de *derating* $\to$ corte (*Cutoff*) | `INVERTER_TEMP_DERATING_START_C` $\to$ `CUTOFF_C` | $65{,}0^\circ\text{C} \to 85{,}0^\circ\text{C}$ | $^\circ\text{C}$ |

---

## 5. Modelagem Matemática e Equacionamento Físico

### 5.1. Dinâmica Longitudinal e Resistências ao Movimento a $80\text{ km/h}$ ($22{,}22\text{ m/s}$)
* **Resistência ao Rolamento:**
  $$F_{\text{roll}} = C_{rr} \cdot m \cdot g = 0{,}022 \times 250 \times 9{,}81 = 53{,}96\text{ N}$$
* **Arrasto Aerodinâmico:**
  $$F_{\text{drag}}(v) = \frac{1}{2} \rho_{\text{ar}} C_d A v^2 \implies F_{\text{drag}}(22{,}22) = 0{,}5 \times 1{,}225 \times 0{,}70 \times 0{,}42 \times (22{,}22)^2 = 88{,}91\text{ N}$$
* **Potência Mecânica e Elétrica de Cruzeiro a $80\text{ km/h}$:**
  $$P_{\text{res}} = (F_{\text{roll}} + F_{\text{drag}}) \cdot v = (53{,}96 + 88{,}91) \times 22{,}22 = 3{,}175\text{ kW} \implies P_{\text{eixo}} = \frac{3{,}175}{0{,}95} = 3{,}34\text{ kW}$$

### 5.2. Cinemática de Ackermann e Controle de Vetorização de Torque (*Torque Vectoring*)
* **Velocidades Angulares Alvo nas Rodas Traseiras** (convenção SAE J670: $\delta > 0$ = curva à direita):
  $$\omega_c = \frac{v_x}{r_{\text{din},r}}, \qquad \omega_{L,\text{alvo}} = \omega_c \left[1 + \frac{B_r}{2L}\tan(\delta)\right], \qquad \omega_{R,\text{alvo}} = \omega_c \left[1 - \frac{B_r}{2L}\tan(\delta)\right]$$
* **Taxa de Guinada Desejada (*Bicycle Model* com gradiente de subesterço $K_{us} = 0{,}0025\text{ rad/(m/s}^2)$):**
  $$\dot{\psi}_{\text{des}} = \left(\frac{v_x}{L + \frac{K_{us} v_x^2}{g}}\right) \tan(\delta), \qquad e_{\psi}[k] = \dot{\psi}_{\text{des}}[k] - \dot{\psi}_{\text{IMU}}[k]$$
* **Controlador PD com Filtro Passa-Baixas IIR de 1ª Ordem ($\tau_d = 20\text{ ms}$, $K_p = 18{,}5$, $K_d = 1{,}2$):**
  $$D[k] = \alpha D[k-1] + (1 - \alpha) K_d \frac{e_{\psi}[k] - e_{\psi}[k-1]}{\Delta t}, \quad \alpha = \frac{\tau_d}{\tau_d + \Delta t}$$
  $$\Delta T = \text{clamp}\!\left(K_p e_{\psi}[k] + D[k],\, -\Delta T_{\max},\, +\Delta T_{\max}\right), \quad T_L = T_{\text{base}} + \Delta T, \quad T_R = T_{\text{base}} - \Delta T$$

### 5.3. *Launch Control*: Transferência de Carga e Controle PI de Escorregamento
* **Força Normal Dinâmica no Eixo Traseiro:**
  $$N_{r,\text{din}} = m \cdot g \cdot w_r + m \cdot a_{x,\text{filt}} \cdot \frac{h_{\text{CG}}}{L}$$
* **Torque de *Feedforward* pelo Limite de Aderência Trativa:**
  $$T_{\text{mot,ff}} = \frac{\mu_{\text{peak}} \cdot N_{r,\text{din}} \cdot r_{\text{din},r}}{2 \cdot i \cdot \eta_{\text{trans}}}$$
* **Razão de Escorregamento Longitudinal (*Slip Ratio* $\lambda$):**
  $$\lambda = \frac{v_{\text{rear}} - v_x}{\max(v_x,\, 1{,}0\text{ m/s})}, \qquad e_{\lambda} = \lambda^* - \lambda \quad (\lambda^* = 0{,}12)$$
* **Lei de Controle PI com *Anti-Windup* Condicional e Rampa de Torque ($120\text{ N}\cdot\text{m/s}$):**
  $$T_{\text{alvo}} = \text{clamp}\!\left(T_{\text{mot,ff}} + K_{p,\lambda} e_{\lambda} + K_{i,\lambda} \int e_{\lambda}\,dt,\, 0,\, T_{\text{piloto}}\right)$$

---

## 6. Segurança Funcional Automotiva e Diagnóstico de Falhas (ISO 26262 / FSAE)

| Flag de Falha (`fault_flags`) | Bitmask | Condição de Disparo | Ação de Segurança (Safe State) |
| :--- | :---: | :--- | :--- |
| `FAULT_NONE` | `0x00` | Operação normal sem anomalias | Tração e regeneração habilitadas |
| `FAULT_APPS_DISCREPANCY` | `1U << 0` (`0x01`) | $\| \text{APPS}_1 - \text{APPS}_2 \| > 10\%$ persistente por $\ge 100\text{ ms}$ | Corte imediato de torque ($0\text{ N}\cdot\text{m}$) e desabilitação dos inversores |
| `FAULT_APPS_OUT_OF_RANGE` | `1U << 1` (`0x02`) | Leitura de $\text{APPS}_1$ ou $\text{APPS}_2 \notin [0\%,\, 100\%]$ | Corte imediato de torque e desabilitação dos inversores |
| `FAULT_BRAKE_OUT_OF_RANGE` | `1U << 2` (`0x04`) | Pressão hidráulica de freio $\notin [0,\, 120\text{ bar}]$ | Corte imediato de torque e desabilitação dos inversores |
| `FAULT_BSPD_TRIPPED` | `1U << 3` (`0x08`) | Freio $> 15\text{ bar}$ simultâneo a $\text{APPS} > 25\%$ (exceto *Staging* $< 3\text{ s}$) | *Latch* de corte de demanda ($0\%$); só rearma quando $\text{APPS} < 5\%$ |
| `FAULT_OVER_TEMP` | `1U << 4` (`0x10`) | $T_{\text{mot}} > 95^\circ\text{C}$ ou $T_{\text{inv}} > 65^\circ\text{C}$ | *Derating* linear de torque; corte total em $125^\circ\text{C}$ (motor) ou $85^\circ\text{C}$ (inversor) |
| `FAULT_UNDERVOLTAGE` | `1U << 5` (`0x20`) | Tensão do barramento DC $V_{\text{batt}} < 64{,}0\text{ V}$ | *Derating* linear de torque; corte total em $V_{\text{batt}} \le 58{,}0\text{ V}$ |
| `FAULT_SENSOR_NAN` | `1U << 6` (`0x40`) | Valor `NaN`/`Inf`, $\Delta t \le 0$ ou leitura fora de faixa física nos sensores | Estado Seguro imediato ($0\text{ N}\cdot\text{m}$, inversores desabilitados) |
| `FAULT_CAN_TIMEOUT` | `1U << 7` (`0x80`) | Contador de *heartbeat* CAN congelado por $\ge 100\text{ ms}$ | Corte imediato de torque e desabilitação dos inversores |

---

## 7. Arquitetura de Software e Interfaces da VCU

### 7.1. Estruturas de Dados Principais
* **[`VehicleSafetyContext_t`](vehicle_control.h)**: Contexto reentrante de segurança funcional, temporizadores de implausibilidade APPS, *latch* do BSPD, *bitmask* `fault_flags`, estado do estimador $v_x$, flag `wheel_encoders_active` e *watchdog* CAN.
* **[`DriverInputs_t`](vehicle_control.h)**: Sinais dos pedais (`apps1_percent`, `apps2_percent`, `raw_pedal_demand`, `throttle_demand`), pressão e chave de freio (`brake_pressure_bar`, `brake_switch_active`), esterçamento (`steering_angle_deg`, `steering_angle_rad`), botão de *Launch Control* e flag `safety_plausibility_ok`.
* **[`SensorData_t`](vehicle_control.h)**: Velocidades angulares das 4 rodas (`wheel_speed_fl/fr/rl/rr_rads`), velocidade longitudinal do chassi (`vehicle_speed_ms`), IMU (`yaw_rate_rads`, `ax_mss`, `ay_mss`), temperaturas dos motores e inversores, tensão/corrente de bateria e `can_heartbeat_counter`.
* **[`MotorCommands_t`](vehicle_control.h)**: Torques finais arbitrados (`torque_left_nm`, `torque_right_nm`), rotações de referência (`target_rpm_left`, `target_rpm_right`), habilitação dos inversores (`inverter_left_enable`, `inverter_right_enable`) e status `regen_active`.
* **[`ElectronicDiffConfig_t`](electronic_differential.h)**: Ganhos $K_p, K_d$, coeficiente de subesterço $K_{us}$, limite $\Delta T_{\max}$ e estados internos do filtro derivativo IIR.
* **[`LaunchControlContext_t`](launch_control.h)**: Estado atual da máquina (`LC_STATE_DISARMED`, `LC_STATE_ARMED`, `LC_STATE_STAGING`, `LC_STATE_LAUNCHING`, `LC_STATE_COMPLETED`, `LC_STATE_ABORTED`), ganhos PI, integrador com *anti-windup*, temporizadores de *Staging* ($3{,}0\text{ s}$) e largada ($4{,}0\text{ s}$), filtro EMA de $a_x$ e fator de *blending* ($500\text{ ms}$).

### 7.2. Pipeline de Execução Cíclica ([`VehicleControl_Step`](vehicle_control.c))
1. [`VehicleControl_ValidateInputs`](vehicle_control.c): Valida finitude, sincroniza/valida direção, verifica faixas elétricas de APPS/freio, temporiza discrepância APPS ($100\text{ ms}$) e executa lógica do BSPD.
2. [`VehicleControl_ValidateSensors`](vehicle_control.c): Verifica finitude e limites físicos de todos os sensores dinâmicos/térmicos/elétricos e monitora o *watchdog* CAN ($100\text{ ms}$).
3. [`VehicleControl_EstimateChassisSpeed`](vehicle_control.c): Calcula a velocidade longitudinal $v_x$ por fusão cinemático-inercial com detecção de travamento dianteiro.
4. [`ElectronicDiff_Update`](electronic_differential.c): Calcula as rotações-alvo de Ackermann e a distribuição diferencial de torque com preservação simétrica de $2\Delta T$.
5. [`LaunchControl_Update`](launch_control.c): Atualiza a máquina de estados de largada, sobrescrevendo ou interpolando suavemente os comandos de torque quando ativa.
6. [`VehicleControl_ApplyPowerAndThermalLimits`](vehicle_control.c): Aplica *derating* térmico/subtensão, modula a frenagem regenerativa (*Brake Blending*), governa o teto de $80\text{ km/h}$ / $6000\text{ RPM}$ e limita a potência elétrica em $13{,}6\text{ kW}$ e a corrente em $220\text{ A}$.

---

## 8. Bancada de Testes Estocásticos e Simulação (Monte Carlo)

O arquivo [`main_simulation.c`](main_simulation.c) valida toda a VCU sorteando variáveis físicas reais a cada execução (mantendo fixas as constantes universais $g = 9{,}81\text{ m/s}^2$ e $\rho_{\text{ar}} = 1{,}225\text{ kg/m}^3$):

| Ensaio | Função | Parâmetros Físicos Sorteados | Invariantes Verificadas |
| :--- | :--- | :--- | :--- |
| **Teste 1** | [`RunCorneringTest`](main_simulation.c) | Raio de curva ($8\text{–}22\text{ m}$), direção (direita/esquerda), velocidade ($27\text{–}59{,}4\text{ km/h}$), pedal ($40\text{–}78\%$), subesterço ($70\text{–}88\%$), temperaturas, tensão e $\Delta t$ | Alocação correta na roda externa, simetria esquerda/direita e preservação de $2\Delta T$ sob saturação + *derating* térmico |
| **Teste 2** | [`RunLaunchControlSimulation`](main_simulation.c) | Massa ($190\text{–}250\text{ kg}$), $w_r$ ($56\text{–}64\%$), $h_{\text{CG}}$, $L$, $\mu_{\text{peak}}$ ($0{,}68\text{–}0{,}96$), $C_{rr}$, $C_d A$, $J_{\text{eixo}}$, $V_{oc}$ ($68\text{–}79{,}5\text{ V}$), $R_{\text{int}}$, pressão/tempo de *Staging* e ruído gaussiano IMU/encoders | Transição `ARMED` $\to$ `STAGING` $\to$ `LAUNCHING` $\to$ `COMPLETED` $\to$ `DISARMED`, convergência de $\lambda \approx 12\%$ e arrancada $0\text{–}60\text{ km/h}$ |
| **Teste 3** | [`RunPlausibilitySafetyTest`](main_simulation.c) | Discrepâncias APPS, pressões de freio BSPD, percentuais de retenção/rearme, temperaturas de *derating* e *cutoff*, subtensões, pressões de regeneração e bloqueio dianteiro | Corte de torque em falhas, exatidão analítica das rampas lineares de *derating*, abortamento de emergência e fusão inercial anti-*lockup* |
| **Teste 4** | [`RunMonteCarloStressTest`](main_simulation.c) | $1.000$ cenários aleatórios combinando velocidades ($0\text{–}24\text{ m/s}$), esterçamentos ($\pm 42^\circ$), temperaturas ($20\text{–}132^\circ\text{C}$), tensões ($56\text{–}81{,}5\text{ V}$), correntes e injeção de falhas | Ausência total de `NaN`/`Inf`, $\|T_{\text{mot}}\| \le 25\text{ N}\cdot\text{m}$, $\|\text{RPM}\| \le 6000$, $P_{\text{el}} \le 13{,}6\text{ kW}$ e torque zero em falhas |

---

## 9. Cronograma de Milestones e Entregáveis do Projeto

| Milestone | Data Limite | Entregável / Marco Principal | Arquivos Relacionados no Repositório |
| :--- | :---: | :--- | :--- |
| **M1** | 01/10/2026 | Kickoff, divisão de subsistemas e planejamento ágil | [`notion.md`](notion.md), [`atributos Ploger.txt`](atributos%20Ploger.txt) |
| **M2** | 23/10/2026 | Entrega do Relatório Técnico Parcial (ABNT NBR 14724) | [`apresentação.tex`](apresentação.tex), [`relatorio_tecnico_abnt.pdf`](relatorio_tecnico_abnt.pdf) |
| **M3** | 24/10/2026 | Apresentação Parcial para a Banca Avaliadora | [`apresentação_slides.tex`](apresentação_slides.tex), [`apresentação_slides.pdf`](apresentação_slides.pdf) |
| **M4** | 06/11/2026 | Entrega do Relatório Técnico Final, CAD e Simulações | [`vehicle_control.c`](vehicle_control.c), [`main_simulation.c`](main_simulation.c), [`kart tesla.dwg`](kart%20tesla.dwg) |
| **M5** | 07/11/2026 | Pitch Final da Startup para a Banca de Investidores | [`sugestões_2.txt`](sugestões_2.txt), [`apresentação_slides.pdf`](apresentação_slides.pdf) |
