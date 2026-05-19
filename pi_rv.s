## Proyecto: Control digital de convertidor Boost con microcontrolador RISC-V
# Procesamiento Electronico de Potencia
 
#   Juan P. Elizondo
#   Fernando J. Rodriguez
#   William Smith

# Controlador PI
#   Entrada: Vo > XADC
#   Salida: D > PWM

# Especificaciones del lazo de control:
#    Vref = 24 V,  Ts = 200 us  ->  fcontrol = 5 kHz
#    PI Tustin:  u[k] = u[k-1] + b0*e[k] + b1*e[k-1]
#    Coeficientes:  b0_Q16 = +59
#                   b1_Q16 = -53

#  Cadena ADC (divisor 300k/10k, k=30):
#    Vo = adc * 30 / 4095 -> Ref_cnt = 24 * 4095 / 30 = 3276

#  Saturación del duty: [45, 75] %  (alrededor de D0 = 50 %)

#  Mapa de memoria:
#    PWM_CTRL = 0x00010100   bit0=enable, [2:1]=freq_sel
#    PWM_DUTY = 0x00010104   [6:0] = duty %  (0..100)
#    ADC_CTRL = 0x00010110   bit1=new_data (W1C), bit2=ext_start_en,
#                            bit4=pwm_trig_en
#    ADC_DATA = 0x00010114   [11:0] = adc_cnt

# ---------- Constantes ----------
.equ PERIPH_BASE,   0x00010000
.equ PWM_CTRL_OFF,  0x0100
.equ PWM_DUTY_OFF,  0x0104
.equ ADC_CTRL_OFF,  0x0110
.equ ADC_DATA_OFF,  0x0114

.equ REF_CNT,       3276
.equ B0_Q16,        59
.equ B1_Q16,        -53
.equ Q_SHIFT,       16
.equ Q_ROUND,       0x00008000 # 1 << (Q_SHIFT - 1) = 32768

.equ D0_PCT,        50
.equ DUTY_MIN,      45
.equ DUTY_MAX,      75
.equ DELTA_MIN_Q,   -327680 # -5  * 2^16
.equ DELTA_MAX_Q,   1638400 # +25 * 2^16

# =============================================================================
#  RESET / Inicialización
# =============================================================================
.section .text
.globl _start
_start:
    # Base de periféricos en s0
    lui   s0, %hi(PERIPH_BASE)
    addi  s0, s0, %lo(PERIPH_BASE)

    # Constantes del PI en registros
    li    s1, REF_CNT
    li    s2, B0_Q16
    li    s3, B1_Q16
    li    s4, 0 # delta_q acumulador = 0
    li    s5, 0 # e_prev = 0

    # Configurar PWM con duty inicial D0
    li    t0, D0_PCT
    sw    t0, PWM_DUTY_OFF(s0)
    li    t0, 0x03 # enable=1, freq_sel=01 (~50 kHz)
    sw    t0, PWM_CTRL_OFF(s0)

    # Configurar ADC: disparo automático por PWM
    li    t0, 0x14 # bit4=pwm_trig_en, bit2=ext_start_en
    sw    t0, ADC_CTRL_OFF(s0)

# =============================================================================
#  LAZO PRINCIPAL DE CONTROL  (5 kHz)
# =============================================================================
ctrl_loop:
    # Esperar new_data del ADC (bit1)
wait_adc:
    lw    a0, ADC_CTRL_OFF(s0)
    andi  a0, a0, 0x2
    beq   a0, zero, wait_adc

    # Leer dato ADC y enmascarar a 12 bits
    lw    a2, ADC_DATA_OFF(s0)
    andi  a2, a2, 0xFFF # a2 = adc_cnt (0..4095)

    # Limpiar flag new_data (W1C en bit1)
    li    t0, 0x2
    sw    t0, ADC_CTRL_OFF(s0)

    # Calcular error e[k] = Ref_cnt - adc_cnt
    sub   a3, s1, a2 # a3 = e[k]  (signed)

    # Producto b0_Q16 * e[k] 
    mv    a0, s2 # a0 = b0_Q16
    mv    a1, a3 # a1 = e[k]
    jal   ra, mul32
    mv    t3, a0 # t3 = b0 * e[k]

    # Producto b1_Q16 * e[k-1]
    mv    a0, s3 # a0 = b1_Q16
    mv    a1, s5 # a1 = e_prev
    jal   ra, mul32
    add   t3, t3, a0 # t3 = b0*e[k] + b1*e[k-1]

    # Acumular en delta_q (Q16)
    add   s4, s4, t3

    # Saturar delta_q (anti-windup)
    li    t0, DELTA_MAX_Q
    blt   s4, t0, sat_hi_ok
    mv    s4, t0
sat_hi_ok:
    li    t0, DELTA_MIN_Q
    bge   s4, t0, sat_lo_ok
    mv    s4, t0
sat_lo_ok:

    # Convertir Q16 a entero con redondeo
    li    t0, Q_ROUND
    add   t1, s4, t0
    srai  t1, t1, Q_SHIFT # t1 = delta_pct (signed)

    # duty = D0 + delta_pct, saturar a [DUTY_MIN, DUTY_MAX]
    addi  t2, t1, D0_PCT
    li    t0, DUTY_MAX
    blt   t2, t0, sat_dmax_ok
    mv    t2, t0
sat_dmax_ok:
    li    t0, DUTY_MIN
    bge   t2, t0, sat_dmin_ok
    mv    t2, t0
sat_dmin_ok:

    # Escribir nuevo duty al PWM
    sw    t2, PWM_DUTY_OFF(s0)

    # 12. Actualizar e_prev
    mv    s5, a3

    j     ctrl_loop

# =============================================================================
#  mul32 - multiplicación 32x32 -> 32 bits bajos (signed/unsigned)
#  Entrada: a0 = X, a1 = Y
#  Salida:  a0 = (X * Y) mod 2^32
#  Algoritmo: shift-and-add de 32 iteraciones. El resultado en complemento
#  a 2 de los 32 bits bajos es correcto para operandos con cualquier signo.
#  Costo: ~6 instrucciones x 32 iter = ~200 ciclos.
#  A 100 MHz: 2 us << 200 us del periodo de muestreo.
# =============================================================================
mul32:
    li    t0, 0                    # acumulador
    li    t1, 32                   # contador de bits
mul32_loop:
    andi  t2, a1, 1                # bit menos significativo de Y
    beq   t2, zero, mul32_skip
    add   t0, t0, a0
mul32_skip:
    slli  a0, a0, 1                # X <<= 1
    srli  a1, a1, 1                # Y >>= 1
    addi  t1, t1, -1
    bne   t1, zero, mul32_loop
    mv    a0, t0
    jalr  zero, ra, 0

    # FIN