## Proyecto: Control digital de convertidor Boost con microcontrolador RISC-V
# Procesamiento Electronico de Potencia
 
# Juan P. Elizondo
# Fernando J. Rodriguez
# William Smith

# Controlador PI
#     Entrada: Vo -> XADC
#     Salida: D -> PWM

# Especificaciones del lazo de control:
#     Vref = 24 V,  Ts = 200 us  ->  fcontrol = 5 kHz
#     PI Tustin:  u[k] = u[k-1] + b0*e[k] + b1*e[k-1]
#     Coeficientes:  b0_Q16 = +59
#                    b1_Q16 = -53


#  Cadena ADC (divisor 300k/10k, k=30):
#     Vo = adc * 30 / 4095 -> Ref_cnt = 24 * 4095 / 30 = 3276

#  Saturación del duty: [45, 75] %  (alrededor de D0 = 50 %)

#  Mapa de memoria:
#     PWM_CTRL = 0x00010100   bit0=enable, [2:1]=freq_sel
#     PWM_DUTY = 0x00010104   [6:0] = duty %  (0..100)
#     ADC_CTRL = 0x00010110   bit1=new_data (W1C), bit2=ext_start_en,
#                            bit4=pwm_trig_en
#     ADC_DATA = 0x00010114   [11:0] = adc_cnt

#  Registros permanentes:
#     s0 = base perifericos  (0x00010000 = 1 << 16)
#     s1 = Ref_cnt           (3276)
#     s2 = b0_Q16            (+43)
#     s3 = b1_Q16            (-39)
#     s4 = delta_q           (acumulador PI en Q16, signed)
#     s5 = e_prev            (error anterior, signed)

#  Mapa de memoria:
#     s0 + 0x100 = 0x00010100  PWM_CTRL
#     s0 + 0x104 = 0x00010104  PWM_DUTY   [6:0] = duty %
#     s0 + 0x110 = 0x00010110  ADC_CTRL   bit1=new_data(W1C), bit4=pwm_trig_en
#     s0 + 0x114 = 0x00010114  ADC_DATA   [11:0] = adc_cnt

# =============================================================================
#  Inicialización
# =============================================================================
.text
.globl _start
_start:
    # Base de periféricos en s0
    # 0x00010000 = 1 << 16
    addi s0, x0, 1
    slli s0, s0, 16

    # Constantes del PI en registros
    
    # s1 = 3276  (Ref_cnt)
    # 3276 = 4096 - 820 = (1<<12) - 820
    addi s1, x0, 1
    slli s1, s1, 12 # s1 = 4096
    addi s1, s1, -820 # s1 = 3276
    
    # s2 = 43  (b0_Q16)
    addi s2, x0, 43

    # s3 = -39  (b1_Q16)
    addi s3, x0, -39

    # s4 = 0  (delta_q)
    addi s4, x0, 0

    # s5 = 0  (e_prev)
    addi s5, x0, 0


    # PWM_DUTY = 50 (duty inicial D0)
    addi t0, x0, 50
    sw t0, 0x104(s0)
    
    # PWM_CTRL = 3  (enable=1, freq_sel=01)
    addi t0, x0, 3
    sw t0, 0x100(s0)

    # ADC_CTRL = 20 (0x14: bit4=pwm_trig_en, bit2=ext_start_en)
    addi t0, x0, 20
    sw t0, 0x110(s0)

# =============================================================================
#  LAZO PRINCIPAL DE CONTROL  (5 kHz)
# =============================================================================
ctrl_loop:
    # Esperar new_data del ADC (bit1)
wait_adc:
    lw    a0, 0x110(s0)
    andi  a0, a0, 2
    beq   a0, x0, wait_adc

    # Leer dato ADC y enmascarar a 12 bits
    # Mascara 0xFFF = 4095 > 2047 (12 bits imm), no cabe en andi, se construye en t0
    lw a2, 0x114(s0)
    addi t0, x0, 1
    slli t0, t0, 12 # t0 = 4096
    addi t0, t0, -1 # t0 = 4095 = 0xFFF
    and a2, a2, t0 # a2 = adc_cnt (0..4095)

    # Limpiar flag new_data (W1C en bit1)
    addi t0, x0, 2
    sw t0, 0x110(s0)

    # Calcular error e[k] = Ref_cnt - adc_cnt
    sub a3, s1, a2 # a3 = e[k]

    # Producto b0_Q16 * e[k] 
    addi a0, s2, 0 # a0 = b0_Q16
    addi a1, a3, 0 # a1 = e[k]
    jal ra, mul32
    addi t3, a0, 0 # t3 = b0 * e[k]

    # Producto b1_Q16 * e[k-1]
    addi a0, s3, 0 # a0 = b1_Q16
    addi a1, s5, 0 # a1 = e_prev
    jal ra, mul32
    add t3, t3, a0 # t3 = b0*e[k] + b1*e[k-1]

    # Acumular en delta_q (Q16)
    add s4, s4, t3

    # Limite superior: DELTA_MAX_Q = 1638400 = 0x190 << 12
    addi t0, x0, 0x190 # t0 = 400
    slli t0, t0, 12 # t0 = 1638400
    blt s4, t0, sat_hi_ok
    addi s4, t0, 0 # s4 = 1638400
sat_hi_ok:

    # Limite inferior: DELTA_MIN_Q = -327680 = -80 << 12
    addi t0, x0, -80
    slli t0, t0, 12 # t0 = -327680
    bge s4, t0, sat_lo_ok
    addi s4, t0, 0 # s4 = -327680
sat_lo_ok:

    # Q_ROUND = 32768 = 8 << 12
    addi t0, x0, 8
    slli t0, t0, 12 # t0 = 32768
    add t1, s4, t0
    srai t1, t1, 16 # t1 = delta_pct (signed)

    # duty = D0 + delta_pct, saturar a [DUTY_MIN, DUTY_MAX]
    addi t2, t1, 50
    
    addi t0, x0, 75
    blt t2, t0, sat_dmax_ok
    addi t2, t0, 0 # t2 = 75
    
sat_dmax_ok:
    addi t0, x0, 45
    bge t2, t0, sat_dmin_ok
    addi t2, t0, 0 # t2 = 45
    
sat_dmin_ok:

    # Escribir nuevo duty al PWM
    sw t2, 0x104(s0)

    # Actualizar e_prev
    addi s5, a3, 0

	# Volver a iniciar el lazo de control
    j ctrl_loop

# =============================================================================
#  mul32 - Multiplicacion 32x32 -> 32 bits bajos
#  Sin instruccion mul. Algoritmo shift-and-add, 32 iteraciones.
#  Entrada:  a0 = X, a1 = Y
#  Salida:   a0 = (X * Y) mod 2^32  (correcto en complemento a 2)
#  Modifica: t0, t1, t2  (no preservados)
#  Costo:    ~192 ciclos a 100 MHz = ~2 us << 200 us del periodo
# =============================================================================
mul32:
    addi  t0, x0, 0             # t0 = acumulador = 0
    addi  t1, x0, 32            # t1 = contador = 32
mul32_loop:
    andi  t2, a1, 1             # t2 = bit LSB de Y
    beq   t2, x0, mul32_skip
    add   t0, t0, a0            # acumulador += X
mul32_skip:
    slli  a0, a0, 1             # X <<= 1
    srli  a1, a1, 1             # Y >>= 1 (logico)
    addi  t1, t1, -1
    bne   t1, x0, mul32_loop
    addi  a0, t0, 0             # a0 = resultado
    jalr  x0, ra, 0             # return

# FIN