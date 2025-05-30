# ## #############################################################
#
# src/pico-code-iic.py
#
# Author: Mauricio Matamoros
#
# Reads temperature from ADC using an LM35 (dummy) and sends
# it via I²C bus
#
# Pro-Tip: rename as main.py in the RP2040
# Pro-Tip: Remove anchor and main functions in i2cslave.py
#
# ## ############################################################
from i2cslave import I2CSlave
from utime import sleep_ms, sleep_us
import ustruct

I2C_SLAVE_ADDR = 0x0A


def main():
    setup()
    i = 0
    
    # Inicializar una lista vacía
    datos = [
    87314, 87322, 87298, 87244, 87113, 86917, 86654, 86275, 85860, 85381,
    84875, 84408, 83961, 83555, 83128, 82753, 82400, 82072, 81761, 81480,
    81217, 80959, 80703, 80483, 80287, 80109, 79945, 79790, 79662, 79520,
    79424, 79335, 79259, 79196, 79144, 79114, 79097, 79085, 79103, 79123,
    79169, 79218, 79277, 79693, 79734, 79756, 79802, 79809, 79860, 79896,
    79921, 79956, 79992, 80012, 80054, 80120, 80155, 80185, 80222, 80257,
    80284, 80337, 80370, 80399, 80445, 80453, 80488, 80513, 80575, 80606,
    80662, 80697, 80720, 80754, 80795, 80823, 80871, 80900, 80938, 80967,
    80988, 81040, 81068, 81118, 81149, 81185, 81226, 81270, 81301, 81335,
    81389, 81425, 81470, 81501, 81520, 81566, 81595, 81642, 81694, 81733,
    81775, 81808, 81852, 81894, 81935, 81953, 81999, 82043, 82068, 82111,
    82132, 82162, 82187, 82227, 82271, 82305, 82354, 82374, 82417, 82441,
    82478, 82522, 82553, 82600, 82634, 82677, 82716, 82748, 82778, 82818,
    82866, 82906, 82946, 82985, 83025, 83054, 83086, 83128, 83175, 83208,
    83246, 83282, 83327, 83369, 83408, 83447, 83488, 83531, 83577, 83618,
    83658, 83683, 83730, 83775, 83814, 83862, 83903, 83936, 83983, 84021,
    84060, 84100, 84138, 84183, 84221, 84264, 84299, 84328, 84388, 84425,
    84482, 84530, 84573, 84611, 84645, 84704, 84738, 84792, 84833, 84873,
    84909, 84943, 84999, 85052, 85099, 85143, 85194, 85218, 85255, 85316,
    85362, 85410, 85446, 85483, 85515, 85559, 85606, 85659, 85675, 85725,
    85755, 85790, 85834, 85872, 85923, 85970, 86008, 86041, 86069, 86103,
    86152, 86206, 86248, 86283, 86325, 86353, 86380, 86442, 86483, 86539,
    86571, 86603, 86632, 86674, 86731, 86769, 86805, 86841, 86900, 86925,
    86946, 86985, 87028, 87081, 87118, 87151, 87181, 87233, 87265, 87302,
    87332, 87369, 87416
    ]

#     # Abrir el archivo en modo lectura
#     with open('presion_50Hz.txt', 'r') as archivo:
#         for linea in archivo:
#             linea = linea.strip()  # Eliminar saltos de línea y espacios
#             if linea:              # Verificar que la línea no esté vacía
#                 datos.append(float(linea))  # Puedes usar float(linea) si los datos son números
                
    #print(datos)

    while True:
        # 3. Check if Master requested data
        if i2c.waitForRdReq(timeout=0):
            
            # 1. Get pressure
            p = datos[i]
            # 2. Converts pressure from py uint_32 to bytes
            # < little endian
            # I uint_32 '<I'
            data = ustruct.pack('<I', p)
            print("Enviando:", [hex(b) for b in data])

            # 1. Get total acceleration
            a = datos[i]
            #a = 17017.91
            # 2. Converts pressure from py float to bytes
            # < little endian
            # f  float IEEE 754 32 bits
            data1 = ustruct.pack('<f', a)
            print("Enviando:", [hex(b) for b in data1])        

            i += 1
            # If so, send the temperature to Master
            i2c.write(data)
            i2c.write(data1)
        # end if

        # 3. Check if Master sent data
        if i2c.waitForData(timeout=0):
            # If so, print it
            rcv = i2c.read()
            print( rcv.decode('utf-8') )
        # end if
# end def


def read_pressure(index):
    # simulates reading pressure sensor
    # datos obtenidos a 2.4Hz
    p = pressures[index]
    return p
# end def

def read_total_accel(index):
    # simulates reading pressure sensor
    # datos obtenidos a 2.4Hz
    ta = total_accel[index]
    return ta
# end def


def setup():
    global i2c, adcm, adcp
    i2c = I2CSlave(id=0, address=I2C_SLAVE_ADDR, sda=16, scl=17)
    adcm = machine.ADC(0)         # Init ADC0
    adcp = machine.ADC(1)         # Init ADC1
# end def


if __name__ == '__main__':
    main()
