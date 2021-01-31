import serial
import serial.tools.list_ports
ports = serial.tools.list_ports.comports()
serial = serial.Serial(ports[-1][0], 115200, timeout=2)


def rotate(num,i2):
	cmd = f'#{num}P{i2}S2000\r\n'
	serial.write(str.encode(cmd))
rotate(3,1213)
#2369:90,1418:0,1844:45