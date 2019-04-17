import sys,os
from PyQt4 import QtGui, QtCore
import paramiko, socket, time


class SecondaryWindow(QtGui.QMainWindow):
	def __init__(self, parent=None):
		super(SecondaryWindow, self).__init__(parent)

		self.setGeometry(200, 200, 190, 140)
		self.setWindowTitle('Acciones')
		self.setWindowIcon(QtGui.QIcon('icon_fondo_3.png'))

		self.cambioModo = QtGui.QPushButton('Cambiar Modo', self)
		self.cambioModo.resize(120,40)
		self.cambioModo.move(35,20)
		self.cambioModo.clicked.connect(self.changeMode)

		self.guardaMapa = QtGui.QPushButton('Guardar Mapa', self)
		self.guardaMapa.resize(120,40)
		self.guardaMapa.move(35,80)
		self.guardaMapa.clicked.connect(self.mapSaver)
	
	def changeMode(self):
		os.system("rosnode kill move_base; rosnode kill explore_lite; rosnode kill rviz; rosnode kill teleop &")
		self.close()

	def mapSaver(self):
		file = str(QtGui.QFileDialog.getSaveFileName(self, "Guardar mapa", ".yaml", "YAML File (*.yaml)"))
		print(file)
		os.system("rosrun map_server map_saver -f " + file)

class MainWindow(QtGui.QMainWindow):

	def __init__(self, parent=None):
		super(MainWindow, self).__init__(parent)

		self.setGeometry(300, 300, 290, 280)
		self.setWindowTitle('Rovert')
		self.setWindowIcon(QtGui.QIcon('icon_fondo_3.png'))

		self.second = SecondaryWindow(self)

		self.rpiSSH = QtGui.QPushButton('Conectar Raspberry Pi', self)
		self.rpiSSH.setEnabled(True)
		self.rpiSSH.resize(200,40)
		self.rpiSSH.move(50,20)
		self.rpiSSH.clicked.connect(self.RaspberryConnect)

		self.teleop = QtGui.QPushButton('Mapeo manual', self)
		self.teleop.setEnabled(False)
		self.teleop.setCheckable(True)
		self.teleop.resize(200,40)
		self.teleop.move(50,80)
		self.teleop.clicked.connect(self.modoManual)

		self.auto = QtGui.QPushButton('Movimiento autonomo', self)
		self.auto.setEnabled(False)
		self.auto.setCheckable(True)
		self.auto.resize(200,40)
		self.auto.move(50,140)
		self.auto.clicked.connect(self.moveBase)

		self.explore = QtGui.QPushButton('Mapeo autonomo', self)
		self.explore.setEnabled(False)
		self.explore.setCheckable(True)
		self.explore.resize(200,40)
		self.explore.move(50,200)
		self.explore.clicked.connect(self.exploration)

	def openNewWindow(self):
		self.second.show()

	def RaspberryConnect(self):
		connection(self)

	def modoManual(self):
		os.system("roslaunch rovert manual_mapping.launch &")
		self.openNewWindow()

	def moveBase(self):
		os.system("roslaunch rovert autonomous_movement.launch &")
		self.openNewWindow()

	def exploration(self):
		os.system("roslaunch rovert exploration.launch &")
		self.openNewWindow()

	def closeEvent(self, *args, **kwargs):
		super(QtGui.QMainWindow, self).closeEvent(*args, **kwargs)
		os.system("rosnode kill -a")

def connection(ventana):
	hostname = "ubiquityrobot"
	port = 22
	username = "ubuntu"
	password = "ubuntu"
	host_name = socket.gethostname() 
	host_ip = socket.gethostbyname(host_name) 
	host_ip = [l for l in ([ip for ip in socket.gethostbyname_ex(socket.gethostname())[2] 
	if not ip.startswith("127.")][:1], [[(s.connect(('8.8.8.8', 53)), 
	s.getsockname()[0], s.close()) for s in [socket.socket(socket.AF_INET, 
	socket.SOCK_DGRAM)]][0][1]]) if l][0][0]
	print host_ip
	command = "source catkin_ws/devel/setup.bash ; export ROS_MASTER_URI=http://"+ host_ip +":11311; roslaunch rover_2dnav rpi_localized_config.launch"

	try:
		ssh = paramiko.SSHClient()

		ssh.load_system_host_keys()

		#ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
		ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
		
		ssh.connect(hostname, username=username, password=password, look_for_keys=False)

		#stdin, stdout, stderr = ssh.exec_command(command)

		channel = ssh.get_transport().open_session()
		channel.invoke_shell()

		while channel.recv_ready():
			channel.recv(1024)
		channel.sendall("source catkin_ws/devel/setup.bash\n")

		channel.sendall("export ROS_MASTER_URI=http://"+ host_ip +":11311\n")
		channel.sendall("echo $ROS_MASTER_URI\n")
		print channel.recv(1024)
		channel.sendall("roslaunch rover_2dnav rpi_localized_config.launch\n")
		print channel.recv(8192)

		ventana.rpiSSH.setEnabled(False)
		ventana.rpiSSH.setText("Conectado")
		ventana.teleop.setEnabled(True)
		ventana.auto.setEnabled(True)
		ventana.explore.setEnabled(True)

	except:
		sshDialog()

def sshDialog():
   msg = QtGui.QMessageBox()
   msg.setIcon(QtGui.QMessageBox.Critical)

   msg.setText("Error de conexion")
   msg.setInformativeText("Comunicacion SSH a Raspberry Pi ha fallado")
   msg.setWindowTitle("Error")
   msg.setStandardButtons(QtGui.QMessageBox.Ok)
   msg.setDefaultButton(QtGui.QMessageBox.Ok)
   msg.buttonClicked.connect(msgbtn)
	
   retval = msg.exec_()

def msgbtn():
	print("Boton apretado")

def main():
	os.system("roscore &")
	app = QtGui.QApplication(sys.argv)
	ex = MainWindow()
	ex.show()
	sys.exit(app.exec_())

if __name__ == '__main__':
	main()
