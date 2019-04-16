import sys,os
from PyQt4 import QtGui, QtCore


class SecondaryWindow(QtGui.QMainWindow):
	def __init__(self, parent=None):
		super(SecondaryWindow, self).__init__(parent)

		self.setGeometry(200, 200, 190, 180)
		self.setWindowTitle('Acciones')
		self.setWindowIcon(QtGui.QIcon('icon_fondo_2.png'))

		self.cambioModo = QtGui.QPushButton('Cambiar Modo', self)
		self.cambioModo.resize(120,40)
		self.cambioModo.move(40,20)
		self.cambioModo.clicked.connect(self.changeMode)

		self.guardaMapa = QtGui.QPushButton('Guardar Mapa', self)
		self.guardaMapa.resize(120,40)
		self.guardaMapa.move(40,70)
		self.guardaMapa.clicked.connect(self.mapSaver)

	
	def changeMode(self):
		os.system("rosnode kill move_base; rosnode kill explore_lite; rosnode kill rviz; rosnode kill teleop &")
		self.close()

	def mapSaver(self):
		file = str(QtGui.QFileDialog.getExistingDirectory(self, "Select Directory"))
		print(file)
		os.system("rosrun map_server map_saver -f " + file)

class MainWindow(QtGui.QMainWindow):

	def __init__(self, parent=None):
		super(MainWindow, self).__init__(parent)

		self.setGeometry(300, 300, 290, 280)
		self.setWindowTitle('Rovert')
		self.setWindowIcon(QtGui.QIcon('icon_fondo_2.png'))

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
 		os.system("../Scripts/rpi_ssh_localized.sh &")
 		self.rpiSSH.setEnabled(False)
 		self.rpiSSH.setText("Conectado")
 		self.teleop.setEnabled(True)
 		self.auto.setEnabled(True)
 		self.explore.setEnabled(True)

	def modoManual(self):
		os.system("roslaunch rovert manual_mapping.launch &")
		self.openNewWindow()

	def moveBase(self):
		os.system("roslaunch rovert autonomous_movement.launch &")
		self.openNewWindow()

	def exploration(self):
		os.system("roslaunch rovert exploration.launch &")
		self.openNewWindow()


def main():
	app = QtGui.QApplication(sys.argv)
	ex = MainWindow()
	ex.show()
	sys.exit(app.exec_())

if __name__ == '__main__':
	main()
