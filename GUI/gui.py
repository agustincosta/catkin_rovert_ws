import sys,os
from PyQt4 import QtGui, QtCore


class SecondaryWindow(QtGui.QMainWindow):
    def __init__(self):
        super(SecondaryWindow, self).__init__()
        

class MainWindow(QtGui.QMainWindow):

	def __init__(self):
		super(MainWindow, self).__init__()

		self.setGeometry(300, 300, 290, 280)
		self.setWindowTitle('Rovert')
		self.setWindowIcon(QtGui.QIcon('icon_fondo_2.png'))

		self.rpiSSH = QtGui.QPushButton('Conectar Raspberry Pi', self)
		self.rpiSSH.resize(200,40)
		self.rpiSSH.move(50,20)
		self.rpiSSH.clicked.connect(self.RaspberryConnect)

		self.teleop = QtGui.QPushButton('Mapeo manual', self)
		self.teleop.resize(200,40)
		self.teleop.move(50,80)
		self.teleop.clicked.connect(self.modoManual)

		self.auto = QtGui.QPushButton('Movimiento autonomo', self)
		self.auto.resize(200,40)
		self.auto.move(50,140)
		self.auto.clicked.connect(self.moveBase)

		self.explore = QtGui.QPushButton('Mapeo autonomo', self)
		self.explore.resize(200,40)
		self.explore.move(50,200)
		self.explore.clicked.connect(self.exploration)

	#def openNewWindow(self):
	#   self.nw = SecondaryWindow(self)
	#	self.nw.show()

	def RaspberryConnect(self):
 		os.system("../Scripts/rpi_ssh_localized.sh &")

	def modoManual(self):
		os.system("roslaunch rovert manual_mapping.launch &")
		#self.openNewWindow()

	def moveBase(self):
		os.system("roslaunch rovert autonomous_movement.launch &")


	def exploration(self):
		os.system("roslaunch rovert exploration.launch &")




def main():
	app = QtGui.QApplication(sys.argv)
	ex = MainWindow()
	ex.show()
	sys.exit(app.exec_())

if __name__ == '__main__':
	main()
