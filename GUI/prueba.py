import sys,os
from PyQt4.QtCore import *
from PyQt4.QtGui import *

class Window(QMainWindow):

 def __init__(self):
        super(Window, self).__init__()

        self.setGeometry(150, 150, 150,150)
        self.roscore = QPushButton('Iniciar ROS', self)
        self.roscore.move(10,20)
        self.roscore.clicked.connect(self.callROSCORE)
	self.inconfig = QPushButton('Ver IP', self)
        self.inconfig .move(10,50)
        self.inconfig .clicked.connect(self.callIFCONFIG)


 def callROSCORE(self):
        os.system("roscore &")

 def callIFCONFIG(self):
        os.system("ifconfig &")

def main():
    app = QApplication(sys.argv)
    ex = Window()
    ex.show()
    sys.exit(app.exec_())

if __name__ == '__main__':
   main()
