# -*- coding: utf-8 -*-

################################################################################
## Form generated from reading UI file 'rs2_concept.ui'
##
## Created by: Qt User Interface Compiler version 6.10.2
##
## WARNING! All changes made in this file will be lost when recompiling UI file!
################################################################################

from PySide6.QtCore import (QCoreApplication, QDate, QDateTime, QLocale,
    QMetaObject, QObject, QPoint, QRect,
    QSize, QTime, QUrl, Qt)
from PySide6.QtGui import (QBrush, QColor, QConicalGradient, QCursor,
    QFont, QFontDatabase, QGradient, QIcon,
    QImage, QKeySequence, QLinearGradient, QPainter,
    QPalette, QPixmap, QRadialGradient, QTransform)
from PySide6.QtWidgets import (QApplication, QGridLayout, QMainWindow, QPushButton,
    QSizePolicy, QTabWidget, QWidget)

class Ui_MainWindow(object):
    def setupUi(self, MainWindow):
        if not MainWindow.objectName():
            MainWindow.setObjectName(u"MainWindow")
        MainWindow.resize(1024, 640)
        MainWindow.setMinimumSize(QSize(1024, 640))
        palette = QPalette()
        brush = QBrush(QColor(255, 255, 220, 255))
        brush.setStyle(Qt.BrushStyle.SolidPattern)
        palette.setBrush(QPalette.ColorGroup.Active, QPalette.ColorRole.Button, brush)
        palette.setBrush(QPalette.ColorGroup.Inactive, QPalette.ColorRole.Button, brush)
        palette.setBrush(QPalette.ColorGroup.Disabled, QPalette.ColorRole.Button, brush)
        MainWindow.setPalette(palette)
        font = QFont()
        font.setFamilies([u"Abyssinica SIL"])
        font.setPointSize(20)
        font.setBold(True)
        MainWindow.setFont(font)
        self.centralwidget = QWidget(MainWindow)
        self.centralwidget.setObjectName(u"centralwidget")
        self.mainTab = QTabWidget(self.centralwidget)
        self.mainTab.setObjectName(u"mainTab")
        self.mainTab.setGeometry(QRect(0, 0, 721, 641))
        self.mainTab.setStyleSheet(u"QTabBar {\n"
"    background: #2e2e2e;\n"
"}\n"
"\n"
"QTabBar::tab {\n"
"    background: #4a4a4a;\n"
"    color: white;\n"
"}\n"
"\n"
"QTabBar::tab:selected {\n"
"    background: #1e1e1e;\n"
"}")
        self.tab = QWidget()
        self.tab.setObjectName(u"tab")
        self.mainTab.addTab(self.tab, "")
        self.tab_2 = QWidget()
        self.tab_2.setObjectName(u"tab_2")
        self.mainTab.addTab(self.tab_2, "")
        self.tab_3 = QWidget()
        self.tab_3.setObjectName(u"tab_3")
        self.mainTab.addTab(self.tab_3, "")
        self.wordle = QWidget(self.centralwidget)
        self.wordle.setObjectName(u"wordle")
        self.wordle.setGeometry(QRect(720, 0, 1024, 640))
        self.wordle.setMinimumSize(QSize(1024, 640))
        self.wordle.setStyleSheet(u"border: 1px solid black")
        self.layoutWidget = QWidget(self.centralwidget)
        self.layoutWidget.setObjectName(u"layoutWidget")
        self.layoutWidget.setGeometry(QRect(720, 490, 301, 141))
        self.gridLayout = QGridLayout(self.layoutWidget)
        self.gridLayout.setObjectName(u"gridLayout")
        self.gridLayout.setContentsMargins(0, 0, 0, 0)
        self.pushButton = QPushButton(self.layoutWidget)
        self.pushButton.setObjectName(u"pushButton")

        self.gridLayout.addWidget(self.pushButton, 0, 0, 1, 1)

        self.pushButton_2 = QPushButton(self.layoutWidget)
        self.pushButton_2.setObjectName(u"pushButton_2")

        self.gridLayout.addWidget(self.pushButton_2, 1, 0, 1, 1)

        self.pushButton_3 = QPushButton(self.layoutWidget)
        self.pushButton_3.setObjectName(u"pushButton_3")

        self.gridLayout.addWidget(self.pushButton_3, 1, 1, 1, 1)

        self.pushButton_4 = QPushButton(self.layoutWidget)
        self.pushButton_4.setObjectName(u"pushButton_4")

        self.gridLayout.addWidget(self.pushButton_4, 0, 1, 1, 1)

        MainWindow.setCentralWidget(self.centralwidget)

        self.retranslateUi(MainWindow)

        self.mainTab.setCurrentIndex(0)


        QMetaObject.connectSlotsByName(MainWindow)
    # setupUi

    def retranslateUi(self, MainWindow):
        MainWindow.setWindowTitle(QCoreApplication.translate("MainWindow", u"WordleBot", None))
        self.mainTab.setTabText(self.mainTab.indexOf(self.tab), QCoreApplication.translate("MainWindow", u"Sim View", None))
        self.mainTab.setTabText(self.mainTab.indexOf(self.tab_2), QCoreApplication.translate("MainWindow", u"Camera", None))
        self.mainTab.setTabText(self.mainTab.indexOf(self.tab_3), QCoreApplication.translate("MainWindow", u"Diagnostics", None))
        self.pushButton.setText(QCoreApplication.translate("MainWindow", u"START", None))
        self.pushButton_2.setText(QCoreApplication.translate("MainWindow", u"RESUME", None))
        self.pushButton_3.setText(QCoreApplication.translate("MainWindow", u"HOME", None))
        self.pushButton_4.setText(QCoreApplication.translate("MainWindow", u"STOP", None))
    # retranslateUi

