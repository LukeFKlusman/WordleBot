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
from PySide6.QtWidgets import (QApplication, QGridLayout, QHBoxLayout, QMainWindow,
    QPushButton, QSizePolicy, QSpacerItem, QTabWidget,
    QVBoxLayout, QWidget)

class Ui_MainWindow(object):
    def setupUi(self, MainWindow):
        if not MainWindow.objectName():
            MainWindow.setObjectName(u"MainWindow")
        MainWindow.resize(1024, 600)
        MainWindow.setMinimumSize(QSize(1024, 600))
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
        self.horizontalLayout = QHBoxLayout(self.centralwidget)
        self.horizontalLayout.setSpacing(12)
        self.horizontalLayout.setObjectName(u"horizontalLayout")
        self.horizontalLayout.setContentsMargins(8, 8, 8, 8)
        self.mainTab = QTabWidget(self.centralwidget)
        self.mainTab.setObjectName(u"mainTab")
        self.mainTab.setMinimumSize(QSize(680, 0))
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

        self.horizontalLayout.addWidget(self.mainTab)

        self.wordleColumnLayout = QVBoxLayout()
        self.wordleColumnLayout.setSpacing(10)
        self.wordleColumnLayout.setObjectName(u"wordleColumnLayout")
        self.wordle = QWidget(self.centralwidget)
        self.wordle.setObjectName(u"wordle")
        sizePolicy = QSizePolicy(QSizePolicy.Policy.Preferred, QSizePolicy.Policy.Expanding)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(1)
        sizePolicy.setHeightForWidth(self.wordle.sizePolicy().hasHeightForWidth())
        self.wordle.setSizePolicy(sizePolicy)
        self.wordle.setMinimumSize(QSize(280, 0))
        self.wordle.setStyleSheet(u"border: 1px solid black")

        self.wordleColumnLayout.addWidget(self.wordle)

        self.safetyControls = QWidget(self.centralwidget)
        self.safetyControls.setObjectName(u"safetyControls")
        sizePolicy1 = QSizePolicy(QSizePolicy.Policy.Preferred, QSizePolicy.Policy.Fixed)
        sizePolicy1.setHorizontalStretch(0)
        sizePolicy1.setVerticalStretch(0)
        sizePolicy1.setHeightForWidth(self.safetyControls.sizePolicy().hasHeightForWidth())
        self.safetyControls.setSizePolicy(sizePolicy1)
        self.safetyControls.setMinimumSize(QSize(0, 128))
        self.safetyControls.setMaximumSize(QSize(16777215, 128))
        self.safetyControlsLayout = QVBoxLayout(self.safetyControls)
        self.safetyControlsLayout.setObjectName(u"safetyControlsLayout")
        self.safetyControlsLayout.setContentsMargins(0, 0, 0, 0)
        self.verticalSpacerTop = QSpacerItem(20, 8, QSizePolicy.Policy.Minimum, QSizePolicy.Policy.Expanding)

        self.safetyControlsLayout.addItem(self.verticalSpacerTop)

        self.gridLayout = QGridLayout()
        self.gridLayout.setObjectName(u"gridLayout")
        self.gridLayout.setHorizontalSpacing(12)
        self.gridLayout.setVerticalSpacing(12)
        self.pushButton = QPushButton(self.safetyControls)
        self.pushButton.setObjectName(u"pushButton")

        self.gridLayout.addWidget(self.pushButton, 0, 0, 1, 1)

        self.pushButton_2 = QPushButton(self.safetyControls)
        self.pushButton_2.setObjectName(u"pushButton_2")

        self.gridLayout.addWidget(self.pushButton_2, 1, 0, 1, 1)

        self.pushButton_3 = QPushButton(self.safetyControls)
        self.pushButton_3.setObjectName(u"pushButton_3")

        self.gridLayout.addWidget(self.pushButton_3, 1, 1, 1, 1)

        self.pushButton_4 = QPushButton(self.safetyControls)
        self.pushButton_4.setObjectName(u"pushButton_4")

        self.gridLayout.addWidget(self.pushButton_4, 0, 1, 1, 1)


        self.safetyControlsLayout.addLayout(self.gridLayout)

        self.verticalSpacerBottom = QSpacerItem(20, 8, QSizePolicy.Policy.Minimum, QSizePolicy.Policy.Expanding)

        self.safetyControlsLayout.addItem(self.verticalSpacerBottom)


        self.wordleColumnLayout.addWidget(self.safetyControls)


        self.horizontalLayout.addLayout(self.wordleColumnLayout)

        MainWindow.setCentralWidget(self.centralwidget)
        self.mainTab.raise_()
        self.wordle.raise_()
        self.pushButton.raise_()
        self.pushButton_2.raise_()
        self.pushButton_3.raise_()
        self.pushButton_4.raise_()

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

