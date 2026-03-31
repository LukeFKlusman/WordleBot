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
from PySide6.QtWidgets import (QApplication, QGridLayout, QHBoxLayout, QLabel,
    QMainWindow, QPushButton, QSizePolicy, QTabWidget,
    QVBoxLayout, QWidget)

class Ui_MainWindow(object):
    def setupUi(self, MainWindow):
        if not MainWindow.objectName():
            MainWindow.setObjectName(u"MainWindow")
        MainWindow.resize(1024, 600)
        MainWindow.setMinimumSize(QSize(1024, 600))
        font = QFont()
        font.setFamilies([u"Ubuntu"])
        font.setPointSize(10)
        font.setBold(False)
        MainWindow.setFont(font)
        MainWindow.setStyleSheet(u"QMainWindow {\n"
"    background-color: #0f1117;\n"
"}")
        self.centralwidget = QWidget(MainWindow)
        self.centralwidget.setObjectName(u"centralwidget")
        self.centralwidget.setStyleSheet(u"QWidget#centralwidget {\n"
"    background-color: #0f1117;\n"
"}")
        self.horizontalLayout = QHBoxLayout(self.centralwidget)
        self.horizontalLayout.setSpacing(0)
        self.horizontalLayout.setObjectName(u"horizontalLayout")
        self.horizontalLayout.setContentsMargins(0, 0, 0, 0)
        self.mainTab = QTabWidget(self.centralwidget)
        self.mainTab.setObjectName(u"mainTab")
        self.mainTab.setMinimumSize(QSize(680, 0))
        self.mainTab.setStyleSheet(u"QTabWidget::pane {\n"
"    background-color: #0d1018;\n"
"    border: none;\n"
"    margin: 0px;\n"
"    padding: 0px;\n"
"    top: -8px;\n"
"}\n"
"QTabWidget::tab-bar {\n"
"    alignment: left;\n"
"}\n"
"QTabBar {\n"colcon build --packages-select interaction_execution
    source install/setup.bash
    ros2 launch interaction_execution gui.launch.py
"    background-color: #1a1f2e;\n"
"}\n"
"QTabBar::tab {\n"
"    background-color: transparent;\n"
"    color: #64748b;\n"
"    padding: 8px 20px;\n"
"    font-size: 9pt;\n"
"    font-weight: 600;\n"
"    border-bottom: 2px solid transparent;\n"
"    border-top: none;\n"
"    border-left: none;\n"
"    border-right: none;\n"
"    min-width: 80px;\n"
"}\n"
"QTabBar::tab:selected {\n"
"    color: #a5b4fc;\n"
"    border-bottom: 2px solid #6366f1;\n"
"    background-color: transparent;\n"
"}\n"
"QTabBar::tab:hover:!selected {\n"
"    color: #94a3b8;\n"
"    background-color: rgba(255, 255, 255, 0.04);\n"
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
        self.wordleColumnLayout.setSpacing(0)
        self.wordleColumnLayout.setObjectName(u"wordleColumnLayout")
        self.wordleColumnLayout.setContentsMargins(0, 0, 0, 0)
        self.wordle = QWidget(self.centralwidget)
        self.wordle.setObjectName(u"wordle")
        sizePolicy = QSizePolicy(QSizePolicy.Policy.Preferred, QSizePolicy.Policy.Expanding)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(1)
        sizePolicy.setHeightForWidth(self.wordle.sizePolicy().hasHeightForWidth())
        self.wordle.setSizePolicy(sizePolicy)
        self.wordle.setMinimumSize(QSize(230, 0))
        self.wordle.setStyleSheet(u"QWidget#wordle {\n"
"    background-color: #0f1117;\n"
"    border: none;\n"
"}")

        self.wordleColumnLayout.addWidget(self.wordle)

        self.safetyControls = QWidget(self.centralwidget)
        self.safetyControls.setObjectName(u"safetyControls")
        sizePolicy1 = QSizePolicy(QSizePolicy.Policy.Preferred, QSizePolicy.Policy.Fixed)
        sizePolicy1.setHorizontalStretch(0)
        sizePolicy1.setVerticalStretch(0)
        sizePolicy1.setHeightForWidth(self.safetyControls.sizePolicy().hasHeightForWidth())
        self.safetyControls.setSizePolicy(sizePolicy1)
        self.safetyControls.setMinimumSize(QSize(0, 130))
        self.safetyControls.setMaximumSize(QSize(16777215, 130))
        self.safetyControls.setStyleSheet(u"QWidget#safetyControls {\n"
"    background-color: #130d0d;\n"
"    border-top: 2px solid rgba(220, 38, 38, 0.5);\n"
"}\n"
"QPushButton {\n"
"    border-radius: 6px;\n"
"    font-size: 9pt;\n"
"    font-weight: 700;\n"
"    padding: 8px 0px;\n"
"}\n"
"QPushButton#pushButton {\n"
"    background-color: #6366f1;\n"
"    color: white;\n"
"    border: none;\n"
"}\n"
"QPushButton#pushButton:hover {\n"
"    background-color: #4f52d4;\n"
"}\n"
"QPushButton#pushButton:pressed {\n"
"    background-color: #4338ca;\n"
"}\n"
"QPushButton#pushButton_4 {\n"
"    background-color: #dc2626;\n"
"    color: white;\n"
"    border: 1px solid #ef4444;\n"
"}\n"
"QPushButton#pushButton_4:hover {\n"
"    background-color: #b91c1c;\n"
"}\n"
"QPushButton#pushButton_4:pressed {\n"
"    background-color: #991b1b;\n"
"}\n"
"QPushButton#pushButton_2,\n"
"QPushButton#pushButton_3 {\n"
"    background-color: #1e293b;\n"
"    color: #94a3b8;\n"
"    border: 1px solid rgba(255, 255, 255, 0.1);\n"
"}\n"
"QPushButton#pushButton_2:hover,\n"
"QPus"
                        "hButton#pushButton_3:hover {\n"
"    background-color: #273449;\n"
"}\n"
"QPushButton#pushButton_2:pressed,\n"
"QPushButton#pushButton_3:pressed {\n"
"    background-color: #1a2438;\n"
"}")
        self.safetyControlsLayout = QVBoxLayout(self.safetyControls)
        self.safetyControlsLayout.setSpacing(8)
        self.safetyControlsLayout.setObjectName(u"safetyControlsLayout")
        self.safetyControlsLayout.setContentsMargins(12, 10, 12, 12)
        self.safetyLabel = QLabel(self.safetyControls)
        self.safetyLabel.setObjectName(u"safetyLabel")
        self.safetyLabel.setStyleSheet(u"QLabel#safetyLabel {\n"
"    color: #ef4444;\n"
"    font-size: 8pt;\n"
"    font-weight: 700;\n"
"    letter-spacing: 2px;\n"
"    background: transparent;\n"
"    border: none;\n"
"    padding: 0px;\n"
"}")

        self.safetyControlsLayout.addWidget(self.safetyLabel)

        self.gridLayout = QGridLayout()
        self.gridLayout.setObjectName(u"gridLayout")
        self.gridLayout.setHorizontalSpacing(8)
        self.gridLayout.setVerticalSpacing(8)
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
        self.safetyLabel.setText(QCoreApplication.translate("MainWindow", u"\u26a0  SAFETY CONTROLS", None))
        self.pushButton.setText(QCoreApplication.translate("MainWindow", u"START", None))
        self.pushButton_2.setText(QCoreApplication.translate("MainWindow", u"RESUME", None))
        self.pushButton_3.setText(QCoreApplication.translate("MainWindow", u"HOME", None))
        self.pushButton_4.setText(QCoreApplication.translate("MainWindow", u"STOP", None))
    # retranslateUi

