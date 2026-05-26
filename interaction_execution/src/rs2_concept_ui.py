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
from PySide6.QtWidgets import (QApplication, QFrame, QGridLayout, QHBoxLayout,
    QLabel, QMainWindow, QPushButton, QSizePolicy,
    QStackedWidget, QVBoxLayout, QWidget)

class Ui_MainWindow(object):
    def setupUi(self, MainWindow):
        if not MainWindow.objectName():
            MainWindow.setObjectName(u"MainWindow")
        MainWindow.resize(1280, 720)
        MainWindow.setMinimumSize(QSize(1280, 720))
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
"    background-color: #1a1f2e;\n"
"}")
        self.horizontalLayout = QHBoxLayout(self.centralwidget)
        self.horizontalLayout.setSpacing(0)
        self.horizontalLayout.setObjectName(u"horizontalLayout")
        self.horizontalLayout.setContentsMargins(0, 0, 0, 0)
        self.drawerWrapper = QWidget(self.centralwidget)
        self.drawerWrapper.setObjectName(u"drawerWrapper")
        self.drawerWrapper.setMinimumSize(QSize(44, 0))
        self.drawerWrapper.setMaximumSize(QSize(220, 16777215))
        self.drawerLayout = QVBoxLayout(self.drawerWrapper)
        self.drawerLayout.setSpacing(0)
        self.drawerLayout.setObjectName(u"drawerLayout")
        self.drawerLayout.setContentsMargins(0, 0, 0, 0)
        self.hamburgerButton = QPushButton(self.drawerWrapper)
        self.hamburgerButton.setObjectName(u"hamburgerButton")
        self.hamburgerButton.setMinimumSize(QSize(44, 44))
        self.hamburgerButton.setMaximumSize(QSize(44, 44))

        self.drawerLayout.addWidget(self.hamburgerButton)

        self.drawerPanel = QWidget(self.drawerWrapper)
        self.drawerPanel.setObjectName(u"drawerPanel")

        self.drawerLayout.addWidget(self.drawerPanel)


        self.horizontalLayout.addWidget(self.drawerWrapper)

        self.contentStack = QStackedWidget(self.centralwidget)
        self.contentStack.setObjectName(u"contentStack")
        self.contentStack.setMinimumSize(QSize(680, 0))
        sizePolicy = QSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Expanding)
        sizePolicy.setHorizontalStretch(1)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.contentStack.sizePolicy().hasHeightForWidth())
        self.contentStack.setSizePolicy(sizePolicy)
        self.pageSimView = QWidget()
        self.pageSimView.setObjectName(u"pageSimView")
        self.contentStack.addWidget(self.pageSimView)
        self.pageTaskView = QWidget()
        self.pageTaskView.setObjectName(u"pageTaskView")
        self.contentStack.addWidget(self.pageTaskView)
        self.pageCameraView = QWidget()
        self.pageCameraView.setObjectName(u"pageCameraView")
        self.contentStack.addWidget(self.pageCameraView)
        self.pageDiagnostics = QWidget()
        self.pageDiagnostics.setObjectName(u"pageDiagnostics")
        self.contentStack.addWidget(self.pageDiagnostics)

        self.horizontalLayout.addWidget(self.contentStack)

        self.wordleColumnLayout = QVBoxLayout()
        self.wordleColumnLayout.setSpacing(0)
        self.wordleColumnLayout.setObjectName(u"wordleColumnLayout")
        self.wordleColumnLayout.setContentsMargins(0, 0, 0, 0)
        self.wordle = QWidget(self.centralwidget)
        self.wordle.setObjectName(u"wordle")
        sizePolicy1 = QSizePolicy(QSizePolicy.Policy.Preferred, QSizePolicy.Policy.Expanding)
        sizePolicy1.setHorizontalStretch(0)
        sizePolicy1.setVerticalStretch(1)
        sizePolicy1.setHeightForWidth(self.wordle.sizePolicy().hasHeightForWidth())
        self.wordle.setSizePolicy(sizePolicy1)
        self.wordle.setMinimumSize(QSize(230, 0))
        self.wordle.setStyleSheet(u"QWidget#wordle {\n"
"    background-color: #0f1117;\n"
"    border: none;\n"
"}")

        self.wordleColumnLayout.addWidget(self.wordle)

        self.voiceControls = QWidget(self.centralwidget)
        self.voiceControls.setObjectName(u"voiceControls")
        sizePolicy2 = QSizePolicy(QSizePolicy.Policy.Preferred, QSizePolicy.Policy.Fixed)
        sizePolicy2.setHorizontalStretch(0)
        sizePolicy2.setVerticalStretch(0)
        sizePolicy2.setHeightForWidth(self.voiceControls.sizePolicy().hasHeightForWidth())
        self.voiceControls.setSizePolicy(sizePolicy2)
        self.voiceControls.setMinimumSize(QSize(0, 176))
        self.voiceControls.setMaximumSize(QSize(16777215, 176))
        self.voiceControls.setStyleSheet(u"QWidget#voiceControls {\n"
"    background-color: #0f1117;\n"
"    border-top: 1px solid rgba(255, 255, 255, 0.06);\n"
"    border-bottom: 1px solid rgba(255, 255, 255, 0.06);\n"
"}\n"
"QLabel#voiceLabel {\n"
"    color: #38bdf8;\n"
"    font-size: 8pt;\n"
"    font-weight: 700;\n"
"    letter-spacing: 2px;\n"
"    background: transparent;\n"
"}\n"
"QFrame#voiceTranscriptFrame {\n"
"    background-color: #111827;\n"
"    border: 1px solid rgba(148, 163, 184, 0.18);\n"
"    border-radius: 8px;\n"
"}\n"
"QLabel#voiceTranscriptValue {\n"
"    color: #f8fafc;\n"
"    font-size: 15pt;\n"
"    font-weight: 700;\n"
"    background: transparent;\n"
"}\n"
"QPushButton#voiceRecordButton,\n"
"QPushButton#voiceStopButton,\n"
"QPushButton#voiceConfirmButton,\n"
"QPushButton#voiceRetryButton {\n"
"    border-radius: 6px;\n"
"    font-size: 9pt;\n"
"    font-weight: 700;\n"
"    padding: 8px 0px;\n"
"}\n"
"QPushButton#voiceRecordButton {\n"
"    background-color: #1f2937;\n"
"    color: #f8fafc;\n"
"    border: 1px solid rgb"
                        "a(248, 113, 113, 0.35);\n"
"}\n"
"QPushButton#voiceRecordButton:hover {\n"
"    background-color: #273244;\n"
"}\n"
"QPushButton#voiceStopButton {\n"
"    background-color: #111827;\n"
"    color: #f8fafc;\n"
"    border: 1px solid rgba(255, 255, 255, 0.16);\n"
"}\n"
"QPushButton#voiceStopButton:hover {\n"
"    background-color: #1b2432;\n"
"}\n"
"QPushButton#voiceConfirmButton {\n"
"    background-color: #0f2f20;\n"
"    color: #dcfce7;\n"
"    border: 1px solid rgba(74, 222, 128, 0.28);\n"
"}\n"
"QPushButton#voiceConfirmButton:hover {\n"
"    background-color: #16402d;\n"
"}\n"
"QPushButton#voiceRetryButton {\n"
"    background-color: #1e293b;\n"
"    color: #cbd5e1;\n"
"    border: 1px solid rgba(255, 255, 255, 0.1);\n"
"}\n"
"QPushButton#voiceRetryButton:hover {\n"
"    background-color: #273449;\n"
"}")
        self.voiceControlsLayout = QVBoxLayout(self.voiceControls)
        self.voiceControlsLayout.setSpacing(10)
        self.voiceControlsLayout.setObjectName(u"voiceControlsLayout")
        self.voiceControlsLayout.setContentsMargins(12, 12, 12, 12)
        self.voiceLabel = QLabel(self.voiceControls)
        self.voiceLabel.setObjectName(u"voiceLabel")

        self.voiceControlsLayout.addWidget(self.voiceLabel)

        self.voiceTranscriptFrame = QFrame(self.voiceControls)
        self.voiceTranscriptFrame.setObjectName(u"voiceTranscriptFrame")
        self.voiceTranscriptFrame.setFrameShape(QFrame.StyledPanel)
        self.voiceTranscriptFrame.setFrameShadow(QFrame.Raised)
        self.voiceTranscriptLayout = QVBoxLayout(self.voiceTranscriptFrame)
        self.voiceTranscriptLayout.setObjectName(u"voiceTranscriptLayout")
        self.voiceTranscriptLayout.setContentsMargins(12, 12, 12, 12)
        self.voiceTranscriptValue = QLabel(self.voiceTranscriptFrame)
        self.voiceTranscriptValue.setObjectName(u"voiceTranscriptValue")
        self.voiceTranscriptValue.setAlignment(Qt.AlignCenter)

        self.voiceTranscriptLayout.addWidget(self.voiceTranscriptValue)


        self.voiceControlsLayout.addWidget(self.voiceTranscriptFrame)

        self.voiceButtonsLayout = QGridLayout()
        self.voiceButtonsLayout.setObjectName(u"voiceButtonsLayout")
        self.voiceButtonsLayout.setHorizontalSpacing(8)
        self.voiceButtonsLayout.setVerticalSpacing(8)
        self.voiceRecordButton = QPushButton(self.voiceControls)
        self.voiceRecordButton.setObjectName(u"voiceRecordButton")

        self.voiceButtonsLayout.addWidget(self.voiceRecordButton, 0, 0, 1, 1)

        self.voiceStopButton = QPushButton(self.voiceControls)
        self.voiceStopButton.setObjectName(u"voiceStopButton")

        self.voiceButtonsLayout.addWidget(self.voiceStopButton, 0, 1, 1, 1)

        self.voiceConfirmButton = QPushButton(self.voiceControls)
        self.voiceConfirmButton.setObjectName(u"voiceConfirmButton")

        self.voiceButtonsLayout.addWidget(self.voiceConfirmButton, 1, 0, 1, 1)

        self.voiceRetryButton = QPushButton(self.voiceControls)
        self.voiceRetryButton.setObjectName(u"voiceRetryButton")

        self.voiceButtonsLayout.addWidget(self.voiceRetryButton, 1, 1, 1, 1)


        self.voiceControlsLayout.addLayout(self.voiceButtonsLayout)


        self.wordleColumnLayout.addWidget(self.voiceControls)

        self.safetyControls = QWidget(self.centralwidget)
        self.safetyControls.setObjectName(u"safetyControls")
        sizePolicy2.setHeightForWidth(self.safetyControls.sizePolicy().hasHeightForWidth())
        self.safetyControls.setSizePolicy(sizePolicy2)
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
        self.drawerWrapper.raise_()
        self.contentStack.raise_()
        self.wordle.raise_()
        self.pushButton.raise_()
        self.pushButton_2.raise_()
        self.pushButton_3.raise_()
        self.pushButton_4.raise_()

        self.retranslateUi(MainWindow)

        QMetaObject.connectSlotsByName(MainWindow)
    # setupUi

    def retranslateUi(self, MainWindow):
        MainWindow.setWindowTitle(QCoreApplication.translate("MainWindow", u"WordleBot", None))
        self.hamburgerButton.setText(QCoreApplication.translate("MainWindow", u"\u2261", None))
        self.voiceLabel.setText(QCoreApplication.translate("MainWindow", u"VOICE CONTROL", None))
        self.voiceTranscriptValue.setText(QCoreApplication.translate("MainWindow", u"Awaiting input...", None))
        self.voiceRecordButton.setText(QCoreApplication.translate("MainWindow", u"Record", None))
        self.voiceStopButton.setText(QCoreApplication.translate("MainWindow", u"Stop", None))
        self.voiceConfirmButton.setText(QCoreApplication.translate("MainWindow", u"Confirm", None))
        self.voiceRetryButton.setText(QCoreApplication.translate("MainWindow", u"Retry", None))
        self.safetyLabel.setText(QCoreApplication.translate("MainWindow", u"SAFETY CONTROLS", None))
        self.pushButton.setText(QCoreApplication.translate("MainWindow", u"START", None))
        self.pushButton_2.setText(QCoreApplication.translate("MainWindow", u"SCAN GAME BOARD", None))
        self.pushButton_3.setText(QCoreApplication.translate("MainWindow", u"HOME", None))
        self.pushButton_4.setText(QCoreApplication.translate("MainWindow", u"STOP", None))
    # retranslateUi

