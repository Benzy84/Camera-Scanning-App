import cProfile
import tkinter as tk
from tkinter import filedialog
from PyQt5.QtCore import qDebug
import json
import sys
import queue
from queue import Queue
import threading
import time
import cv2
import os
import numpy as np
import time
from threading import Event
from collections import deque
from tl_dotnet_wrapper import TL_SDK
from PyDAQmx import Task
import PyDAQmx.DAQmxConstants as DAQmxConstants
from PyQt5.QtCore import pyqtSignal, QObject, Qt, pyqtSlot, QThread
from PyQt5.QtWidgets import QApplication, QMainWindow, QPushButton, QVBoxLayout, QSlider, QHBoxLayout, \
    QGroupBox, QLineEdit, QWidget, QFrame, QFileDialog, QGridLayout, QSpacerItem, QSizePolicy, QComboBox, QLabel
from PyQt5 import QtGui
from PyQt5.QtGui import QImage, QPixmap, QFont


class Camera_Application(QMainWindow):
    new_frame_signal = pyqtSignal(object)

    def __init__(self):
        super(Camera_Application, self).__init__()
        self.create_queues()
        self.initialize_camera()
        self.initialize_galvo()
        self.initialize_ui()
        self.initialize_events()
        self.initialize_threads()
        self.connect_signals_and_slots()

    def create_queues(self):
        frames_to_stream_queue = Queue()
        captured_frames_to_save_queue = deque(maxlen=1)
        scanned_frames_to_save_queue = Queue()
        scanned_voltages_queue = Queue()
        self.queues = [frames_to_stream_queue, captured_frames_to_save_queue, scanned_frames_to_save_queue,
                       scanned_voltages_queue]

    def initialize_galvo(self):
        self.galvo = Galvo()

    def initialize_events(self):
        frame_captured_event = threading.Event()
        galvo_ready_event = threading.Event()
        self.events = [frame_captured_event, galvo_ready_event]

    def initialize_camera(self):
        self.sdk = TL_SDK()
        cameras = self.sdk.get_camera_list()
        first_camera_id = cameras[0]
        self.camera = self.sdk.open_camera(first_camera_id)
        self.camera.set_exposure_time_us(int(50000))
        self.camera.set_black_level(1023)
        self.camera.set_gain(120)
        self.camera.set_frames_per_trigger_zero_for_unlimited(0)
        taps_values = [1, 2, 4]
        for taps in reversed(taps_values):
            if self.camera.get_is_taps_supported(taps):
                self.camera.set_taps(taps)
                break
        self.camera.set_data_rate('40MHz')
        self.camera.set_roi_binning()
        self.camera.arm()
        self.camera.issue_software_trigger()
        self.camera.capturing = True
        self.camera.streaming = True

    def initialize_threads(self):
        self.image_saving_thread = image_saving_thread(self.galvo, self.queues)
        self.camera_capturing_thread = camera_capturing_thread(self.camera, self.queues, self.galvo, self.events,
                                                               self.image_saving_thread)
        self.camera_streaming_thread = camera_streaming_thread(self.camera, self.queues[0], self.image_label.width(),
                                                               self.image_label.height())
        self.camera_control_thread = camera_controlling_thread(self.camera)
        self.galvo_scanning_thread = galvo_scanning_thread(self.galvo, self.camera, self.events,
                                                           self.image_saving_thread)
        self.galvo_controlling_thread = galvo_controlling_thread(self.galvo)

        self.camera_capturing_thread.start()
        self.camera_streaming_thread.start()

    def connect_signals_and_slots(self):
        self.camera_streaming_thread.update_display_signal.connect(self.update_display)
        self.camera_capturing_thread.update_capturing_rate_signal.connect(self.update_capturing_rate)
        self.camera_streaming_thread.update_streaming_rate_signal.connect(self.update_streaming_rate)
        self.galvo_scanning_thread.update_total_points_signal.connect(self.update_total_points)
        self.galvo_scanning_thread.update_scanned_points_signal.connect(self.update_scanned_points)
        self.galvo_scanning_thread.update_remaining_points_signal.connect(self.update_remaining_points)
        self.galvo_scanning_thread.update_eta_signal.connect(self.update_eta)

        # Signals and slots for UI elements
        self.choose_folder_button.clicked.connect(self.image_saving_thread.choose_folder)
        self.open_folder_button.clicked.connect(self.image_saving_thread.open_folder)
        self.capture_button.clicked.connect(self.image_saving_thread.save_captures_frame)
        self.start_stop_button.clicked.connect(self.start_stop_camera)
        self.start_stop_scan_button.clicked.connect(self.start_stop_scan)
        self.scanning_type_combo.currentIndexChanged.connect(lambda x: self.update_scanning_type_and_ui())
        self.scanning_radius_entry.returnPressed.connect(self.update_scanning_parameters)
        self.scanning_height_entry.returnPressed.connect(self.update_scanning_parameters)
        self.scanning_width_entry.returnPressed.connect(self.update_scanning_parameters)
        self.step_size_entry.returnPressed.connect(self.update_scanning_parameters)
        self.galvo_controlling_thread.scanning_type_changed_signal.connect(
            lambda x: self.scanning_type_value_label.setText(f"Current: {x}"))
        self.galvo_controlling_thread.scanning_parameters_changed_signal.connect(
            lambda x: self.scanning_radius_value_label.setText(f"Current: {x[0]}"))
        self.galvo_controlling_thread.scanning_parameters_changed_signal.connect(
            lambda x: self.scanning_height_value_label.setText(f"Current: {x[1]}"))
        self.galvo_controlling_thread.scanning_parameters_changed_signal.connect(
            lambda x: self.scanning_width_value_label.setText(f"Current: {x[2]}"))
        self.galvo_controlling_thread.scanning_parameters_changed_signal.connect(
            lambda x: self.step_size_value_label.setText(f"Current: {x[3]}"))
        self.galvo_scanning_thread.scan_complete_signal.connect(self.update_start_stop_scan_button)
        self.data_rate_20_button.clicked.connect(lambda: self.update_data_rate("20MHz"))
        self.data_rate_40_button.clicked.connect(lambda: self.update_data_rate("40MHz"))
        self.taps_1_button.clicked.connect(lambda: self.update_taps(1))
        self.taps_2_button.clicked.connect(lambda: self.update_taps(2))
        self.taps_4_button.clicked.connect(lambda: self.update_taps(4))

        self.exposure_time_entry.returnPressed.connect(self.update_exposure_time)
        self.black_level_entry.returnPressed.connect(self.update_black_level)
        self.gain_entry.returnPressed.connect(self.update_gain)
        self.ROI_origin_X_entry.returnPressed.connect(self.update_roi_binning)
        self.ROI_origin_Y_entry.returnPressed.connect(self.update_roi_binning)
        self.ROI_width_entry.returnPressed.connect(self.update_roi_binning)
        self.ROI_height_entry.returnPressed.connect(self.update_roi_binning)
        self.ROI_bin_X_entry.returnPressed.connect(self.update_roi_binning)
        self.ROI_bin_Y_entry.returnPressed.connect(self.update_roi_binning)

        # Connect camera control thread signals to UI update slots
        self.camera_control_thread.exposure_time_changed_signal.connect(
            lambda x: self.exposure_time_value_label.setText(f"Current: {x}"))
        self.camera_control_thread.black_level_changed_signal.connect(
            lambda x: self.black_level_value_label.setText(f"Current: {x}"))
        self.camera_control_thread.gain_changed_signal.connect(lambda x: self.gain_value_label.setText(f"Current: {x}"))
        self.camera_control_thread.ROI_changed_signal.connect(
            lambda x: self.ROI_origin_X_value_label.setText(f"Current: {x[0]}"))
        self.camera_control_thread.ROI_changed_signal.connect(
            lambda x: self.ROI_origin_Y_value_label.setText(f"Current: {x[1]}"))
        self.camera_control_thread.ROI_changed_signal.connect(
            lambda x: self.ROI_width_value_label.setText(f"Current: {x[2]}"))
        self.camera_control_thread.ROI_changed_signal.connect(
            lambda x: self.ROI_height_value_label.setText(f"Current: {x[3]}"))
        self.camera_control_thread.ROI_changed_signal.connect(
            lambda x: self.ROI_bin_X_value_label.setText(f"Current: {x[4]}"))
        self.camera_control_thread.ROI_changed_signal.connect(
            lambda x: self.ROI_bin_Y_value_label.setText(f"Current: {x[5]}"))

    def initialize_ui(self):
        self.setWindowTitle("Camera App with Galvo Control")
        self.setGeometry(100, 100, 1600, 900)
        central_widget = QWidget(self)
        self.setCentralWidget(central_widget)
        main_layout = QGridLayout(central_widget)

        self.create_camera_streaming_ui(main_layout)
        self.create_camera_controlling_ui(main_layout)
        self.create_scanning_controls_ui(main_layout)
        self.create_processed_video_streaming_ui(main_layout)

    def create_ui_groupbox(self, title, layout, width=None, height=None):
        groupbox = QGroupBox(title, self)
        groupbox.setStyleSheet(STYLESHEET)
        if width:
            groupbox.setFixedWidth(width)
        if height:
            groupbox.setFixedHeight(height)
        groupbox.setLayout(layout)
        return groupbox

    def create_camera_streaming_ui(self, main_layout):
        layout = QVBoxLayout()
        self.image_label = QLabel(self)
        aspect_ratio = 3296 / 2472
        width = 640
        height = int(width / aspect_ratio)
        self.image_label.setFixedSize(width, height)
        layout.addWidget(self.image_label)
        groupbox = self.create_ui_groupbox("Camera Streaming", layout)
        font = QFont()
        font.setPointSize(16)
        groupbox.setFont(font)
        main_layout.addWidget(groupbox, 0, 0)

    def create_camera_controlling_ui(self, main_layout):
        layout = QVBoxLayout()

        # Start-Stop Control panel
        start_stop_layout = QVBoxLayout()
        self.start_stop_button = QPushButton("---", self)
        self.running_label = QLabel("---")
        self.update_start_stop_button()
        start_stop_layout.addWidget(self.start_stop_button)
        start_stop_layout.addWidget(self.running_label)
        start_stop_inner_panel = self.create_ui_groupbox("Start-Stop Control", start_stop_layout, height=80)

        layout.addItem(QSpacerItem(0, 20, QSizePolicy.Minimum, QSizePolicy.Fixed))
        layout.addWidget(start_stop_inner_panel)

        # Rate Info panel
        rate_info_layout = QVBoxLayout()
        self.capturing_rate_label = QLabel("Capturing Rate: N/A")
        self.streaming_rate_label = QLabel("Streaming Rate: N/A")
        self.processed_video_streaming_rate_label = QLabel("Processed Video Streaming Rate: N/A")
        rate_info_layout.addWidget(self.capturing_rate_label)
        rate_info_layout.addWidget(self.streaming_rate_label)
        rate_info_layout.addWidget(self.processed_video_streaming_rate_label)
        rate_info_inner_panel = self.create_ui_groupbox("Rate Info", rate_info_layout, height=100)

        layout.addWidget(rate_info_inner_panel)

        # Camera Parameters panel
        camera_parameters_layout = QGridLayout()

        # Exposure Time
        exposure_time_label = QLabel("Exposure Time (0-2000 (ms)):")
        self.exposure_time_entry = CustomLineEdit(self)
        self.exposure_time_value_label = QLabel(f"Current: {self.camera.get_exposure_time_ms()}")
        camera_parameters_layout.addWidget(exposure_time_label, 0, 0)
        camera_parameters_layout.addWidget(self.exposure_time_entry, 1, 0)
        camera_parameters_layout.addWidget(self.exposure_time_value_label, 1, 1)

        # Black level
        black_level_label = QLabel("Black level (0-1023):")
        self.black_level_entry = CustomLineEdit(self)
        self.black_level_value_label = QLabel(f"Current: {self.camera.get_black_level()}")
        camera_parameters_layout.addWidget(black_level_label, 2, 0)
        camera_parameters_layout.addWidget(self.black_level_entry, 3, 0)
        camera_parameters_layout.addWidget(self.black_level_value_label, 3, 1)

        # Gain
        gain_label = QLabel("Gain (0-1024):")
        self.gain_entry = CustomLineEdit(self)
        self.gain_value_label = QLabel(f"Current: {self.camera.get_gain()}")
        camera_parameters_layout.addWidget(gain_label, 4, 0)
        camera_parameters_layout.addWidget(self.gain_entry, 5, 0)
        camera_parameters_layout.addWidget(self.gain_value_label, 5, 1)

        camera_parameters_inner_panel = self.create_ui_groupbox("Camera Parameters", camera_parameters_layout,
                                                                height=180)
        layout.addWidget(camera_parameters_inner_panel)

        # Data Rate panel
        data_rate_layout = QHBoxLayout()
        self.data_rate_20_button = QPushButton("20MHz", self)
        self.data_rate_40_button = QPushButton("40MHz", self)
        data_rate_layout.addWidget(self.data_rate_20_button)
        data_rate_layout.addWidget(self.data_rate_40_button)
        data_rate_inner_panel = self.create_ui_groupbox("Data Rate", data_rate_layout, height=60)
        layout.addWidget(data_rate_inner_panel)
        self.update_data_rate()

        # Taps panel
        taps_layout = QHBoxLayout()
        self.taps_1_button = QPushButton("1", self)
        self.taps_2_button = QPushButton("2", self)
        self.taps_4_button = QPushButton("4", self)
        taps_layout.addWidget(self.taps_1_button)
        taps_layout.addWidget(self.taps_2_button)
        taps_layout.addWidget(self.taps_4_button)
        taps_inner_panel = self.create_ui_groupbox("Taps", taps_layout, height=60)
        layout.addWidget(taps_inner_panel)
        self.update_taps()

        # ROI and Binning panel
        ROI_and_Binning_layout = QGridLayout()

        # ROI Origin - X
        ROI_origin_X_label = QLabel("ROI Origin - X (0-3295)")
        self.ROI_origin_X_entry = CustomLineEdit(self)
        self.ROI_origin_X_value_label = QLabel(f"Current: {self.camera.get_roi_binning()[0]}")
        ROI_and_Binning_layout.addWidget(ROI_origin_X_label, 0, 0)
        ROI_and_Binning_layout.addWidget(self.ROI_origin_X_entry, 1, 0)
        ROI_and_Binning_layout.addWidget(self.ROI_origin_X_value_label, 1, 1)

        # ROI Origin - Y
        ROI_origin_Y_label = QLabel("ROI Origin - Y (0-2471)")
        self.ROI_origin_Y_entry = CustomLineEdit(self)
        self.ROI_origin_Y_value_label = QLabel(f"Current: {self.camera.get_roi_binning()[1]}")
        ROI_and_Binning_layout.addWidget(ROI_origin_Y_label, 2, 0)
        ROI_and_Binning_layout.addWidget(self.ROI_origin_Y_entry, 3, 0)
        ROI_and_Binning_layout.addWidget(self.ROI_origin_Y_value_label, 3, 1)

        # ROI Width
        ROI_width_label = QLabel("ROI Width (1-3296)")
        self.ROI_width_entry = CustomLineEdit(self)
        self.ROI_width_value_label = QLabel(f"Current: {self.camera.get_roi_binning()[2]}")
        ROI_and_Binning_layout.addWidget(ROI_width_label, 4, 0)
        ROI_and_Binning_layout.addWidget(self.ROI_width_entry, 5, 0)
        ROI_and_Binning_layout.addWidget(self.ROI_width_value_label, 5, 1)

        # ROI Height
        ROI_height_label = QLabel("ROI Height (1-2472)")
        self.ROI_height_entry = CustomLineEdit(self)
        self.ROI_height_value_label = QLabel(f"Current: {self.camera.get_roi_binning()[3]}")
        ROI_and_Binning_layout.addWidget(ROI_height_label, 6, 0)
        ROI_and_Binning_layout.addWidget(self.ROI_height_entry, 7, 0)
        ROI_and_Binning_layout.addWidget(self.ROI_height_value_label, 7, 1)

        # Bin - X
        ROI_bin_X_label = QLabel("Bin - X (1-24)")
        self.ROI_bin_X_entry = CustomLineEdit(self)
        self.ROI_bin_X_value_label = QLabel(f"Current: {self.camera.get_roi_binning()[4]}")
        ROI_and_Binning_layout.addWidget(ROI_bin_X_label, 9, 0)
        ROI_and_Binning_layout.addWidget(self.ROI_bin_X_entry, 10, 0)
        ROI_and_Binning_layout.addWidget(self.ROI_bin_X_value_label, 10, 1)

        # Bin - Y
        ROI_bin_Y_label = QLabel("Bin - Y (1-24)")
        self.ROI_bin_Y_entry = CustomLineEdit(self)
        self.ROI_bin_Y_value_label = QLabel(f"Current: {self.camera.get_roi_binning()[5]}")
        ROI_and_Binning_layout.addWidget(ROI_bin_Y_label, 11, 0)
        ROI_and_Binning_layout.addWidget(self.ROI_bin_Y_entry, 12, 0)
        ROI_and_Binning_layout.addWidget(self.ROI_bin_Y_value_label, 12, 1)

        ROI_and_Binning_inner_panel = self.create_ui_groupbox("Camera Parameters", ROI_and_Binning_layout,
                                                              height=300)
        layout.addWidget(ROI_and_Binning_inner_panel)

        # Create the main camera controlling panel
        groupbox = self.create_ui_groupbox("Camera Controlling", layout)
        font = QFont()
        font.setPointSize(16)
        groupbox.setFont(font)
        main_layout.addWidget(groupbox, 0, 1)

    def create_scanning_controls_ui(self, main_layout):
        layout = QVBoxLayout()

        # Start-Stop Scan inner panel
        start_stop_scan_layout = QVBoxLayout()
        self.start_stop_scan_button = QPushButton("---", self)
        self.scanning_label = QLabel("---")
        self.update_start_stop_scan_button()
        start_stop_scan_layout.addWidget(self.start_stop_scan_button)
        start_stop_scan_layout.addWidget(self.scanning_label)
        start_stop_scan_inner_panel = self.create_ui_groupbox("Start-Stop Control", start_stop_scan_layout, height=80)
        layout.addWidget(start_stop_scan_inner_panel)

        # Scanning Parameters inner panel
        scanning_parameters_layout = QGridLayout()

        # Create the QComboBox (dropdown) for selecting scan types
        self.scanning_type_label = QLabel("Scan Type:")
        self.scanning_type_combo = QComboBox()
        self.scanning_type_combo.addItem("Circular")
        self.scanning_type_combo.addItem("Rectangular")
        self.scanning_type_value_label = QLabel(f"Current: {self.galvo.get_scanning_type()}")
        scanning_parameters_layout.addWidget(self.scanning_type_label, 0, 0)
        scanning_parameters_layout.addWidget(self.scanning_type_combo, 1, 0)
        scanning_parameters_layout.addWidget(self.scanning_type_value_label, 1, 1)

        # Scanning Radius
        scanning_radius_label = QLabel("Scanning Radius")
        self.scanning_radius_entry = CustomLineEdit(self)
        self.scanning_radius_value_label = QLabel(f"Current: {self.galvo.get_scanning_parameters()[0]}")
        scanning_parameters_layout.addWidget(scanning_radius_label, 2, 0)
        scanning_parameters_layout.addWidget(self.scanning_radius_entry, 3, 0)
        scanning_parameters_layout.addWidget(self.scanning_radius_value_label, 3, 1)

        # Scanning height
        scanning_height_label = QLabel("scanning_height")
        self.scanning_height_entry = CustomLineEdit(self)
        self.scanning_height_entry.setEnabled(False)
        self.scanning_height_value_label = QLabel(f"Current: {self.galvo.get_scanning_parameters()[1]}")
        scanning_parameters_layout.addWidget(scanning_height_label, 4, 0)
        scanning_parameters_layout.addWidget(self.scanning_height_entry, 5, 0)
        scanning_parameters_layout.addWidget(self.scanning_height_value_label, 5, 1)

        # Scanning Width
        scanning_width_label = QLabel("scanning width")
        self.scanning_width_entry = CustomLineEdit(self)
        self.scanning_width_entry.setEnabled(False)
        self.scanning_width_value_label = QLabel(f"Current: {self.galvo.get_scanning_parameters()[2]}")
        scanning_parameters_layout.addWidget(scanning_width_label, 6, 0)
        scanning_parameters_layout.addWidget(self.scanning_width_entry, 7, 0)
        scanning_parameters_layout.addWidget(self.scanning_width_value_label, 7, 1)

        # Step Size
        step_size_label = QLabel("Step Size")
        self.step_size_entry = CustomLineEdit(self)
        self.step_size_value_label = QLabel(f"Current: {self.galvo.get_scanning_parameters()[3]}")
        scanning_parameters_layout.addWidget(step_size_label, 8, 0)
        scanning_parameters_layout.addWidget(self.step_size_entry, 9, 0)
        scanning_parameters_layout.addWidget(self.step_size_value_label, 9, 1)

        scanning_parameters_inner_panel = self.create_ui_groupbox("Scanning Parameters", scanning_parameters_layout,
                                                                  height=240)
        layout.addWidget(scanning_parameters_inner_panel)

        # Scanning Info inner panel
        scanning_info_layout = QGridLayout()

        # Total Points to Scan
        self.total_points_value_label = QLabel(f"Total Points: ")
        scanning_info_layout.addWidget(self.total_points_value_label)

        # Scanned Points
        self.scanned_points_value_label = QLabel(f"Scanned Points: ")
        scanning_info_layout.addWidget(self.scanned_points_value_label)

        # Remaining Points
        self.remaining_points_value_label = QLabel(f"Remaining Points: ")
        scanning_info_layout.addWidget(self.remaining_points_value_label)

        # ETA
        self.eta_value_label = QLabel(f"ETA: ")
        scanning_info_layout.addWidget(self.eta_value_label)

        # add data within inner panel
        scanning_info_inner_panel = self.create_ui_groupbox("Scanning Info", scanning_info_layout, height=160)
        layout.addWidget(scanning_info_inner_panel)

        # Capture and open folder buttons
        capturing_layout = QGridLayout()
        self.capture_button = QPushButton("Capture Frame", self)
        self.choose_folder_button = QPushButton("Choose Folder", self)
        self.open_folder_button = QPushButton("Open Folder", self)
        capturing_layout.addWidget(self.capture_button)
        capturing_layout.addWidget(self.choose_folder_button)
        capturing_layout.addWidget(self.open_folder_button)
        Capture_inner_panel = self.create_ui_groupbox("Capture", capturing_layout, height=120)
        layout.addWidget(Capture_inner_panel)

        # Complete the layout
        groupbox = self.create_ui_groupbox("Scanning Controls", layout)
        font = QFont()
        font.setPointSize(16)
        groupbox.setFont(font)
        main_layout.addWidget(groupbox, 0, 2)

    def create_processed_video_streaming_ui(self, main_layout):
        layout = QVBoxLayout()
        groupbox = self.create_ui_groupbox("Processed Video Streaming", layout)
        font = QFont()
        font.setPointSize(16)
        groupbox.setFont(font)
        main_layout.addWidget(groupbox, 0, 3)

    @pyqtSlot()
    def update_exposure_time(self):
        new_exposure_time = int(self.exposure_time_entry.text())
        if self.camera_capturing_thread.camera.capturing:
            self.stop_camera()
            self.camera_control_thread.set_exposure_time(new_exposure_time)
            self.start_camera()
        else:
            self.camera_control_thread.set_exposure_time(new_exposure_time)

    @pyqtSlot()
    def update_roi_binning(self):
        new_roi_binning = []
        entries = [
            self.ROI_origin_X_entry.text(),
            self.ROI_origin_Y_entry.text(),
            self.ROI_width_entry.text(),
            self.ROI_height_entry.text(),
            self.ROI_bin_X_entry.text(),
            self.ROI_bin_Y_entry.text()
        ]
        default_values = [0, 0, 3296, 2472, 1, 1]
        idx = 0
        for text_value in entries:
            if text_value:
                new_roi_binning.append(int(text_value))
                idx += 1
            else:
                new_roi_binning.append(default_values[idx])
                idx += 1

        if self.camera_capturing_thread.camera.capturing:
            self.stop_camera()
            self.camera_control_thread.set_roi_binning(new_roi_binning)
            self.start_camera()
        else:
            self.camera_control_thread.set_roi_binning(new_roi_binning)

    @pyqtSlot()
    def update_scanning_type_and_ui(self):
        scanning_type = self.scanning_type_combo.currentText()
        if self.galvo_scanning_thread.galvo.scanning:
            self.stop_scan()
            self.galvo_controlling_thread.set_scanning_type(scanning_type)
            self.start_scan()
        else:
            self.galvo_controlling_thread.set_scanning_type(scanning_type)
        # Update the UI
        if scanning_type == "Circular":
            self.scanning_radius_entry.setEnabled(True)
            self.step_size_entry.setEnabled(True)
            self.scanning_width_entry.setEnabled(False)
            self.scanning_height_entry.setEnabled(False)
        elif scanning_type == "Rectangular":
            self.scanning_radius_entry.setEnabled(False)
            self.step_size_entry.setEnabled(True)
            self.scanning_width_entry.setEnabled(True)
            self.scanning_height_entry.setEnabled(True)

    @pyqtSlot()
    def update_scanning_parameters(self):
        new_scanning_parameters = []
        entries = [
            self.scanning_radius_entry.text(),
            self.scanning_height_entry.text(),
            self.scanning_width_entry.text(),
            self.step_size_entry.text()
        ]
        default_values = [0.045, 0.045, 0.045, 0.01]
        idx = 0
        for text_value in entries:
            if text_value:
                new_scanning_parameters.append(float(text_value))
                idx += 1
            else:
                new_scanning_parameters.append(default_values[idx])
                idx += 1

        if self.galvo_scanning_thread.galvo.scanning:
            self.stop_scan()
            self.galvo_controlling_thread.set_scanning_parameters(new_scanning_parameters)
            self.start_scan()
        else:
            self.galvo_controlling_thread.set_scanning_parameters(new_scanning_parameters)

    @pyqtSlot()
    def update_black_level(self):
        new_black_level = int(self.black_level_entry.text())
        if self.camera_capturing_thread.camera.capturing:
            self.stop_camera()
            self.camera_control_thread.set_black_level(new_black_level)
            self.start_camera()
        else:
            self.camera_control_thread.set_black_level(new_black_level)

    @pyqtSlot()
    def update_gain(self):
        new_gain = int(self.gain_entry.text())
        if self.camera_capturing_thread.camera.capturing:
            self.stop_camera()
            self.camera_control_thread.set_gain(new_gain)
            self.start_camera()
        else:
            self.camera_control_thread.set_gain(new_gain)

    @pyqtSlot()
    def update_data_rate(self, new_data_rate=None):
        if new_data_rate:
            if self.camera_capturing_thread.camera.capturing:
                self.stop_camera()
                self.camera_control_thread.set_data_rate(new_data_rate)
                self.start_camera()
            else:
                self.camera_control_thread.set_data_rate(new_data_rate)

        current_data_rate = self.camera.get_data_rate()
        self.data_rate_20_button.setStyleSheet("font-size: 20px; height: 38px")
        self.data_rate_40_button.setStyleSheet("font-size: 20px; height: 38px")
        if current_data_rate == "20MHz":
            self.data_rate_20_button.setStyleSheet(
                "font-size: 20px; height: 38px; width: 78px; border: 2px solid green;")
        elif current_data_rate == "40MHz":
            self.data_rate_40_button.setStyleSheet(
                "font-size: 20px; height: 38px; width: 78px; border: 2px solid green;")

    @pyqtSlot()
    def update_taps(self, new_taps=None):
        if new_taps:
            if self.camera_capturing_thread.camera.capturing:
                self.stop_camera()
                self.camera_control_thread.set_taps(new_taps)
                self.start_camera()
            else:
                self.camera_control_thread.set_taps(new_taps)

        # Highlight the selected button
        current_taps_value = self.camera.get_taps()
        self.taps_1_button.setStyleSheet("font-size: 20px")
        self.taps_2_button.setStyleSheet("font-size: 20px")
        self.taps_4_button.setStyleSheet("font-size: 20px")
        if current_taps_value == 1:
            self.taps_1_button.setStyleSheet("font-size: 20px; height: 38px; width: 78px; border: 2px solid green;")
        elif current_taps_value == 2:
            self.taps_2_button.setStyleSheet("font-size: 20px; height: 38px; width: 78px; border: 2px solid green;")
        elif current_taps_value == 4:
            self.taps_4_button.setStyleSheet("font-size: 20px; height: 38px; width: 78px; border: 2px solid green;")

    def update_capturing_rate(self, rate):
        self.capturing_rate_label.setText(f"Capturing Rate: {rate:.2f} FPS")

    def update_streaming_rate(self, rate):
        self.streaming_rate_label.setText(f"Streaming Rate: {rate:.2f} FPS")

    def update_total_points(self, total_points):
        self.total_points_value_label.setText(f"Total Points: {total_points}")

    def update_scanned_points(self, scanned_points):
        self.scanned_points_value_label.setText(f"Scanned Points: {scanned_points}")

    def update_remaining_points(self, remaining_points):
        self.remaining_points_value_label.setText(f"Remaining Roints: {remaining_points}")

    def update_eta(self, eta):
        self.eta_value_label.setText(f" ETA: {eta:.2f} sec")

    def start_stop_camera(self):
        if self.camera.capturing:
            self.stop_camera()
        else:
            self.start_camera()
        self.update_start_stop_button()

    def start_camera(self):
        # print("Starting camera")
        self.camera.arm()
        self.camera.issue_software_trigger()
        self.camera.capturing = True
        self.camera.streaming = True
        # Restart the threads if they've exited
        if not self.camera_capturing_thread.isRunning():
            self.camera_capturing_thread.start()
        if not self.camera_streaming_thread.isRunning():
            self.camera_streaming_thread.start()

    def stop_camera(self):
        # print("Stopping camera")
        self.camera_streaming_thread.stop()
        self.camera_capturing_thread.stop()
        self.camera.disarm()
        # Stop the threads if they are still running:
        if self.camera_capturing_thread.isRunning():
            self.camera_capturing_thread.stop()
        if self.camera_streaming_thread.isRunning():
            self.camera_streaming_thread.stop()

    def update_start_stop_button(self):
        if self.camera.capturing:
            self.start_stop_button.setText("Stop")
            self.running_label.setText("Running...")
            self.running_label.setStyleSheet("background-color: green; color: white; padding: 5px;")
        else:
            self.start_stop_button.setText("Start")
            self.running_label.setText("Not Running")
            self.running_label.setStyleSheet("background-color: red; color: white; padding: 5px;")

    def start_stop_scan(self):
        if self.galvo.scanning:
            self.stop_scan()
        else:
            self.start_scan()
        self.update_start_stop_scan_button()

    def start_scan(self):
        # print("Starting scan")
        self.galvo.scanning = True
        self.galvo.scan_stopped = False
        self.galvo.scan_complete = False
        if not self.galvo_scanning_thread.isRunning():
            self.galvo_scanning_thread.start()

    def stop_scan(self):
        # print("Stopping scan")
        self.galvo_scanning_thread.stop()

    def update_start_stop_scan_button(self):
        if self.galvo.scanning:
            self.start_stop_scan_button.setText("Stop")
            self.scanning_label.setText("Running...")
            self.scanning_label.setStyleSheet("background-color: green; color: white; padding: 5px;")
        else:
            if self.galvo.scan_stopped:
                self.start_stop_scan_button.setText("Start")
                self.scanning_label.setText("Scan Stopped")
                self.scanning_label.setStyleSheet("background-color: orange; color: white; padding: 5px;")
            elif self.galvo.scan_complete:
                self.start_stop_scan_button.setText("Start")
                self.scanning_label.setText("Scan Complete")
                self.scanning_label.setStyleSheet("background-color: blue; color: white; padding: 5px;")
            else:
                self.start_stop_scan_button.setText("Start")
                self.scanning_label.setText("Not Running")
                self.scanning_label.setStyleSheet("background-color: red; color: white; padding: 5px;")

    @pyqtSlot(QPixmap)
    def update_display(self, pixmap):
        self.image_label.setPixmap(pixmap)


class camera_capturing_thread(QThread):
    update_capturing_rate_signal = pyqtSignal(float)

    def __init__(self, camera, queues, galvo, events, image_saving_thread):
        super(camera_capturing_thread, self).__init__()
        self.camera = camera
        self.image_saving_thread = image_saving_thread
        self.frames_to_stream_queue = queues[0]
        self.captured_frames_to_save_queue = queues[1]
        self.scanned_frames_to_save_queue = queues[2]
        self.scanned_voltages_queue = queues[3]
        self.frame_counter = 0
        self.start_time = time.time()
        self.galvo = galvo
        self.frame_captured_event = events[0]
        self.galvo_ready_event = events[1]
        self.current_voltages = []

    def run(self):
        scanning = False
        print("capturing Thread started")
        timestamps = deque()
        while self.camera.capturing:
            counter = 0
            if self.galvo.scanning:
                print('waiting for galvo')
                self.galvo_ready_event.wait()
                self.camera.issue_software_trigger()
                if not scanning:
                    time.sleep(1)
                    scanning = True
                    frame = self.camera.get_pending_frame_or_null()
                    self.camera.issue_software_trigger()
            else:
                scanning = False
            current_time = time.time()
            frame = self.camera.get_pending_frame_or_null()
            start_time = time.time()
            while frame is None:
                frame = self.camera.get_pending_frame_or_null()
                counter += 1
            try:
                if scanning:
                    self.current_voltages = self.galvo.current_voltages
                    self.frame_captured_event.set()
                    self.galvo_ready_event.clear()
                    print("Frame captured")
                image_array = self.camera.frame_to_array(frame)
                image_array_8bit = np.right_shift(image_array, 8).astype(np.uint8)
                image_rgb = cv2.cvtColor(image_array_8bit, cv2.COLOR_BGR2RGB)
                self.frames_to_stream_queue.put(image_rgb)
                self.captured_frames_to_save_queue.append(image_rgb)
                if scanning:
                    self.scanned_frames_to_save_queue.put(image_rgb)
                    self.scanned_voltages_queue.put(self.current_voltages)
                    if not self.image_saving_thread.active:
                        self.image_saving_thread.start()
                end_time = time.time()
                queue_size = self.frames_to_stream_queue.qsize()
                # print(f"Current queue size(in capturing): {queue_size}")
                # print(f'frame captured on try number {counter}')
                self.frame_counter += 1

                timestamps.append(current_time)

                # Remove timestamps older than 4 seconds
                while timestamps and current_time - timestamps[0] > 1:
                    timestamps.popleft()

                # Calculate the rate based on the last x seconds
                rate = len(timestamps) / 1.0
                self.update_capturing_rate_signal.emit(rate)


            except Exception as e:
                print(f"Exception occurred: {e}")
            # print(f"Time taken for one capturing iteration: {end_time - start_time} seconds")
        print("capturing Thread ended")

    def stop(self):
        self.camera.capturing = False


class camera_streaming_thread(QThread):
    update_display_signal = pyqtSignal(QPixmap)
    update_streaming_rate_signal = pyqtSignal(float)

    def __init__(self, camera, frames_to_stream_queue, label_width, label_height):
        super(camera_streaming_thread, self).__init__()
        self.camera = camera
        self.frames_to_stream_queue = frames_to_stream_queue
        self.frame_counter = 0
        self.start_time = time.time()
        self.label_width = label_width
        self.label_height = label_height

    def run(self):
        print("Streaming Thread started")
        timestamps = deque()
        while self.camera.streaming:
            frame = None
            start_time = time.time()
            current_time = time.time()
            frame = self.frames_to_stream_queue.get()
            end_time = time.time()
            queue_size = self.frames_to_stream_queue.qsize()
            # print(f"Current queue size: {queue_size}")
            if frame is None:
                break
            self.display_image(frame)
            self.frame_counter += 1
            timestamps.append(current_time)

            # Remove timestamps older than x seconds
            while timestamps and current_time - timestamps[0] > 1:
                timestamps.popleft()

            # Calculate the rate based on the last x seconds
            rate = len(timestamps) / 1
            self.update_streaming_rate_signal.emit(rate)
            # print(f"Time taken for one streaming iteration: {end_time - start_time} seconds")
        print("Streaming Thread ended")

    def stop(self):
        self.camera.streaming = False

    def display_image(self, image_array):
        original_height, original_width, _ = image_array.shape

        width_scale = self.label_width / original_width
        height_scale = self.label_height / original_height

        # Choose the smaller of the two scaling factors, but not greater than 1
        scale_factor = min(min(width_scale, height_scale), 1)

        # Calculate new dimensions
        new_width = int(original_width * scale_factor)
        new_height = int(original_height * scale_factor)
        if new_width == 0:
            new_width = 1
        elif new_height == 0:
            new_height = 1

        resized_image = cv2.resize(image_array, (new_width, new_height), interpolation=cv2.INTER_NEAREST)

        # Convert to QImage and display
        height, width, channel = resized_image.shape
        bytes_per_line = 3 * width
        q_img = QImage(resized_image.data, width, height, bytes_per_line, QImage.Format_RGB888)
        pixmap = QPixmap.fromImage(q_img)
        self.update_display_signal.emit(pixmap)
        # print('frame displayed')


class camera_controlling_thread(QThread):
    # Define signals to update the UI
    exposure_time_changed_signal = pyqtSignal(int)
    black_level_changed_signal = pyqtSignal(int)
    gain_changed_signal = pyqtSignal(int)
    data_rate_changed_signal = pyqtSignal(str)
    taps_changed_signal = pyqtSignal(int)
    ROI_changed_signal = pyqtSignal(list)

    def __init__(self, camera):
        super(camera_controlling_thread, self).__init__()
        self.camera = camera

    @pyqtSlot(int)
    def set_exposure_time(self, new_exposure_time_ms):
        exposure_time_us = 1000 * int(new_exposure_time_ms)
        self.camera.set_exposure_time_us(exposure_time_us)
        self.exposure_time_changed_signal.emit(new_exposure_time_ms)
        # print(f"Exposure time set to {new_exposure_time_ms} msec")

    @pyqtSlot(int)
    def set_black_level(self, new_black_level):
        self.camera.set_black_level(new_black_level)
        self.black_level_changed_signal.emit(new_black_level)
        # print(f"Black level set to {new_black_level}")

    @pyqtSlot(int)
    def set_gain(self, new_gain):
        self.camera.set_gain(new_gain)
        self.gain_changed_signal.emit(new_gain)
        # print(f"Gain set to {new_gain}")

    @pyqtSlot(int)
    def set_roi_binning(self, new_roi_binning):
        origin_X = new_roi_binning[0]
        origin_Y = new_roi_binning[1]
        width_pixels = new_roi_binning[2]
        height_pixels = new_roi_binning[3]
        binX = new_roi_binning[4]
        binY = new_roi_binning[5]
        self.camera.set_roi_binning(origin_X, origin_Y, width_pixels, height_pixels, binX, binY)
        self.ROI_changed_signal.emit(new_roi_binning)
        # print(f"Roi and binning set to {new_roi_binning} msec")

    @pyqtSlot(str)
    def set_data_rate(self, new_data_rate):
        self.camera.set_data_rate(new_data_rate)
        self.data_rate_changed_signal.emit(new_data_rate)
        # print(f"Data rate set to {new_data_rate}")

    @pyqtSlot(str)
    def set_taps(self, new_taps):
        self.camera.set_taps(new_taps)
        self.taps_changed_signal.emit(new_taps)
        # print(f"Taps set to {new_taps}")


class galvo_scanning_thread(QThread):
    scan_complete_signal = pyqtSignal()
    update_total_points_signal = pyqtSignal(int)
    update_scanned_points_signal = pyqtSignal(int)
    update_remaining_points_signal = pyqtSignal(int)
    update_eta_signal = pyqtSignal(float)

    def __init__(self, galvo, camera, events, image_saving_thread):
        super().__init__()
        self.galvo = galvo
        self.camera = camera
        self.frame_captured_event = events[0]
        self.galvo_ready_event = events[1]
        self.total_points_to_scan = []
        self.image_saving_thread = image_saving_thread  # Create an instance
        self.galvo.current_voltages = []

        # Coefficients to convert from pixel shift to voltage
        self.a, self.b, self.p, self.q = 1054.7636044909243, 3816.2346252097523, -4149.638962731514, 973.4974079819152

    def set_coefficients(self, a, b, p, q):
        """
        Set the coefficients for voltage calculations.

        Assumption:
        We assume that the relationship between pixel shift and required voltage
        is linear. The coefficients a, b, p, q define this linear relationship.
        """
        self.a, self.b, self.p, self.q = a, b, p, q

    def pixel_to_voltage(self, delta_pixel_x, delta_pixel_y):
        """
        Convert pixel shift to corresponding voltage based on the defined coefficients.

        Assumption:
        We assume the relationship between pixel shifts (in x and y) and the required
        voltages (V_x and V_y) is given by the equations:
        delta_pixel_x = a * delta_vx + b * delta_vy
        delta_pixel_y = p * delta_vx + q * delta_vy
        """

        # Create the coefficient matrix
        coeff_matrix = np.array([[self.a, self.b], [self.p, self.q]])

        # Create the pixel shifts matrix
        pixel_shifts = np.array([delta_pixel_x, delta_pixel_y])

        # Solve for the voltages
        delta_voltages = np.linalg.solve(coeff_matrix, pixel_shifts)

        return delta_voltages[0], delta_voltages[1]

    def calculate_total_points_to_scan(self):
        scanning_radius = self.galvo.scanning_radius
        scanning_height = self.galvo.scanning_height
        scanning_width = self.galvo.scanning_width
        step_size = self.galvo.step_size
        total_points = 0
        if self.galvo.scanning_type == 'Circular':
            x_range = np.arange(-scanning_radius, scanning_radius + step_size, step_size)
            y_range = np.arange(-scanning_radius, scanning_radius + step_size, step_size)
            num_of_decimal_points = len(str(step_size).split('.')[1]) + 2
            threshold = 10 ^ (-num_of_decimal_points)
            x_range = np.round(x_range, num_of_decimal_points)
            y_range = np.round(y_range, num_of_decimal_points)
            x_range[np.abs(x_range) < threshold] = 0
            y_range[np.abs(y_range) < threshold] = 0
            for x in x_range:
                for y in y_range:
                    if x ** 2 + y ** 2 <= scanning_radius ** 2:
                        total_points += 1
        elif self.galvo.scanning_type == 'Rectangular':
            x_range = np.arange(-0.5 * scanning_width, 0.5 * (scanning_width + step_size), step_size)
            y_range = np.arange(-0.5 * scanning_height, 0.5 * (scanning_height + step_size), step_size)
            num_of_decimal_points = len(str(step_size).split('.')[1]) + 2
            threshold = 10 ^ (-num_of_decimal_points)
            x_range = np.round(x_range, num_of_decimal_points)
            y_range = np.round(y_range, num_of_decimal_points)
            x_range[np.abs(x_range) < threshold] = 0
            y_range[np.abs(y_range) < threshold] = 0
            total_points = len(x_range) * len(y_range)
        return total_points

    def run(self):
        self.camera.set_frames_per_trigger_zero_for_unlimited(1)
        scanning_radius = self.galvo.scanning_radius
        scanning_height = self.galvo.scanning_height
        scanning_width = self.galvo.scanning_width
        step_size = self.galvo.step_size
        scanned_points = 0
        last_n_times = deque(maxlen=10)
        start_time_for_this_point = None
        self.total_points_to_scan = self.calculate_total_points_to_scan()
        self.image_saving_thread.total_points_to_scan = self.total_points_to_scan
        self.update_total_points_signal.emit(self.total_points_to_scan)

        # Instead of using voltages directly, we first define our scanning pattern in terms of target pixel positions.
        # We then convert these target pixel positions to the required voltages.
        if self.galvo.scanning_type == 'Circular':
            # For a circular scan, we define a pixel_radius and calculate the set of pixels that fall within this radius.
            pixel_radius = scanning_radius  # Define the radius in terms of pixels
            x_pixels = np.arange(-pixel_radius, pixel_radius + step_size, step_size)  # All possible x pixel positions
            y_pixels = np.arange(-pixel_radius, pixel_radius + step_size, step_size)  # All possible y pixel positions


            # Calculate the number of decimal places in the step size.
            # This is used to address potential floating-point inaccuracies.
            num_of_decimal_points = len(str(step_size).split('.')[1]) + 2

            # Define a threshold based on the number of decimal places.
            # Any value below this threshold is essentially considered as zero.
            threshold = 10 ** (-num_of_decimal_points)

            # Round the values in x_range and y_range to ensure precision.
            x_pixels = np.round(x_pixels, num_of_decimal_points)
            y_pixels = np.round(y_pixels, num_of_decimal_points)

            # Set very small values (below the threshold) to be exactly zero.
            x_pixels[np.abs(x_pixels) < threshold] = 0
            y_pixels[np.abs(y_pixels) < threshold] = 0

            for x_pixel in x_pixels:
                for y_pixel in y_pixels:
                    # For each pixel position, we check if it falls within the defined circle.
                    if x_pixel ** 2 + y_pixel ** 2 <= pixel_radius ** 2 and self.galvo.scanning:
                        # Convert the desired pixel shift to the required voltages
                        x_voltage, y_voltage = self.pixel_to_voltage(x_pixel, y_pixel)
                        start_time_for_previous_point = start_time_for_this_point
                        start_time_for_this_point = time.time()
                        data = np.array([x_voltage, y_voltage], dtype=np.float64)
                        self.galvo.task.WriteAnalogF64(1, True, 10.0, DAQmxConstants.DAQmx_Val_GroupByChannel, data,
                                                       None, None)
                        time.sleep(0.01)
                        self.galvo.current_voltages = [x_voltage, y_voltage]
                        if self.camera.capturing:
                            self.galvo_ready_event.set()
                            print("Galvo ready")
                            print('waiting for frame to be captured')
                            self.frame_captured_event.wait()
                            self.frame_captured_event.clear()

                        # Update counters and signals
                        if start_time_for_previous_point is not None:
                            last_n_times.append(start_time_for_this_point - start_time_for_previous_point)
                        scanned_points += 1
                        self.update_scanned_points_signal.emit(scanned_points)
                        self.update_remaining_points_signal.emit(self.total_points_to_scan - scanned_points)

                        # Calculate and update ETA
                        if last_n_times:
                            average_time_per_point = np.mean(last_n_times)
                            eta = average_time_per_point * (self.total_points_to_scan - scanned_points)
                            self.update_eta_signal.emit(eta)
        elif self.galvo.scanning_type == 'Rectangular':
            pixel_width = scanning_width  # Define the width in terms of pixels
            pixel_height = scanning_height  # Define the height in terms of pixels
            x_pixels = np.arange(-0.5 * pixel_width, 0.5 * (pixel_width + step_size), step_size)
            y_pixels = np.arange(-0.5 * pixel_height, 0.5 * (pixel_height + step_size), step_size)

            # Similarly, for the rectangular scan, we handle potential floating-point inaccuracies.
            num_of_decimal_points = len(str(step_size).split('.')[1]) + 2
            threshold = 10 ** (-num_of_decimal_points)
            x_pixels = np.round(x_pixels, num_of_decimal_points)
            y_pixels = np.round(y_pixels, num_of_decimal_points)
            x_pixels[np.abs(x_pixels) < threshold] = 0
            y_pixels[np.abs(y_pixels) < threshold] = 0

            for x_pixel in x_pixels:
                for y_pixel in y_pixels:
                    # Convert the desired pixel shift to the required voltages
                    x_voltage, y_voltage = self.pixel_to_voltage(x_pixel, y_pixel)
                    if self.galvo.scanning:
                        start_time_for_previous_point = start_time_for_this_point
                        start_time_for_this_point = time.time()
                        data = np.array([x_voltage, y_voltage], dtype=np.float64)
                        self.galvo.task.WriteAnalogF64(1, True, 10.0, DAQmxConstants.DAQmx_Val_GroupByChannel, data,
                                                       None, None)
                        time.sleep(0.01)
                        self.galvo.current_voltages = [x_voltage, y_voltage]
                        if self.camera.capturing:
                            self.galvo_ready_event.set()
                            print("Galvo ready")
                            print('waiting for frame to be captured')
                            self.frame_captured_event.wait()
                            self.frame_captured_event.clear()
                        # Update counters and signals
                        if start_time_for_previous_point is not None:
                            last_n_times.append(start_time_for_this_point - start_time_for_previous_point)
                        scanned_points += 1
                        print(f'in scanning thread, point no. {scanned_points}, volt:{x_voltage} and {y_voltage}')
                        self.update_scanned_points_signal.emit(scanned_points)
                        self.update_remaining_points_signal.emit(self.total_points_to_scan - scanned_points)

                        # Calculate and update ETA
                        if last_n_times:
                            average_time_per_point = np.mean(last_n_times)
                            eta = average_time_per_point * (self.total_points_to_scan - scanned_points)
                            self.update_eta_signal.emit(eta)
        if not self.galvo.scan_stopped:
            self.galvo.scan_complete = True
            self.galvo.scanning = False
            self.scan_complete_signal.emit()
            self.galvo.go_to_zero_pos()

        self.camera.set_frames_per_trigger_zero_for_unlimited(0)
        self.camera.issue_software_trigger()

    def stop(self):
        self.galvo.scanning = False
        self.galvo.scan_stopped = True
        self.galvo.go_to_zero_pos()
        self.camera.set_frames_per_trigger_zero_for_unlimited(0)
        self.camera.issue_software_trigger()


class galvo_controlling_thread(QThread):
    scanning_parameters_changed_signal = pyqtSignal(list)
    scanning_type_changed_signal = pyqtSignal(str)

    def __init__(self, galvo):
        super().__init__()
        self.galvo = galvo

    @pyqtSlot(int)
    def set_scanning_type(self, scanning_type):
        self.galvo.set_scanning_type(scanning_type)
        self.scanning_type_changed_signal.emit(scanning_type)

    @pyqtSlot(int)
    def set_scanning_parameters(self, scanning_parameters):
        scanning_radius = scanning_parameters[0]
        scanning_height = scanning_parameters[1]
        scanning_width = scanning_parameters[2]
        step_size = scanning_parameters[3]
        self.galvo.set_scanning_parameters(scanning_radius, scanning_height, scanning_width, step_size)
        self.scanning_parameters_changed_signal.emit(scanning_parameters)


class CustomLineEdit(QLineEdit):
    def mousePressEvent(self, event):
        # print("Mouse clicked.")
        super().mousePressEvent(event)
        self.selectAll()


class Galvo:
    def __init__(self):
        self.scanning = False
        self.scan_stopped = False
        self.scan_complete = False
        self.scanning_type = ''
        self.scanning_radius = []
        self.scanning_height = []
        self.scanning_width = []
        self.step_size = []
        self.task = None
        self.initialize()

    def initialize(self):
        # Create a new Task
        self.task = Task()
        self.set_scanning_type()
        self.set_scanning_parameters()
        self.go_to_zero_pos()

    def go_to_zero_pos(self):
        # Create a new Task
        self.task = Task()
        device = "Dev1"
        x_channel = "ao0"
        y_channel = "ao1"
        # Define the voltage range for the analog outputs
        voltage_min = -10.0
        voltage_max = 10.0

        # Configure the analog output channels
        self.task.CreateAOVoltageChan(f"{device}/{x_channel}", "", voltage_min, voltage_max,
                                      DAQmxConstants.DAQmx_Val_Volts, None)
        self.task.CreateAOVoltageChan(f"{device}/{y_channel}", "", voltage_min, voltage_max,
                                      DAQmxConstants.DAQmx_Val_Volts, None)

        # Write initial position to (0,0)
        self.task.WriteAnalogF64(1, True, 10.0, DAQmxConstants.DAQmx_Val_GroupByChannel, np.array([0.0, 0.0]), None,
                                 None)

    def set_scanning_type(self, scanning_type='Circular'):
        self.scanning_type = scanning_type

    def get_scanning_type(self):
        return self.scanning_type

    def set_scanning_parameters(self, scanning_radius=0.045, scanning_height=0.045, scanning_width=0.045,
                                step_size=0.01):
        self.scanning_radius = scanning_radius
        self.step_size = step_size
        self.scanning_height = scanning_height
        self.scanning_width = scanning_width

    def get_scanning_parameters(self):
        parameters = [self.scanning_radius,
                      self.scanning_height,
                      self.scanning_width,
                      self.step_size
                      ]
        return parameters


class image_processing_thread(QThread):
    pass


class image_saving_thread(QThread):
    def __init__(self, galvo, queues):
        super().__init__()
        self.active = False
        self.image_folder = self.load_image_folder()
        self.galvo = galvo
        self.captured_frames_to_save_queue = queues[1]
        self.scanned_frames_to_save_queue = queues[2]
        self.scanned_voltages_queue = queues[3]
        self.total_points_to_scan = []
        self.num_of_saved_frames = 0

    def run(self):
        timestamp = time.strftime('%d.%m.%Y %H-%M-%S')
        saving_folder = os.path.join(self.image_folder, timestamp)
        self.activate()
        self.num_of_saved_frames = 0
        while self.active and self.num_of_saved_frames != self.total_points_to_scan:
            try:
                frame_scanned = self.scanned_frames_to_save_queue.get()
                if frame_scanned is not None:
                    self.num_of_saved_frames += 1
                    self.save_frame(frame_scanned, self.num_of_saved_frames, False, saving_folder)
                if self.galvo.scan_stopped:
                    self.deactivate()
            except Exception as e:
                print(f"Error creating directory: {e}")
        self.deactivate()

    def activate(self):
        self.active = True

    def deactivate(self):
        self.active = False

    def save_frame(self, frame, frame_number, captured, saving_folder):
        if captured:
            filename = os.path.join(saving_folder,
                                    "frame_{}.png".format(time.strftime('%d.%m.%Y %H-%M-%S')))
        else:
            try:
                os.makedirs(saving_folder, exist_ok=True)
            except Exception as e:
                print(f"Error creating directory: {e}")
            frame_voltages = self.scanned_voltages_queue.get()
            print(f'in saving thread, point no. {frame_number}, volt:{frame_voltages[0]} and {frame_voltages[1]}')
            filename = os.path.join(saving_folder,
                                    f"frame_{frame_number} V_x is {frame_voltages[0]} and V_y is {frame_voltages[1]}.png")
        cv2.imwrite(filename, frame)

    def save_captures_frame(self):
        frame = self.captured_frames_to_save_queue.popleft()
        self.save_frame(frame, 0, True, self.image_folder)

    def load_image_folder(self):
        desktop_path = os.path.expanduser("~/Desktop")
        default_folder = os.path.join(desktop_path, "Images")
        settings_file = 'last_folder.json'
        try:
            with open(settings_file, 'r') as f:
                settings = json.load(f)
                image_folder = settings.get('last_folder', default_folder)
        except FileNotFoundError:
            image_folder = default_folder
        return image_folder

    def choose_folder(self):
        root = tk.Tk()
        root.withdraw()  # Hide the main window
        folder = filedialog.askdirectory()
        if folder:
            self.image_folder = folder
            self.save_last_folder()

    def save_last_folder(self):
        script_directory = os.path.dirname(os.path.abspath(__file__))
        file_path = os.path.join(script_directory, 'last_folder.json')
        settings = {'last_folder': self.image_folder}
        with open(file_path, 'w') as f:
            json.dump(settings, f)

    def open_folder(self):
        if hasattr(self, "image_folder") and os.path.exists(self.image_folder):
            os.startfile(self.image_folder)
        else:
            os.makedirs(self.image_folder)
            os.startfile(self.image_folder)
            print("No folder is selected or the selected folder does not exist.")


class processed_video_streaming_thread(QThread):
    pass


def main():
    app = QApplication(sys.argv)
    window = Camera_Application()
    window.show()
    sys.exit(app.exec_())


# Global stylesheet
STYLESHEET = """
    QGroupBox { 
        border: 3px solid gray; 
        border-radius: 5px;
        margin-top: 0.5em;
        background-color: lightgray; 
    }
    QGroupBox::title {
        subcontrol-origin: margin;
        left: 10px;
        padding: 0 3px 0 3px;
    }
"""
if __name__ == '__main__':
    cProfile.run('main()')
