import json
import serial
import serial.tools.list_ports
import sched
import sys
import threading
import time
import os
from PyQt5 import QtWidgets, uic, QtGui
from zaber_motion.ascii import Connection, Device, DeviceIO
import math

class Ui(QtWidgets.QMainWindow):
    def __init__(self):
        super(Ui, self).__init__()
        uic.loadUi('transfer_ui.ui', self)
        self.show()

        self.zaber_baud = 115200
        self.current_comport = ''
        self.SYSTEM_STATE = 'STARTUP'    
        # Transfer positions. These should always be set to zero. They will be loaded from the 
        # transfer_positions.json file.
        self.positions = {"robomet_load": {"xpos": 0, "ypos": 0, "zpos": 0},
                          "xz_transfer": {"xpos": 0, "ypos": 0, "zpos": 0},
                          "sras_load": {"xpos": 0, "ypos": 0, "zpos": 0}}

        self.is_at_home = False
        self.is_at_r3d_load = False
        self.is_at_xz_load = False
        self.is_at_sras_load = False

        # signal and slot definitions
        self.button_comconnect.clicked.connect(self.connect_com_port)
        self.btn_r3dh_sync.clicked.connect(self.read_encoder_position_r3dh)
        self.btn_stp_sync.clicked.connect(self.read_encoder_position_stp)
        self.btn_sdp_sync.clicked.connect(self.read_encoder_position_sdp)

        # Save buttons
        self.btn_r3dh_save.clicked.connect(self.write_json_data)
        self.btn_stp_save.clicked.connect(self.write_json_data)
        self.btn_sdp_save.clicked.connect(self.write_json_data)

        # Auto Button
        self.btn_auto_toggle.clicked.connect(self.toggle_auto_mode)
        # Set system state to the idling / disconnected state
        if self.SYSTEM_STATE == 'STARTUP':
            self.set_ui_state()
            self.enumerate_com_ports()
            if(os.path.exists('transfer_positions.json')):
                self.msgbox = QtWidgets.QMessageBox.critical(self, "JSON Data Found!", 
                                                            "transfer_positions.json was found. Loading saved positions.", 
                                                            QtWidgets.QMessageBox.Ok) 

            else:
                self.msgbox = QtWidgets.QMessageBox.critical(self, "JSON Data Not Found!", 
                                                            "transfer_positions.json was not found. An empty JSON file has been created. You'll need to save new positions.", 
                                                            QtWidgets.QMessageBox.Ok)
                self.write_json_data()
            
    def set_ui_state(self):
        '''
        set_ui_state: Display "Not connected" text and disable all UI
                                elements with the exception of the COM connection controls
        '''
        if (self.SYSTEM_STATE == 'OFFLINE') or (self.SYSTEM_STATE == 'STARTUP'):
            self.label_system_state.setText("Not Connected")
            self.xaxis_counts.setEnabled(False)
            self.xaxis_mm.setEnabled(False)
            self.yaxis_counts.setEnabled(False)
            self.yaxis_mm.setEnabled(False)
            self.zaxis_counts.setEnabled(False)
            self.zaxis_mm.setEnabled(False)
            self.label_rtl_signal.setEnabled(False)
            self.label_rts_signal.setEnabled(False)
            self.label_r3dsafe.setEnabled(False)
            self.label_srasready_signal.setEnabled(False)
            self.label_ctl_signal.setEnabled(False)
            self.label_srascomplete_signal.setEnabled(False)
            self.label_sraserror_signal.setEnabled(False)
            self.txt_r3dh_x.setEnabled(False)
            self.txt_r3dh_y.setEnabled(False)
            self.txt_r3dh_z.setEnabled(False)
            self.btn_r3dh_sync.setEnabled(False)
            self.btn_r3dh_save.setEnabled(False)
            self.txt_stp_x.setEnabled(False)
            self.txt_stp_y.setEnabled(False)
            self.txt_stp_z.setEnabled(False)
            self.btn_stp_sync.setEnabled(False)
            self.btn_stp_save.setEnabled(False)
            self.txt_sdp_x.setEnabled(False)
            self.txt_sdp_y.setEnabled(False)
            self.txt_sdp_z.setEnabled(False)
            self.btn_sdp_sync.setEnabled(False)
            self.btn_sdp_save.setEnabled(False)
            self.btn_auto_toggle.setEnabled(False)
        elif self.SYSTEM_STATE == "ONLINE":
            self.label_system_state.setText("Connected. Not Homed.")
            self.xaxis_counts.setEnabled(True)
            self.xaxis_mm.setEnabled(True)
            self.yaxis_counts.setEnabled(True)
            self.yaxis_mm.setEnabled(True)
            self.zaxis_counts.setEnabled(True)
            self.zaxis_mm.setEnabled(True)
            self.label_rtl_signal.setEnabled(True)
            self.label_rts_signal.setEnabled(True)
            self.label_r3dsafe.setEnabled(True)
            self.label_srasready_signal.setEnabled(True)
            self.label_ctl_signal.setEnabled(True)
            self.label_srascomplete_signal.setEnabled(True)
            self.label_sraserror_signal.setEnabled(True)
            self.txt_r3dh_x.setEnabled(True)
            self.txt_r3dh_y.setEnabled(True)
            self.txt_r3dh_z.setEnabled(True)
            self.btn_r3dh_sync.setEnabled(True)
            self.btn_r3dh_save.setEnabled(True)
            self.txt_stp_x.setEnabled(True)
            self.txt_stp_y.setEnabled(True)
            self.txt_stp_z.setEnabled(True)
            self.btn_stp_sync.setEnabled(True)
            self.btn_stp_save.setEnabled(True)
            self.txt_sdp_x.setEnabled(True)
            self.txt_sdp_y.setEnabled(True)
            self.txt_sdp_z.setEnabled(True)
            self.btn_sdp_sync.setEnabled(True)
            self.btn_sdp_save.setEnabled(True)
            self.btn_auto_toggle.setEnabled(True)
        self.encoder_thread = threading.Thread(target=self.start_polling_axes, args=(),
                                               daemon=True)
        return

    def enumerate_com_ports(self):
        '''
        enumerate_com_ports(): Checks the COM ports for any zaber controllers that 
                               may be connected.

        '''
        possible_ports = serial.tools.list_ports.comports()
        if possible_ports.__len__() == 0:
            print("No valid COMports found.")
            error_message = QtWidgets.QMessageBox.critical(self, "Serial Subsystem Error", 
                                                            "No valid COM ports were detected on this computer. Check connections to PC.", 
                                                            QtWidgets.QMessageBox.Ok)
        else:
            for current_port in possible_ports:
                self.combo_comselect.addItem(current_port.__str__())

    def connect_com_port(self):
        if self.button_comconnect.text() == "Connect to Transfer Controller":
            combobox_string = self.combo_comselect.currentText()
            self.current_comport = combobox_string.split(' ')[0]
            com_msg = QtWidgets.QMessageBox.critical(self, "COMPORT", "Chosen com is : " + self.current_comport)
            self.SYSTEM_STATE = "ONLINE"
            self.set_ui_state()
            time.sleep(0.250)
            self.zaber_ascii_connection = Connection.open_serial_port(self.current_comport)
            self.xy_controller = self.zaber_ascii_connection.get_device(1)
            self.z_controller = self.zaber_ascii_connection.get_device(2)
            self.x_axis = self.xy_controller.get_axis(1)
            self.y_axis = self.xy_controller.get_axis(2)
            self.z_axis = self.z_controller.get_axis(1)
            self.encoder_thread.start()
            self.xy_controller.io.set_all_digital_outputs([False, False, False, False])
            self.button_comconnect.setText("Disconnect Transfer Controller")
            self.homing_thread = threading.Thread(None, self.home_connected_stages)
            self.homing_thread.start()
            self.read_json_data()
            self.is_ready = True
            self.xy_controller.io.set_digital_output(1, True)
        else:
            self.zaber_ascii_connection.close()
            self.button_comconnect.setText("Connect to Transfer Controller")
            self.enumerate_com_ports()
            self.SYSTEM_STATE = "OFFLINE"
            self.set_ui_state()
        return

    def home_connected_stages(self):
        self.label_system_state.setText("Homing X...")
        self.x_axis.home(wait_until_idle=True)
        self.label_system_state.setText("Homing Y...")
        self.y_axis.home(wait_until_idle=True)
        self.label_system_state.setText("Homing Z...")
        self.z_axis.home(wait_until_idle=True)
        self.label_system_state.setText("Homing Complete.")
        return

    def start_polling_axes(self, poll_delay_ms=250):
        while(self.SYSTEM_STATE == 'ONLINE'):
            fuzz_factor_steps = 200
            self.xaxis_steps = self.x_axis.get_position()
            self.yaxis_steps = self.y_axis.get_position()
            self.zaxis_steps = self.z_axis.get_position()
            self.xaxis_counts.setText(self.xaxis_steps.__str__())
            self.yaxis_counts.setText(self.yaxis_steps.__str__())
            self.zaxis_counts.setText(self.zaxis_steps.__str__())

            # Homed position check
            if ((0 - fuzz_factor_steps <= (self.xaxis_steps) < 0 + fuzz_factor_steps) and
                (0 - fuzz_factor_steps <= (self.yaxis_steps) < 0 + fuzz_factor_steps) and
                (0 - fuzz_factor_steps <= (self.zaxis_steps) < 0 + fuzz_factor_steps)):
                self.system_at_home = True
            else:
                self.system_at_home = False
            
            # Robomet Load Position Check
            if ((self.positions["robomet_load"]["xpos"] - fuzz_factor_steps) <= (self.xaxis_steps) < (self.positions["robomet_load"]["xpos"] + fuzz_factor_steps) and 
                (self.positions["robomet_load"]["ypos"] - fuzz_factor_steps) <= (self.yaxis_steps) < (self.positions["robomet_load"]["ypos"] + fuzz_factor_steps) and
                (self.positions["robomet_load"]["zpos"] - fuzz_factor_steps) <= (self.zaxis_steps) < (self.positions["robomet_load"]["zpos"] + fuzz_factor_steps)):
                self.system_at_r3d_load = True
            else:
                self.system_at_r3d_load = False

            # X-Z Handoff Position Check
            if ((self.positions["xz_transfer"]["xpos"] - fuzz_factor_steps) <= (self.xaxis_steps) < (self.positions["xz_transfer"]["xpos"] + fuzz_factor_steps) and 
                (self.positions["xz_transfer"]["ypos"] - fuzz_factor_steps) <= (self.yaxis_steps) < (self.positions["xz_transfer"]["ypos"] + fuzz_factor_steps) and
                (self.positions["xz_transfer"]["zpos"] - fuzz_factor_steps) <= (self.zaxis_steps) < (self.positions["xz_transfer"]["zpos"] + fuzz_factor_steps)):
                self.system_at_xz_load = True
            else:
                self.system_at_xz_load = False

            # SRAS Dropoff Position Check
            if ((self.positions["sras_load"]["xpos"] - fuzz_factor_steps) <= (self.xaxis_steps) < (self.positions["sras_load"]["xpos"] + fuzz_factor_steps) and 
                (self.positions["sras_load"]["ypos"] - fuzz_factor_steps) <= (self.yaxis_steps) < (self.positions["sras_load"]["ypos"] + fuzz_factor_steps) and
                (self.positions["sras_load"]["zpos"] - fuzz_factor_steps) <= (self.zaxis_steps) < (self.positions["sras_load"]["zpos"] + fuzz_factor_steps)):
                self.system_at_xz_load = True
            else:
                self.system_at_xz_load = False

            # Read Controller Input States
            self.xy_digi_inputs = self.xy_controller.io.get_all_digital_inputs()
            self.z_digi_inputs = self.z_controller.io.get_all_digital_inputs()
            self.xy_digi_outputs = self.xy_controller.io.get_all_digital_outputs()
            self.z_digi_outputs = self.z_controller.io.get_all_digital_outputs()

            if self.xy_digi_inputs[0] == True:
                self.label_rtl_signal.setFont(QtGui.QFont("MS Shell Dlg 2", 8, QtGui.QFont.Bold))
            else:
                self.label_rtl_signal.setFont(QtGui.QFont("MS Shell Dlg 2", 8, QtGui.QFont.Medium))
            if self.xy_digi_inputs[1] == True:
                self.label_rts_signal.setFont(QtGui.QFont("MS Shell Dlg 2", 8, QtGui.QFont.Bold))
            else:
                self.label_rts_signal.setFont(QtGui.QFont("MS Shell Dlg 2", 8, QtGui.QFont.Medium))
            if self.xy_digi_inputs[2] == True:
                self.label_r3dsafe.setFont(QtGui.QFont("MS Shell Dlg 2", 8, QtGui.QFont.Bold))
            else:
                self.label_r3dsafe.setFont(QtGui.QFont("MS Shell Dlg 2", 8, QtGui.QFont.Medium))

            if self.xy_digi_outputs[0] == True:
                self.label_srasready_signal.setFont(QtGui.QFont("MS Shell Dlg 2", 8, QtGui.QFont.Bold))
            else:
                self.label_srasready_signal.setFont(QtGui.QFont("MS Shell Dlg 2", 8, QtGui.QFont.Medium))
            if self.xy_digi_outputs[1] == True:
                self.label_ctl_signal.setFont(QtGui.QFont("MS Shell Dlg 2", 8, QtGui.QFont.Bold))
            else:
                self.label_ctl_signal.setFont(QtGui.QFont("MS Shell Dlg 2", 8, QtGui.QFont.Medium))
            if self.xy_digi_outputs[2] == True:
                self.label_srascomplete_signal.setFont(QtGui.QFont("MS Shell Dlg 2", 8, QtGui.QFont.Bold))
            else:
                self.label_srascomplete_signal.setFont(QtGui.QFont("MS Shell Dlg 2", 8, QtGui.QFont.Medium))
            if self.xy_digi_outputs[3] == True:
                self.label_sraserror_signal.setFont(QtGui.QFont("MS Shell Dlg 2", 8, QtGui.QFont.Bold))
            else:
                self.label_sraserror_signal.setFont(QtGui.QFont("MS Shell Dlg 2", 8, QtGui.QFont.Medium))
            time.sleep(poll_delay_ms/1000)


        return
    
    def write_json_data(self):
        self.positions["robomet_load"]["xpos"] = round(float(self.txt_r3dh_x.text()))
        self.positions["robomet_load"]["ypos"] = round(float(self.txt_r3dh_y.text()))
        self.positions["robomet_load"]["zpos"] = round(float(self.txt_r3dh_z.text()))
        self.positions["xz_transfer"]["xpos"] = round(float(self.txt_stp_x.text()))
        self.positions["xz_transfer"]["ypos"] = round(float(self.txt_stp_y.text()))
        self.positions["xz_transfer"]["zpos"] = round(float(self.txt_stp_z.text()))
        self.positions["sras_load"]["xpos"] = round(float(self.txt_sdp_x.text()))
        self.positions["sras_load"]["ypos"] = round(float(self.txt_sdp_y.text()))
        self.positions["sras_load"]["zpos"] = round(float(self.txt_sdp_z.text()))
        with open('transfer_positions.json', 'w') as json_file:
            json.dump(self.positions, json_file)

        self.msgbox = QtWidgets.QMessageBox.information(self, "JSON Data Written", 
                                                        "JSON Data for the transfer system positions has been written to \'transfer_positions.json\'.")
        return

    def read_json_data(self):
        with open('transfer_positions.json', 'r') as json_file:
            self.positions = json.load(json_file)

        self.msgbox = QtWidgets.QMessageBox.information(self, "JSON Data Loaded", "JSON Data has been loaded from \'transfer_positions.json\'.")
        
        # Populate text boxes with current values
        self.txt_r3dh_x.setText(self.positions["robomet_load"]["xpos"].__str__())
        self.txt_r3dh_y.setText(self.positions["robomet_load"]["ypos"].__str__())
        self.txt_r3dh_z.setText(self.positions["robomet_load"]["zpos"].__str__())
        
        self.txt_stp_x.setText(self.positions["xz_transfer"]["xpos"].__str__())
        self.txt_stp_y.setText(self.positions["xz_transfer"]["ypos"].__str__())
        self.txt_stp_z.setText(self.positions["xz_transfer"]["zpos"].__str__())

        self.txt_sdp_x.setText(self.positions["sras_load"]["xpos"].__str__())
        self.txt_sdp_y.setText(self.positions["sras_load"]["ypos"].__str__())
        self.txt_sdp_z.setText(self.positions["sras_load"]["zpos"].__str__())
        return

    def read_encoder_position_r3dh(self):
        self.txt_r3dh_x.setText(self.xaxis_steps.__str__())
        self.txt_r3dh_y.setText(self.yaxis_steps.__str__())
        self.txt_r3dh_z.setText(self.zaxis_steps.__str__())
        return

    def read_encoder_position_stp(self):
        self.txt_stp_x.setText(self.xaxis_steps.__str__())
        self.txt_stp_y.setText(self.yaxis_steps.__str__())
        self.txt_stp_z.setText(self.zaxis_steps.__str__())
        
    def read_encoder_position_sdp(self):
        self.txt_sdp_x.setText(self.xaxis_steps.__str__())
        self.txt_sdp_y.setText(self.yaxis_steps.__str__())
        self.txt_sdp_z.setText(self.zaxis_steps.__str__())

    def toggle_auto_mode(self):

        if(self.label_system_state.text() != "Auto Mode"):
            self.label_system_state.setText("Auto Mode")
            self.auto_daemon_thread = threading.Thread(None, 
                                                        self.auto_mode_daemon,
                                                        daemon=True)
            self.auto_daemon_thread.start()
        else:
            self.label_system_state.setText("Auto Mode Disabled")

    def auto_mode_daemon(self):
        while(self.label_system_state.text() == "Auto Mode"):
            # wait for the robomet OK
            
        print("daemon shutting down...")

app = QtWidgets.QApplication(sys.argv)
window = Ui()
app.exec_()