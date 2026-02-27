
from PyQt5 import QtCore, QtGui, QtWidgets
from PyQt5.QtWidgets import QApplication
import sys
from classes.gui_functions import MainWindow


# change excle data saving routine because over time saving teh data is bad for memory
"""
import csv

# Create the file and write headers once
self.csv_file = open('sensor_data.csv', 'w', newline='')
self.csv_writer = csv.writer(self.csv_file)
self.csv_writer.writerow(["Time (s)", "I(A) | +Y", "I(A) | +X", "I(A) | -Y", "I(A) | -X", "I(A) | +Z", "I(A) | -Z"])

if self.save_status:
    self.sensor_values = [time.time() - self.starttime, self.coil1_current, self.coil2_current, 
                          self.coil3_current, self.coil4_current, self.coil5_current, self.coil6_current]
    
    # This is much faster and will not cause lag over time
    self.csv_writer.writerow(self.sensor_values)
    
    # Optional: ensure data is saved to disk immediately
    # self.csv_file.flush() 

    def close_application(self):
    self.csv_file.close()
    self.arduino_handler.close()
"""
if __name__ == "__main__":
    app = QtWidgets.QApplication(sys.argv)
    w = MainWindow()
    w.show()
    sys.exit(app.exec_())
