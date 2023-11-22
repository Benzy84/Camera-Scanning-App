import numpy as np
from PyDAQmx import Task, int32
import PyDAQmx.DAQmxConstants as DAQmxConstants
import time

# Create new Tasks for AO and AI
ao_task = Task()
ai_task = Task()

# Define the USB-6003 device and analog channels
device = "Dev1"  # Replace this with your device name if different
ao_channels = ["ao0", "ao1"]
ai_channels = ["ai0", "ai1"]  # Replace with your actual AI channels

# Define the voltage range for the analog outputs and inputs
voltage_min = -10.0
voltage_max = 10.0

# Configure the analog output channels
for ao_channel in ao_channels:
    ao_task.CreateAOVoltageChan(f"{device}/{ao_channel}", "", voltage_min, voltage_max, DAQmxConstants.DAQmx_Val_Volts, None)

# Configure the analog input channels
for ai_channel in ai_channels:
    ai_task.CreateAIVoltageChan(f"{device}/{ai_channel}", "", DAQmxConstants.DAQmx_Val_Cfg_Default, voltage_min, voltage_max, DAQmxConstants.DAQmx_Val_Volts, None)

# Define the circular scanning parameters
radius = 0.045
step_size = 0.01

data = np.array([0.0, 0.0], dtype=np.float64)
ao_task.WriteAnalogF64(1, True, 10.0, DAQmxConstants.DAQmx_Val_GroupByChannel,data, None, None)
data = np.array([0.12, 0.12], dtype=np.float64)
ao_task.WriteAnalogF64(1, True, 10.0, DAQmxConstants.DAQmx_Val_GroupByChannel,data, None, None)
# Add a time delay to allow the voltages to settle
time.sleep(0.01)
# Prepare a buffer to store the data read from the AI channels
ai_data = np.zeros((2,), dtype=np.float64)  # Buffer for two channels
# Read from the AI channels
ai_data = np.zeros((2,), dtype=np.float64)  # Buffer for two channels
read = int32()
ai_task.ReadAnalogF64(1, 10.0, DAQmxConstants.DAQmx_Val_GroupByChannel, ai_data, 2, read, None)
print(f"Read voltages: X={ai_data[0]}, Y={ai_data[1]}")
# Read from the AI channels
ai_data = np.zeros((2,), dtype=np.float64)


data = np.array([6.0, 9.0], dtype=np.float64)
ao_task.WriteAnalogF64(1, True, 10.0, DAQmxConstants.DAQmx_Val_GroupByChannel,data, None, None)
# Add a time delay to allow the voltages to settle
time.sleep(0.01)
# Prepare a buffer to store the data read from the AI channels
ai_data = np.zeros((2,), dtype=np.float64)  # Buffer for two channels
read = int32()
ai_task.ReadAnalogF64(1, 10.0, DAQmxConstants.DAQmx_Val_GroupByChannel, ai_data, 2, read, None)
print(f"Read voltages: X={ai_data[0]}, Y={ai_data[1]}")
# Read from the AI channels
ai_data = np.zeros((2,), dtype=np.float64)




# Perform the area scanning within the circle
x_range = np.arange(-radius, radius, step_size)
# x_range = np.array([0.0])
y_range = np.arange(-radius, radius, step_size)
y_range = np.array([0.0])

for x_voltage in x_range:
    for y_voltage in y_range:
        # Check if the point is inside the circle
        if x_voltage ** 2 + y_voltage ** 2 <= radius ** 2:
            data = np.array([x_voltage, y_voltage], dtype=np.float64)
            task.WriteAnalogF64(1, True, 10.0, DAQmxConstants.DAQmx_Val_GroupByChannel, data, None, None)

            # Add a time delay between each point
            time.sleep(0.5)

# Move the Galvo to the center (0, 0)
task.WriteAnalogF64(1, True, 10.0, DAQmxConstants.DAQmx_Val_GroupByChannel, np.array([0.0, 0.0]), None, None)

# Clean up and close the Task
task.StopTask()
task.ClearTask()

