import numpy as np
from tqdm import tqdm
from PyDAQmx import Task, int32
import PyDAQmx.DAQmxConstants as DAQmxConstants
import time
import matplotlib.pyplot as plt
from scipy.stats import linregress
import os
import pickle


def characterize_device(device_name, ao_channels, ai_channels, v_x_min, v_x_max, v_y_min, v_y_max, step_size, voltage_min, voltage_max):

    # Create new Tasks for AO and AI
    ao_task = Task()
    ai_task = Task()

    # Create the voltage ranges
    voltage_range_x = np.arange(v_x_min, v_x_max + step_size, step_size)
    voltage_range_y = np.arange(v_y_min, v_y_max + step_size, step_size)

    sent_voltages = []
    read_voltages = []

    # Configure the analog output channels with the correct voltage_min and voltage_max
    for ao_channel in ao_channels:
        ao_task.CreateAOVoltageChan(f"{device_name}/{ao_channel}", "", voltage_min, voltage_max, DAQmxConstants.DAQmx_Val_Volts, None)
    
    # Configure the analog input channels
    for ai_channel in ai_channels:
        ai_task.CreateAIVoltageChan(f"{device_name}/{ai_channel}", "", DAQmxConstants.DAQmx_Val_Cfg_Default, voltage_min, voltage_max, DAQmxConstants.DAQmx_Val_Volts, None)

    # Start the AO and AI tasks
    ao_task.StartTask()
    ai_task.StartTask()

    # Nested loops to iterate over voltage ranges
    for vx in tqdm(voltage_range_x, desc=f'Progress {device_name} - X'):
        for vy in voltage_range_y:
            # Write voltages to AO channels
            sent_voltage = np.array([vx, vy], dtype=np.float64)
            sent_voltages.append(sent_voltage)
            ao_task.WriteAnalogF64(1, True, 10.0, DAQmxConstants.DAQmx_Val_GroupByChannel, sent_voltage, None, None)
            
            # Small delay to allow the voltage to settle
            time.sleep(1/130)
            
            # Read voltages from AI channels
            ai_data = np.zeros((2,), dtype=np.float64)
            read = int32()
            ai_task.ReadAnalogF64(1, 10.0, DAQmxConstants.DAQmx_Val_GroupByChannel, ai_data, 2, read, None)
            read_voltages.append(ai_data)

    # Stop and clear the tasks
    ao_task.StopTask()
    ao_task.ClearTask()
    ai_task.StopTask()
    ai_task.ClearTask()

    # Convert lists to numpy arrays for analysis
    sent_voltages = np.array(sent_voltages)
    read_voltages = np.array(read_voltages)

    # Calculate error metrics
    differences = read_voltages - sent_voltages
    offset_error = np.mean(differences, axis=0)
    noise = np.std(differences, axis=0)
    slope, intercept, r_value, p_value, std_err = linregress(sent_voltages[:, 0], read_voltages[:, 0])
    gain_error = slope - 1  # Ideal slope is 1
    linearity_error = std_err

    # Return the results
    return {
        'sent_voltages': sent_voltages,
        'read_voltages': read_voltages,
        'offset_error': offset_error,
        'gain_error': gain_error,
        'linearity_error': linearity_error,
        'noise': noise
    }

# Voltage range for NI USB-6003
voltage_min_6003 = -10.0
voltage_max_6003 = 10.0

# Voltage range for NI USB-6009
voltage_min_6009 = 0.0
voltage_max_6009 = 5.0

# Define constants
v_x_min = -0.05 + 2.5
v_x_max = 0.05 + 2.5
v_y_min = -0.05 + 2.5
v_y_max = 0.05 + 2.5
step_size = 1*1e-2
ao_channels = ["ao0", "ao1"]
ai_channels = ["ai0", "ai1"]

# Run characterization on both devices
device_6003 = 'Dev1'  # Replace with the actual name for the USB-6003
device_6009 = 'Dev2'  # Replace with the actual name for the USB-6009

# Run characterization on NI USB-6003
results_6003 = characterize_device(device_6003, ao_channels, ai_channels, v_x_min, v_x_max, v_y_min, v_y_max, step_size, voltage_min_6003, voltage_max_6003)

# Run characterization on NI USB-6009
results_6009 = characterize_device(device_6009, ao_channels, ai_channels, v_x_min, v_x_max, v_y_min, v_y_max, step_size, voltage_min_6009, voltage_max_6009)

# Create a 2D plot for sent voltages
sent_voltages = results_6003['sent_voltages']
read_voltages = results_6003['read_voltages']

plt.figure()
plt.scatter(sent_voltages[:, 0], sent_voltages[:, 1], c='blue', label='Sent Voltages')
plt.title('2D Plot of Sent Voltages')
plt.xlabel('Vx (V)')
plt.ylabel('Vy (V)')
plt.legend()
plt.grid(True)
plt.show(block=False)

# Create a 2D plot for read voltages
plt.figure()
plt.scatter(read_voltages[:, 0], read_voltages[:, 1], c='red', label='Read Voltages')
plt.title('2D Plot of Read Voltages')
plt.xlabel('Vx (V)')
plt.ylabel('Vy (V)')
plt.legend()
plt.grid(True)
plt.show(block=False)

# For example, to plot sent vs read voltages for both devices:
plt.figure()
plt.plot(results_6003['sent_voltages'][:, 0], results_6003['read_voltages'][:, 0], 'o-', label='USB-6003')
plt.plot(results_6009['sent_voltages'][:, 0], results_6009['read_voltages'][:, 0], 'o-', label='USB-6009')
plt.title('Comparison of Sent vs Read Voltages')
plt.xlabel('Sent Voltage (V)')
plt.ylabel('Read Voltage (V)')
plt.legend()
plt.grid(True)
plt.show(block=False)

combined_results = {
    'results_6003': results_6003,
    'results_6009': results_6009
}


# Get the current working directory
current_directory = os.getcwd()
print(f"The current working directory is: {current_directory}")

# Specify the filename
filename = 'combined_characterization_data.pkl'

# Create a full path for the file
file_path = os.path.join(current_directory, filename)
print(f"The file will be saved as: {file_path}")

# ... your existing code ...

# Use the full path when saving the file
with open(file_path, 'wb') as output_file:
    pickle.dump(combined_results, output_file)
print(f"The current working directory is: {current_directory}")


a=5