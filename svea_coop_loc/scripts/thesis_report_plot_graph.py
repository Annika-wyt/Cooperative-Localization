# import pandas as pd
# import numpy as np
# import matplotlib.pyplot as plt

# # Load the CSV file
# csv_file = "/home/annika/ITRL/kth_thesis/src/svea_thesis/scripts/test_case_aruco_error_log_full.csv"

# data = pd.read_csv(csv_file)

# # Remove duplicate rows based on 'timestamp' and 'source_frame'
# data = data.drop_duplicates(subset=['timestamp', 'source_frame'])

# # Convert timestamp to seconds relative to the first timestamp
# data['timestamp'] = data['timestamp'] -  1716814400.614320105 #data['timestamp'].min()

# # Compute the combined positional error (Euclidean distance) for each marker
# data['euclidean_error'] = np.sqrt(
#     data['translation_x']**2 + data['translation_y']**2 + data['translation_z']**2
# )

# # Plot Euclidean errors for all ArUco markers from 1 to 41
# plt.figure(figsize=(14, 8))

# # Loop over ArUco markers 1 to 41 dynamically
# for marker in range(1, 42):
#     marker_name = f'aruco{marker}'
#     if marker_name in data['source_frame'].unique():
#         marker_data = data[data['source_frame'] == marker_name]
#         plt.plot(marker_data['timestamp'], marker_data['euclidean_error'], label=f'{marker_name}')

# # Customize the plot
# plt.title('Euclidean Distance Error for ArUco Markers 1 to 41')
# plt.xlabel('Time (seconds)')
# plt.ylabel('Error (meters)')
# plt.ylim([0.0, 1.5])  # Adjust Y-axis limits
# plt.legend(loc='upper right', title='Legend', ncol=2)  # Add legend with multiple columns
# plt.grid(True)
# plt.tight_layout()

# # Save and show the plot
# # plt.savefig("aruco_markers_error_plot.png", bbox_inches='tight')
# plt.show()

# import pandas as pd
# import numpy as np
# import matplotlib.pyplot as plt

# # Load the data
# csv_file = "/home/annika/ITRL/kth_thesis/src/svea_thesis/scripts/test_case_2_aruco_38_to_41.csv"  # Replace with your actual CSV file path

# data = pd.read_csv(csv_file, engine='python')

# data = data.drop_duplicates(subset=['timestamp', 'marker_name'])


# # Calculate the positional difference (Euclidean distance) between actual and estimated positions
# data['position_difference'] = np.sqrt(
#     (data['est_position_x'] - data['act_position_x'])**2 +
#     (data['est_position_y'] - data['act_position_y'])**2 +
#     (data['est_position_z'] - data['act_position_z'])**2
# )

# # Plot the position difference over time for each ArUCo marker
# plt.figure(figsize=(14, 8))

# # Loop through each unique marker
# for marker_name in data['marker_name'].unique():
#     marker_data = data[data['marker_name'] == marker_name]
#     marker_data['timestamp'] = marker_data['timestamp'] - 1716825879.238437569
#     # 1716814400.614320105 test case 1 
#     # #data['timestamp'].min()

#     plt.plot(marker_data['timestamp'], marker_data['position_difference'], label=marker_name)

# # Customize the plot
# plt.title('Distance Error for ArUCo Marker #38 to #41')
# plt.xlabel('Time (seconds)')
# plt.ylabel('Error (meters)')
# plt.ylim([0.0, 0.6])  # Adjust Y-axis range
# plt.yticks(np.arange(0, data['position_difference'].max(), step=0.05))
# # plt.xlim([0.0, 30.0])  # Adjust X-axis range
# plt.legend(loc='upper right', title='ArUCo Markers')  # Multi-column legend
# plt.grid(True)
# plt.tight_layout()

# # Save and show the plot
# # plt.savefig("aruco_pose_difference_plot.png", bbox_inches='tight')
# plt.show()

import pandas as pd
import matplotlib.pyplot as plt

# Load the CSV file
csv_file = "/home/annika/ITRL/kth_thesis/thesis_report_ubuntu/single_vehicle_result_3_velocities.csv"
data = pd.read_csv(csv_file)

# Inspect the first few rows to understand the structure
print(data.head())

# Filter data for linear and angular velocity
linear_velocity_data = data[data['topic'] == '/actuation_twist.twist.twist.linear.x']  # Replace with the actual topic name for linear velocity
angular_velocity_data = data[data['topic'] == '/actuation_twist.twist.twist.angular.z']  # Replace with the actual topic name for angular velocity

# Plot linear and angular velocity over time
plt.figure(figsize=(12, 6))

# Linear velocity plot
plt.plot(linear_velocity_data['elapsed time'], linear_velocity_data['value'], label='Linear Velocity (m/s)', color='blue')

# Angular velocity plot
plt.plot(angular_velocity_data['elapsed time'], angular_velocity_data['value'], label='Angular Velocity (rad/s)', color='orange')


# Customize the plot
plt.title('Linear and Angular Velocities of SVEA in Test Case 3')
plt.xlabel('Time(s)')
plt.ylabel('Velocity')
plt.legend()
plt.grid(True)

# Show the plot
plt.tight_layout()
plt.show()
