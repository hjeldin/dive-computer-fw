import matplotlib.pyplot as plt
import numpy as np
import json
import pandas as pd 

depth = [0];
ascending = False;
descending = True;
bottom = False;

target_depth = 40

bottom_time = 35 * 60
descent_speed = 0.33 # m/s
ascent_speed = 0.17 # m/s
safety_stop_speed = 0.083 # m/s
safety_stop_duration = 0 * 60
safety_stop_depth = -5

# Generate a dive profile
# descent
while depth[-1]-descent_speed > -target_depth:
    depth.append( (depth[-1] - descent_speed) )
    print(depth[-1])

# bottom
for i in range(bottom_time):
    depth.append(-target_depth)

# ascent
while depth[-1] + ascent_speed < safety_stop_depth:
    depth.append((depth[-1] + ascent_speed))

# deco stop
for i in range(safety_stop_duration):
    depth.append(safety_stop_depth)

# ascent
while depth[-1] < 0:
    if depth[-1] + safety_stop_speed > 0:
        depth.append(0)
    else:
        depth.append((depth[-1] + safety_stop_speed))

# # Plot the dive profile
# plt.figure(figsize=(10, 6))
# plt.xlabel('Time (seconds)')
# plt.ylabel('Depth (meters)')
# plt.title('Scuba Dive: Depth vs Time')
# plt.grid(True)
# plt.axhline(y=0, color='black', linewidth=1)  # Surface line
# plt.axhline(y=5, color='r', linestyle='--', label='Safety Stop at 5m')  # Safety stop
# plt.axhline(y=40, color='g', linestyle='--', label='Bottom at 40m')  # Bottom depth
# plt.plot(depth)

# # Adding legend
# plt.legend()

# # Show plot
# plt.show()

df = pd.DataFrame(depth)
df.to_csv("../dive-deco-x86/depth.csv", index=False, header=False)