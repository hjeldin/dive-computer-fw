# read file tissues.csv and plot the saturation of the tissues

import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from plotly.subplots import make_subplots
import plotly.graph_objects as go
from bokeh.plotting import figure, show, save
from bokeh.io import curdoc
from bokeh.models import HoverTool, LinearAxis, Range1d

# Read the tissues.csv file
df = pd.read_csv('../dive-deco-x86/tissues.csv')

m_values = pd.read_csv('../dive-deco-x86/m_values.csv')
# plots = []
# ig, (ax1, ax2) = plt.subplots(2, 1)
# for i in range(1, 16):
#     plots.append(ax1.plot(df['amb_pressure'], df['tissue_'+str(i)], label='Tissue ' + str(i)))
#     plots.append(ax1.plot(m_values['amb_pressure'], m_values['m_value_'+str(i)]))
# ax1.set_title('Tissues Saturation vs Pressure')
# ax1.set_xlabel('Ambient pressure (bar - absolute)')
# ax1.set_ylabel('Tissue N2 partial pressure (bar)')
# ax1.axis('equal')
# ax1.set_aspect('equal', 'box')
# leg = ax1.legend()
# leg.set_draggable(True)

# ax1.grid(True)
# ax2.set_xlabel('Time (min)')
# ax2.set_ylabel('Tissue N2 partial pressure (bar)')
# ax2.set_title('Tissues Saturation vs Time')
# ax2.grid(True)
# for i in range(1, 16):
#     ax2.plot(df['time']/60, df['tissue_'+str(i)], label='Tissue ' + str(i))

# plt.tight_layout()
# plt.show()



# curdoc().theme = "dark_minimal"
# p = figure(title="Tissues saturation vs pressure", x_axis_label='Ambient pressure (bar - absolute)', y_axis_label='Tissue N2 partial pressure (bar)', sizing_mode="stretch_both",tools=[HoverTool()],tooltips="Data point @x has the value @y",)
# for i in range(1,16):
#     max_value = max(df['tissue_'+str(i)])
#     p.extra_y_ranges['m_value_'+str(i)] = Range1d(start=m_values['m_value_'+str(i)][0], end=max_value)
#     ax2 = LinearAxis(y_range_name='m_value_'+str(i))
#     p.add_layout(ax2, 'left')
# for i in range(1, 16):
#     p.line(df['amb_pressure'], df['tissue_'+str(i)], legend_label='Tissue ' + str(i), line_width=2)
#     # p.line(m_values['amb_pressure'], m_values['m_value_'+str(i)], line_width=0.5)

# save(p, './aux/viz/tissues_saturation_pressure.html')


import plotly.express as px
fig = make_subplots(rows=1, cols=1)
for i in range(1, 16):
    fig.add_trace(go.Scatter(x=df['amb_pressure'], y=df['tissue_'+str(i)], mode='lines', name='Tissue ' + str(i)), row=1, col=1)
    fig.add_trace(go.Scatter(x=m_values['amb_pressure'], y=m_values['m_value_'+str(i)], mode='lines', name='M-value ' + str(i)), row=1, col=1)
# fig = px.line(df, x='amb_pressure', y=df.columns[2:], title='Tissues Saturation vs Pressure')
# px.line(m_values, x='amb_pressure', y=m_values.columns[1:], title='M-values vs Pressure')
fig.write_html('first_figure.html', auto_open=True)