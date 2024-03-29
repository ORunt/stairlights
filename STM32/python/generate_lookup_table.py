import os, sys, re
import matplotlib.pyplot as plt
import numpy as np

plot = True
if len(sys.argv) > 1:
    if str(sys.argv[1]) == "no_plot":
        plot = False


# Setup working directories
script_dir = os.path.dirname(os.path.realpath(__file__))
lt_path_h = os.path.join(os.path.split(script_dir)[0], 'inc/graph_lookup_table.h')
lt_path_c = os.path.join(os.path.split(script_dir)[0], 'src/graph_lookup_table.c')
ref_path = os.path.join(os.path.split(script_dir)[0], 'inc/main.h')

graph_types = ["LINEAR", "PARABOLA_FS", "PARABOLA_SS", "EXPONENTIAL_FS", "EXPONENTIAL_SS"]
graph_params = {"FADE_DURATION":0, "STEPS_DURATION":0, "MAX_BRIGHTNESS_DAY":0, "MAX_BRIGHTNESS_NIGHT":0, "LED_CHANNELS":0, "FREQUENCY":0}
graph_sel = {"STAIR_FUNCTION":"", "FADE_FUNCTION":""}

def graph_function(graph, x_lim, y_lim):
    y = np.linspace(0, y_lim-1, y_lim)
    if graph == "PARABOLA_FS":
        a = pow(y_lim-1, 2) / x_lim
        return np.square(y) / a, y
    if graph == "PARABOLA_SS":
        a = pow(x_lim, 2) / (y_lim-1)
        return np.sqrt(y * a), y
    if graph == "LINEAR":
        m = y_lim / x_lim
        return y / m, y

with open(ref_path, 'r') as f:
    line = f.readline()
    while line:
        if re.findall(r'^#define\s+(\w+)\s+(\d+)', line):
            param, value = re.findall(r'^#define\s+(\w+)\s+(\d+)', line)[0]
            if param in graph_params.keys():
                graph_params[param] = int(value)
        if re.findall(r'^#define\s+(\w+)\s+(\w+)', line):
            item, g_type = re.findall(r'^#define\s+(\w+)\s+(\w+)', line)[0]
            if item in graph_sel.keys():
                graph_sel[item] = g_type
        line = f.readline()
    graph_params["FADE_DURATION"] = graph_params["FADE_DURATION"] * graph_params["FREQUENCY"] / 10
    graph_params["STEPS_DURATION"] = graph_params["STEPS_DURATION"] * graph_params["FREQUENCY"] / 10

print(graph_params)
print("STAIR_FUNCTION: " + graph_sel["STAIR_FUNCTION"])
print("FADE_FUNCTION: " + graph_sel["FADE_FUNCTION"])

fade_graph_d_x, fade_graph_d_y = graph_function(graph_sel["FADE_FUNCTION"], graph_params["FADE_DURATION"], graph_params["MAX_BRIGHTNESS_DAY"])
fade_graph_n_x, fade_graph_n_y = graph_function(graph_sel["FADE_FUNCTION"], graph_params["FADE_DURATION"], graph_params["MAX_BRIGHTNESS_NIGHT"])
stair_graph_x, stair_graph_y = graph_function(graph_sel["STAIR_FUNCTION"], graph_params["STEPS_DURATION"], graph_params["LED_CHANNELS"])

# plot the graph
if plot:
    plt.figure(1)
    plt.subplot(2, 1, 1)
    plt.title("LED Fade Profile")
    plt.plot(fade_graph_d_x / (graph_params["FREQUENCY"] * 100.0), fade_graph_d_y, color = 'tab:green', marker='o', label='Day')
    plt.plot(fade_graph_n_x / (graph_params["FREQUENCY"] * 100.0), fade_graph_n_y, color = 'tab:blue', marker='o', label='Night')
    plt.xlabel('Time (sec)')
    plt.ylabel('LED Brightness')
    plt.legend()

    plt.subplot(2, 1, 2)
    plt.title("Stair light Profile")
    plt.plot(stair_graph_x / (graph_params["FREQUENCY"] * 100.0), stair_graph_y, color = 'tab:orange', marker='o', label='Stair on times')
    plt.xlabel('Time (sec)')
    plt.ylabel('Stairs')
    plt.show()


with open(lt_path_c, 'w') as f:
    f.write("#include \"graph_lookup_table.h\"\n\n")
    f.write("const uint32_t stair_times[{}] = {{{}}};\n\n".format(graph_params["LED_CHANNELS"], np.array2string(stair_graph_x.astype(int), separator=', ')[1:-1]))
    f.write("const uint32_t fade_times_day[{}] = {{{}}};\n\n".format(graph_params["MAX_BRIGHTNESS_DAY"], np.array2string(fade_graph_d_x.astype(int), separator=', ')[1:-1]))
    f.write("const uint32_t fade_times_night[{}] = {{{}}};\n\n".format(graph_params["MAX_BRIGHTNESS_NIGHT"], np.array2string(fade_graph_n_x.astype(int), separator=', ')[1:-1]))

with open(lt_path_h, 'w') as f:
    f.write("#ifndef __GRAPH_LOOKUP_TABLE_H\n")
    f.write("#define __GRAPH_LOOKUP_TABLE_H\n\n")
    f.write("#include \"stdint.h\"\n\n")
    f.write("extern const uint32_t stair_times[{}];\n".format(graph_params["LED_CHANNELS"]))
    f.write("extern const uint32_t fade_times_day[{}];\n".format(graph_params["MAX_BRIGHTNESS_DAY"]))
    f.write("extern const uint32_t fade_times_night[{}];\n".format(graph_params["MAX_BRIGHTNESS_NIGHT"]))
    f.write("\n#endif // __GRAPH_LOOKUP_TABLE_H\n")