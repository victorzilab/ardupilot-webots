import tkinter as tk
from pymavlink import mavutil
import threading

# Criar conexão MAVLink
master = mavutil.mavlink_connection('udpin:0.0.0.0:14550')
master.wait_heartbeat()
print("Conexão estabelecida com o drone!")

# Criar janela principal
root = tk.Tk()
root.title("Controle Manual do Drone")

# Variáveis de controle
x_val = tk.IntVar(value=0)
y_val = tk.IntVar(value=0)
z_val = tk.IntVar(value=500)  # Neutro
r_val = tk.IntVar(value=0)

# Inicializar variáveis para o ponto inicial
initial_lat = None
initial_lon = None
initial_alt = None

# Função para resetar sliders
def reset_slider(event, slider, default_value):
    slider.set(default_value)

# Função para enviar controle manual
def send_manual_control():
    if master.target_system:
        master.mav.manual_control_send(
            master.target_system,
            x_val.get(),
            y_val.get(),
            z_val.get(),
            r_val.get(),
            0
        )
    root.after(100, send_manual_control)

# Funções para modos e controle de voo
def set_mode_guided():
    master.mav.set_mode_send(master.target_system, mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, 4)  # Guided mode
    print("Modo Guided ativado")

def set_mode_rtl():
    master.mav.set_mode_send(master.target_system, mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, 6)  # RTL mode
    print("Modo RTL ativado")

def set_mode_loiter():
    master.mav.set_mode_send(master.target_system, mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, 5)  # Loiter mode
    print("Modo Loiter ativado")

def takeoff():
    altitude = float(altitude_entry.get())  # Get altitude from input field
    master.mav.command_long_send(
        master.target_system, 
        master.target_component, 
        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 
        0,  # confirmation
        0, 0, 0, 0, 0, 0, altitude
    )
    print(f"Solicitação de decolagem para {altitude} metros")

def land():
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_NAV_LAND,
        0,  # confirmation
        0, 0, 0, 0, 0, 0, 0
    )
    print("Solicitação de aterrissagem")

# Funções para armar e desarmar o drone
def arm_drone():
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0,  # confirmation
        1,  # Armar o drone
        0, 0, 0, 0, 0, 0
    )
    print("Drone armado")

def disarm_drone():
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0,  # confirmation
        0,  # Desarmar o drone
        0, 0, 0, 0, 0, 0
    )
    print("Drone desarmado")

# Criar sliders
controls = [
    ("X (Pitch)", x_val, -1000, 1000, 0),
    ("Y (Roll)", y_val, -1000, 1000, 0),
    ("Z (Throttle)", z_val, 0, 1000, 500),
    ("R (Yaw)", r_val, -1000, 1000, 0)
]

for text, var, min_val, max_val, default_value in controls:
    tk.Label(root, text=text).pack()
    slider = tk.Scale(root, from_=min_val, to=max_val, variable=var, orient="horizontal")
    slider.pack()
    slider.bind("<ButtonRelease-1>", lambda event, s=slider, d=default_value: reset_slider(event, s, d))

# Campo para altitude
tk.Label(root, text="Altura para Decolagem (metros)").pack()
altitude_entry = tk.Entry(root)
altitude_entry.pack()
altitude_entry.insert(0, "10")  # Default altitude

# Botões para mudar modos
tk.Button(root, text="Modo Guided", command=set_mode_guided).pack()
tk.Button(root, text="Modo RTL", command=set_mode_rtl).pack()
tk.Button(root, text="Modo Loiter", command=set_mode_loiter).pack()

# Botões de decolagem e aterrissagem
tk.Button(root, text="Decolar", command=takeoff).pack()
tk.Button(root, text="Aterrissar", command=land).pack()

# Botões para armar e desarmar o drone
tk.Button(root, text="Armar Drone", command=arm_drone).pack()
tk.Button(root, text="Desarmar Drone", command=disarm_drone).pack()

# Labels para exibição das informações
attitude_label = tk.Label(root, text="Orientação: Pitch: 0 | Roll: 0 | Yaw: 0")
attitude_label.pack()
position_label = tk.Label(root, text="Posição: Lat: 0 | Lon: 0 | Alt: 0")
position_label.pack()

mode_label = tk.Label(root, text="Modo: -")
mode_label.pack()

angular_velocity_label = tk.Label(root, text="Velocidades Angulares: Pitch: 0 | Roll: 0 | Yaw: 0")
angular_velocity_label.pack()

relative_position_label = tk.Label(root, text="Posição Relativa: X: 0 | Y: 0 | Z: 0")
relative_position_label.pack()

# Atualizar dados do drone
def update_drone_data():
    global initial_lat, initial_lon, initial_alt

    msg_att = master.recv_match(type='ATTITUDE', blocking=False)
    if msg_att:
        attitude_label.config(text=f"Orientação: Pitch: {msg_att.pitch:.2f} | Roll: {msg_att.roll:.2f} | Yaw: {msg_att.yaw:.2f}")
        angular_velocity_label.config(text=f"Velocidades Angulares: Pitch: {msg_att.pitchspeed:.2f} | Roll: {msg_att.rollspeed:.2f} | Yaw: {msg_att.yawspeed:.2f}")

    msg_pos = master.recv_match(type='GLOBAL_POSITION_INT', blocking=False)
    if msg_pos:
        lat = msg_pos.lat / 1e7
        lon = msg_pos.lon / 1e7
        alt = msg_pos.alt / 1000

        # Atualiza a posição global
        position_label.config(text=f"Posição: Lat: {lat:.7f} | Lon: {lon:.7f} | Alt: {alt:.2f}m")

        # Se o ponto inicial não foi registrado, registra a posição inicial
        if initial_lat is None and initial_lon is None and initial_alt is None:
            initial_lat = lat
            initial_lon = lon
            initial_alt = alt

        # Posição relativa ao ponto inicial
        if initial_lat is not None and initial_lon is not None:
            rel_lat = lat - initial_lat
            rel_lon = lon - initial_lon
            rel_alt = alt - initial_alt
            relative_position_label.config(text=f"Posição Relativa: Lat: {rel_lat:.7f} | Lon: {rel_lon:.7f} | Alt: {rel_alt:.2f}m")

    msg_mode = master.recv_match(type='STATUSTEXT', blocking=False)
    if msg_mode:
        mode_label.config(text=f"Modo: {msg_mode.text}")

    root.after(500, update_drone_data)

# Iniciar threads
threading.Thread(target=send_manual_control, daemon=True).start()
root.after(500, update_drone_data)

# Rodar interface
tk.mainloop()
