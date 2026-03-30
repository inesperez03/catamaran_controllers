# catamaran_controllers


## Controladores disponibles

Actualmente hay dos controladores:

* `thruster_test_controller`
* `body_force_controller`

---

## 1. thruster_test_controller

### Qué hace

Controlador simple para enviar fuerza directamente a cada thruster.

No hace ningún cálculo: lo que envías es lo que se aplica.

---

### Topic

```text
/thruster_test_controller/commands
```

---

### Tipo de mensaje

```text
std_msgs/msg/Float64MultiArray
```

---

### Ejemplo

```bash
ros2 topic pub /thruster_test_controller/commands std_msgs/msg/Float64MultiArray "{data: [-20.0, -10.0]}"
```

---

## 2. body_force_controller

Qué hace

Controlador que recibe un Wrench (fuerza + momento del robot) y calcula automáticamente cuánto debe empujar cada thruster.

Internamente:

usa el URDF para obtener la posición y orientación de cada thruster,

construye la Thruster Allocation Matrix (TAM),

en cada ciclo calcula las fuerzas de los thrusters a partir del wrench usando:

u = B⁻1 · τ

Donde:

τ es el Wrench (fuerza + momento del cuerpo),

B-1 es la inversa de la TAM,

u son las fuerzas de los thrusters.

---

### Topic

```text
/body_force/command
```

---

### Tipo de mensaje

```text
geometry_msgs/msg/Wrench
```

---

### Ejemplo

```bash
ros2 topic pub /body_force_command geometry_msgs/msg/Wrench "{force: {x: 0.0, y: 0.0, z: 0.0}, torque: {x: 0.0, y: 0.0, z: 20.0}}"