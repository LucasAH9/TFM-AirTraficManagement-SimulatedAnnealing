# TFM-AirTraficManagement-SimulatedAnnealing
# Simulación y Optimización en Gestión de Tráfico Aéreo con Vuelo Libre: Recocido Simulado

Este repositorio contiene la implementación técnica del **Trabajo Fin de Máster (TFM)** para el **Máster en Ciencia de Datos** de la **Universidad Politécnica de Madrid (UPM)**. El proyecto aborda la detección y resolución automatizada de conflictos en el espacio aéreo bajo el paradigma de *Free Flight* (Vuelo Libre).

##  Autor y Tutor
* **Autor:** Lucas Andosilla Herráiz
* **Tutor:** Antonio Jiménez Martín
* **Institución:** Escuela Técnica Superior de Ingenieros Informáticos (ETSI Informáticos) - Universidad Politécnica de Madrid
* **Departamento:** Departamento de Inteligencia Artificial
* Proyecto financiado por la Agencia Estatal de Investigación (AEI) y el Ministerio de Ciencia, Innovación y Universidades (España), en el marco del Proyecto PID2024-155179NB-C22.

---

##  Resumen del Proyecto
La gestión tradicional del tráfico aéreo (ATM) divide el espacio de navegación en sectores estáticos vigilados manualmente. Con la introducción de nuevos paradigmas como el *Free Flight* (Vuelo Libre), las aeronaves adquieren la libertad de seleccionar y modificar sus trayectorias en tiempo real, incrementando drásticamente la complejidad del sistema y haciendo indispensable la automatización de la seguridad.

Este proyecto desarrolla una **metodología dual** en **C++**:
1.  **Motor de Simulación de Sucesos Discretos (SSD):** Gestiona y monitoriza el espacio aéreo basándose en hitos cronológicos discretos (como eventos de entrada y salida de aeronaves en el sector cúbico), optimizando el coste computacional.
2.  **Algoritmo de Optimización (Recocido Simulado):** Al detectarse un conflicto (intersección de los cilindros de seguridad de las aeronaves), esta metaheurística explora un espacio de soluciones basado en maniobras físicas para resolver el problema de manera segura y eficiente.

La evaluación de las soluciones se realiza mediante un enfoque **multi-objetivo** a través de una función de *fitness* ponderada que equilibra tres criterios:
* **$f_1$ - Magnitud de la maniobra:** Minimiza la agresividad y alteración del plan de vuelo.
* **$f_2$ - Retraso temporal:** Minimiza la desviación del tiempo de salida planificado.
* **$f_3$ - Coordinación espacial:** Minimiza la distancia euclídea respecto al punto de salida original para evitar transferir conflictos a los sectores adyacentes.

---

##  Estructura de Archivos del Proyecto
La base del código del simulador y la metaheurística está organizada de la siguiente manera:

* `TFM_LucasAndosilla_2026/` (o directorio raíz de código fuente):
    * `TFM_LucasAndosilla_2026.cpp` Archivo principal fuente: Contiene el bucle de eventos de la Simulación de Sucesos Discretos (SSD), la cola cronológica de eventos y la lógica principal del programa.
    * `Metaheuristica.h` : Implementación de la metaheurística de Recocido Simulado. Incluye las políticas de enfriamiento de temperatura, el criterio de parada y los generadores de entornos para la exploración de maniobras.
    * `AerialSet.cpp` / `AerialSet.h` : Implementación matemática encargada de evaluar las trayectorias e identificar colisiones tanto bajo condiciones estándar (cono de exclusión por pendientes tangenciales) como patológicas (con rotación de ejes).
    * `Aircraft.h` / `simulador_data.h`: Define las variables de las aeronaves / Define las variables de estado por instante de tiempo ($id_i, x_i, y_i, z_i, v_i, alpha_i$) y sus restricciones físicas operacionales ($vMin, vMax, zMin, zMax$, etc.).
    * `GraficosGenerator.py` : Genera una imagen con las gráficas de rendimiento de una ejecución del Recocido Simulado. A utilizar solo en Escenarios 1 y 2. Requiere de descomentar la sección de código correspondiente a la escritura en csv dentro de Metaheuristica.h. Ejecutar en entorno de python tras finalizar la ejecución de la simulación.

Los escenarios de validacion que incluyen conflictos frontales, colisiones diagonales y simulaciones con diferentes densidades de tráfico (bajo, medio, alto y situaciones de colapso) se incluyen en el archivo principal como métodos diferentes. Para su ejecucion basta con comentar/descomentar en Main() el método seleccionado.

---

##  Requisitos Tecnológicos
* **Compilador:** Compatible con estándar C++11 o superior (ej. `g++`, `clang++` o MSVC). Durante el desarrollo se ha empleado Visual Studio 2022.
* **Herramientas de construcción:** `CMake` (opcional, recomendado) o compilación directa vía terminal.
* **Entorno operativo:** Validado en entornos Windows/Linux a través de Git y GitHub Desktop para el control de versiones.

---

### 1. Clonar el repositorio o descargar en ZIP
```bash

git clone [https://github.com/tu-usuario/tu-repositorio.git](https://github.com/tu-usuario/tu-repositorio.git)
cd tu-repositorio

```
### 2. Abrir projecto en entorno de ejecucion 
Para la primera ejecucion se recomienda ejecutar con compilación.

### 3. Modificar según necesidades
Modificar variables de estado de Simulación o Recocido Simulado (variar trafico, variables de entorno de metaheurística, salidas por pantalla, etc)
