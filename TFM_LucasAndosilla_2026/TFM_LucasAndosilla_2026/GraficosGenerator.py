import pandas as pd
import matplotlib.pyplot as plt
import numpy as np

# 1. Cargar los datos unificados de C++
df = pd.read_csv("ProgresoMetaheuristica.csv")

# Obtener la lista de los IDs únicos de aviones que ejecutaron la MH
idAviones = df['IdVuelo'].unique()

# Generar una paleta con tantos colores únicos como aviones tengamos
coloresBase = plt.cm.get_cmap('rainbow', len(idAviones))

# CONFIGURACIÓN NUEVA: 4 filas, 1 sola columna. Ajustamos el tamaño para que sea estilizado en vertical
fig, axs = plt.subplots(4, 1, figsize=(11, 18))
fig.suptitle("Análisis Multivuelo del Rendimiento de la Metaheurística (MH)", fontsize=16, fontweight='bold')

# 2. Iterar por cada avión y superponer sus datos en las gráficas
for idx, IdVuelo in enumerate(idAviones):
    # Filtrar el sub-dataset exclusivo de este avión
    subDf = df[df['IdVuelo'] == IdVuelo]
    colorAvion = coloresBase(idx)
    etiqueta = f"Avión ID {IdVuelo}"

    # --- GRÁFICO 1: temperatura (Gráfico de Línea) ---
    axs[0].plot(subDf['iteracion'], subDf['temperatura'], color=colorAvion, label=etiqueta, linewidth=1.5)
    
    # --- GRÁFICO 2: Evolución de f* (Gráfico de Línea) ---
    axs[1].plot(subDf['iteracion'], subDf['mejorFitness'], color=colorAvion, label=etiqueta, linewidth=2)
    
    # --- GRÁFICO 3: Evolución de p(i) (CAMBIADO A: Gráfico de Línea) ---
    # Usamos un alpha intermedio para que si se cruzan las líneas de varios aviones se sigan distinguiendo
    axs[2].scatter(subDf['iteracion'], subDf['Probabilidad'], color=colorAvion, alpha=0.3, s=6, label=etiqueta)
    
    # --- GRÁFICO 4: Evolución de f sin asterisco (CAMBIADO A: Gráfico de Línea) ---
    axs[3].plot(subDf['iteracion'], subDf['candidatoFitness'], color=colorAvion, alpha=0.5, linewidth=1.0, label=etiqueta)

# 3. Formatear y añadir detalles estéticos a las 4 filas
titulos = [
    "1. Descenso de la temperatura (Enfriamiento)",
    "2. Evolución de f* (Convergencia del Óptimo)",
    "3. Evolución de la Probabilidad p(i) de Aceptación (Metrópolis)",
    "4. Evolución del Fitness de las Soluciones Evaluadas f (Exploración)"
]
ejesY = ["temperatura", "Fitness Mínimo f*", "Probabilidad p(i)", "Fitness del Candidato f"]

# Al ser una sola columna, iteramos directamente de 0 a 3 sobre el vector de ejes
for i in range(4):
    axs[i].set_title(titulos[i], fontweight='bold', fontsize=12)
    axs[i].set_xlabel("Iteración Válida")
    axs[i].set_ylabel(ejesY[i])
    axs[i].grid(True, linestyle='--', alpha=0.5)
    
    # Mostramos la leyenda en cada uno si no hay una saturación masiva de aviones
    if len(idAviones) <= 10:
        axs[i].legend(loc='best', fontsize='small')

# Ajustar márgenes verticales para que los títulos no colisionen con los ejes X del gráfico superior
plt.tight_layout()
plt.subplots_adjust(top=0.94)

# Guardar la imagen vertical en alta definición para la memoria del TFM
plt.savefig("Analisis_Multivuelo_MH_Vertical.png", dpi=300)
plt.show()