#pragma once
#include <vector>
#include <string>

// Representa un evento en la simulación (Entrada o Salida)
struct Evento {
	double tiempo;   // Instante en el que ocurre
	int id_vuelo;    // 0 para entrada, > 0 para salida del vuelo con ese ID
};

// Registro completo para el análisis posterior (Métricas)
struct RegistroHistorico {
	int id;
	int tipo; // Podrías usar un enum o int para el modelo de avión

	// Datos de entrada
	double x_in, y_in, z_in;
	double t_in;

	// Datos de salida planificada
	double x_sal_p, y_sal_p, z_sal_p;
	double t_sal_p;

	// Datos de la maniobra realizada
	int hay_conflicto; // 0: no, 1: si
	int resuelto; // 0: no, 1: si
	int tipo_maniobra; // 0: ninguna, 1: velocidad, 2: altitud, 3: dirección
	double magnitud;   // % sobre el rango permitido

	// Datos de salida real (se completan al procesar el evento de salida)
	double x_sal_r, y_sal_r, z_sal_r;
	double t_sal_r;

	int numAvionesActivos;
};
