#pragma once
#ifndef TFM_LUCASANDOSILLA_2026_H
#define TFM_LUCASANDOSILLA_2026_H

#include <vector>
#include <string>
#include "aircraft_set.h"
#include "aerial_sector.h"
#include "simulador_data.h"



// --- Declaración de Funciones Globales del Simulador (Métodos en CamelCase) ---

// Calcula el instante del próximo arribo basándose en un proceso de Poisson con tasa lambda
double TiempoSiguienteEntrada(double lambda);

// Instancia y posiciona una aeronave de forma aleatoria en una de las 6 caras del sector cúbico
Aircraft CreateRandomAircraft(int idVuelo, AerialSector* sector);

// Escanea el espacio aéreo activo usando getRisk para verificar la presencia de conflictos
bool HayConflictos(Aircraft& avionA, const std::vector<Aircraft>& avionesActivos);

// Exporta todo el vector del histórico procesado a un archivo formateado en formato CSV
void ExportarACsv(const std::vector<RegistroHistorico>& historico, const std::string& nombreArchivo);

// Vuelca los datos consolidados del histórico por pantalla formateados en estructura estricta JSON
void ExportarAJson(const std::vector<RegistroHistorico>& historico);

#endif // TFM_LUCASANDOSILLA_2026_H