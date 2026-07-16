#pragma once




#include <vector>
#include "aircraft_set.h"
#include "aerial_sector.h"
#include "TFM_LucasAndosilla_2026.h"
#include "tools.h"
#include <random>
#include <cmath>
#include <iostream>
#include <chrono>

using namespace std::chrono;

struct ManiobraCiclica {
    float valores[3] = { 0.0f, 0.0f, 0.0f }; // [0]: Velocidad, [1]: Altitud, [2]: Rumbo
};

struct RegistroProgreso {
    int iteracion;
    float temperatura;
    float fitnessActual;
    float mejorFitness;     // f* (Líneas)
    float candidatoFitness; // f (Puntos)
    float probabilidad;     // p(i) (Puntos)
    float maniobra;
    float magnitud;
};

class Metaheuristica {
private:

    int maxIterations;       // Número máximo de iteraciones del algoritmo
    float deltaRange; // Rango de dispersion de nuevas soluciones

    // temp
    float coolingRate;
    float initialTemp;
    float minTemp;
    int chainLength;

    //  Pesos de las Funciones Objetivo 
    float w1; // Peso: Minimizar magnitud de la maniobra
    float w2; // Peso: Minimizar riesgo de colisión (Inactiva)
    float w3; // Peso: Minimizar número de maniobras (Inactiva)
    float w4; // Peso: Minimizar retraso de tiempo
    float w5; // Peso: Minimizar distancia de desvío del punto de salida
    float w6; // Peso: Minimizar dispersión de maniobras (Inactiva)

    int k1 = 120;            // Bloques de espera para mejora de f*
    int k2 = 80;            // Bloques de espera para congelación (p(i))
    float e1 = 0.001f;      // Porcentaje mínimo de mejora requerido
    float e2 = 20.0f;       // Porcentaje mínimo de aceptación tolerado
    // --- VARIABLES DE ESTADO DEL CRITERIO (Requieren reinicio por cada avión) ---
    int stagesNoImprovement = 0;
    int stagesLowAcceptance = 0;
    int acceptedInCurrentStage = 0;
    float fitnessAtStageStart = 10000000000000000000000.0f;

public:
    // Constructor con inicialización de parámetros
    Metaheuristica(float temp, float cooling, float minTempe, int its, int chainL, float delta, float weight1, float weight2, float weight3, float weight4, float weight5, float weight6) {
        initialTemp = temp; coolingRate = cooling; minTemp = minTempe; chainLength = chainL; maxIterations = its; deltaRange = delta;
        w1 = weight1;
        w2 = 0.0f; // Inactiva por ahora
        w3 = 0.0f; // Inactiva por ahora
        w4 = weight4;
        w5 = weight5;
        w6 = 0.0f; // Inactiva por ahora
    };



    float EvaluateFitness(const ManiobraCiclica& maniobra, const Aircraft& avionOriginal, const std::vector<Aircraft>& avionesActivos, AerialSector* sectorAereo) {

        //  PREPARACIÓN Y PROYECCIÓN FÍSICA ---
        // Creamos un clon del avión para evaluar cómo afectaría la maniobra a su trayectoria
        Aircraft avionCandidato = avionOriginal;
        ApplyManeuver(avionCandidato, maniobra, avionOriginal);

      

        // Calculamos el nuevo punto de salida estimado tras aplicar la maniobra
        //getEstimatedOutPoint(&avionCandidato.xFin, &avionCandidato.yFin, sectorAereo, avionCandidato.angle, avionCandidato.x, avionCandidato.y);

        getEstimatedOutPoint(&avionCandidato.xFin, &avionCandidato.yFin, &avionCandidato.zFin, sectorAereo,
            avionCandidato.angle, avionCandidato.x, avionCandidato.y, avionCandidato.z);

        // 1. MINI-FUNCIÓN 1: MINIMIZAR MAGNITUD DE MANIOBRA (Activa) ---
        // Sumamos el valor absoluto de los componentes (como solo uno es distinto de cero, mide esa magnitud)
        float f1 = 0.0f;
        for (int i = 0; i < 3; i++) {
            f1 += std::abs(maniobra.valores[i]);
        }

        // 2. MINI-FUNCIÓN 2: MINIMIZAR COLISION RISK (Inactiva) ---
        float f2 = 0.0f; // Vacía. El bucle CallMH ya descarta si HayConflictos == true

        // 3. MINI-FUNCIÓN 3: MINIMIZAR NÚMERO DE MANIOBRAS (Inactiva) ---
        float f3 = 0.0f; // Vacía. Solo se ejecuta una maniobra cíclica por avión

        // 4. MINI-FUNCIÓN 4: MINIMIZAR RETRASO DE TIEMPO (Activa) ---
        // Calculamos el tiempo que tardará en recorrer la nueva trayectoria
                //float nuevaDistancia = distance(avionCandidato.x, avionCandidato.y, avionCandidato.xFin, avionCandidato.yFin);
                //float nuevoTFin = (avionCandidato.v > 0) ? (nuevaDistancia / avionCandidato.v) : 0.0f;
        float dx = avionCandidato.xFin - avionCandidato.x;
        float dy = avionCandidato.yFin - avionCandidato.y;
        float dz = avionCandidato.zFin - avionCandidato.z;
        float distancia3D = sqrt(dx * dx + dy * dy + dz * dz); // Distancia real recorrida por el espacio

        // El coste es la desviación absoluta respecto al tiempo original planificado (retraso o adelanto drástico)
        float nuevoTFin = (avionCandidato.v > 0) ? (distancia3D / avionCandidato.v) : 0.0f;
        float f4 = std::abs(nuevoTFin - avionOriginal.tFin);
                //float f4 = std::abs(nuevoTFin - avionOriginal.tFin);

        // 5. MINI-FUNCIÓN 5: MINIMIZAR DISTANCIA DE DESVÍO DE SALIDA (Activa) ---
        // Mide la distancia euclídea entre el punto de salida original y el nuevo punto de salida.
        // Al minimizar esto, evitamos que la metaheurística desvíe el avión de forma exagerada de su ruta planeada.
        float dxSalida = avionOriginal.xFin - avionCandidato.xFin;
        float dySalida = avionOriginal.yFin - avionCandidato.yFin;
        float dzSalida = avionOriginal.zFin - avionCandidato.zFin; // Desvío en el punto de salida tridimensional
        float f5 = sqrt(dxSalida * dxSalida + dySalida * dySalida + dzSalida * dzSalida);
                //float f5 = distance(avionOriginal.xFin, avionOriginal.yFin, avionCandidato.xFin, avionCandidato.yFin);

        // 6. MINI-FUNCIÓN 6: MINIMIZAR DISPERSIÓN DE MANIOBRAS (Inactiva) ---
        float f6 = 0.0f; // Vacía por el momento

        // --- CONSOLIDACIÓN PONDERADA ---
        // Suma ponderada final que el Recocido Simulado intentará minimizar
        float fitnessTotal = (w1 * f1) + (w2 * f2) + (w3 * f3) + (w4 * f4) + (w5 * f5) + (w6 * f6);
        //std::cout << "f1 magnitud: " << f1 << ", f4 delay: " << f4 << ", f5 distance: " << f5 << std::endl;
        return fitnessTotal;
    };

    float UpdateTemperature(float currentTemp, int iteration) {
        // Reducimos la temperatura únicamente al cumplir el bloque de la cadena de Markov
        if (iteration > 0 && iteration % chainLength == 0) {
            return currentTemp * coolingRate;
        }
        return currentTemp;
    }

    bool CheckTermination(int iteration, float mejorFitness, float currentTemperature) {
        // Evaluamos únicamente al completar un bloque completo de L pasos (y si ya hemos avanzado)
        if (iteration > 0 && iteration % chainLength == 0) {

            // CRITERIO 1: Verificar si f* ha mejorado significativamente en esta etapa
            float mejoraPorcentaje = 0.0f;
            if (fitnessAtStageStart > 0.0f && fitnessAtStageStart < 10000000000000000000000.0f) {
                mejoraPorcentaje = ((fitnessAtStageStart - mejorFitness) / fitnessAtStageStart) * 100.0f;
            }

            if (mejoraPorcentaje >= e1) {
                stagesNoImprovement = 0; // Hubo una mejora de calidad real, reseteamos el contador
            }
            else {
                stagesNoImprovement++;   // La mejora fue insignificante o nula
            }

            // CRITERIO 2: Verificar la fluidez térmica (Tasa de Aceptación del bloque)
            float tasaAceptacion = ((float)acceptedInCurrentStage / chainLength) * 100.0f;
            if (tasaAceptacion < e2) {
                stagesLowAcceptance++;   // El algoritmo está atrapado/solidificado
            }
            else {
                stagesLowAcceptance = 0;  // Aún posee suficiente movilidad estocástica
            }

            // Guardamos la foto fija de f* para la siguiente etapa y reiniciamos el contador de aceptados
            fitnessAtStageStart = mejorFitness;
            acceptedInCurrentStage = 0;
            // --- COMPROBACIÓN DE DETENCIÓN ---
            
            if (stagesNoImprovement >= k1) {
                std::cout << "  [STOP ADAPTATIVO] Criterio 1: f* estancado sin mejorar un "
                    << e1 << "% durante " << k1 << " etapas seguidas." << std::endl;
                return true; // Detiene el while
            }
            if (stagesLowAcceptance >= k2) {
                std::cout << "  [STOP ADAPTATIVO] Criterio 2: Sistema congelado. Tasa de aceptacion < "
                    << e2 << "% durante " << k2 << " etapas seguidas." << std::endl;
                return true; // Detiene el while
            }
            
        }

        return false; // El algoritmo puede continuar explorando
    }

    float CalibrateInitialTemperature(const Aircraft& avionOriginal, const std::vector<Aircraft>& avionesActivos, AerialSector* sectorAereo) {

        int numPilotos = 10000;        // Número de muestras estables
        int intentosSeguridad = 50000; // Evitar bucles infinitos en espacios colapsados

        std::vector<float> deltasPeores;

        static std::random_device rd;
        static std::mt19937 gen(rd());
        std::uniform_real_distribution<float> randMagn(-100.0f, 100.0f);

        // 1. Encontrar soluciones base válidas (libres de conflictos) explorando los 3 tipos de maniobras
        std::vector<ManiobraCiclica> basesValidas;
        std::vector<float> fitnessBases;

        // Repartimos los intentos de seguridad entre los 3 tipos de maniobra
        int maxIntentosPorManiobra = intentosSeguridad / 3;

        for (int man = 0; man < 3; ++man) {
            bool baseEncontrada = false;
            int intentosBase = 0;

            while (!baseEncontrada && intentosBase < maxIntentosPorManiobra) {
                intentosBase++;
                float magn = randMagn(gen);

                ManiobraCiclica solucionTemp;
                solucionTemp.valores[0] = 0.0f;
                solucionTemp.valores[1] = 0.0f;
                solucionTemp.valores[2] = 0.0f;
                solucionTemp.valores[man] = magn;

                Aircraft avionTest = avionOriginal;
                ApplyManeuver(avionTest, solucionTemp, avionOriginal);

                if (!HayConflictos(avionTest, avionesActivos)) {
                    basesValidas.push_back(solucionTemp);
                    fitnessBases.push_back(EvaluateFitness(solucionTemp, avionOriginal, avionesActivos, sectorAereo));
                    baseEncontrada = true;
                }
            }
        }

        // Si es físicamente imposible encontrar una solución libre de conflicto en ninguna maniobra
        if (basesValidas.empty()) {
            std::cout << "  [ALERTA] Espacio colapsado: No se encontraron bases seguras." << std::endl;
            return 100.0f; // Valor de rescate estándar
        }

        // 2. Lanzar el muestreo piloto para recolectar perturbaciones que EMPEOREN el escenario
        int muestrasRecogidas = 0;
        int intentosMuestreo = 0;

        // Para elegir aleatoriamente desde cuál base válida generar el vecino
        std::uniform_int_distribution<int> randBaseIdx(0, basesValidas.size() - 1);

        while (muestrasRecogidas < numPilotos && intentosMuestreo < intentosSeguridad) {
            intentosMuestreo++;

            // Elegimos una de las bases válidas descubiertas para explorar su vecindario
            int idx = randBaseIdx(gen);
            ManiobraCiclica candidato = GenerateNeighbor(basesValidas[idx]);

            Aircraft avionCandidato = avionOriginal;
            ApplyManeuver(avionCandidato, candidato, avionOriginal);

            // Solo nos interesan maniobras físicamente seguras
            if (HayConflictos(avionCandidato, avionesActivos)) {
                continue;
            }

            float fitnessCandidato = EvaluateFitness(candidato, avionOriginal, avionesActivos, sectorAereo);
            float delta = fitnessCandidato - fitnessBases[idx];

            // Buscamos estrictamente soluciones PEORES (delta de coste positivo)
            if (delta > 0.0f) {
                deltasPeores.push_back(delta);
                muestrasRecogidas++;
            }
        }

        // 3. Procesamiento matemático
        if (deltasPeores.empty()) {
            std::cout << "  [ALERTA] Espacio plano: No se encontraron peores soluciones." << std::endl;
            return 50.0f; // Valor por defecto si el espacio es tan plano que nada empeora
        }

        // Calcular estadísticas (Promedio, Max, Min)
        float sumaDeltas = 0.0f;
        float maxDelta = deltasPeores[0];
        float minDelta = deltasPeores[0];

        for (float d : deltasPeores) {
            sumaDeltas += d;
            if (d > maxDelta) maxDelta = d;
            if (d < minDelta) minDelta = d;
        }

        float deltaPromedio = sumaDeltas / deltasPeores.size();

        // Aplicación estricta de la ecuación de Metrópolis despejada para p = 0.9
        // AHORA BASADA EN EL PEOR ESCENARIO (maxDelta)
        float tCalculada = -maxDelta / std::log(0.9f);

        std::cout << "  [CALIBRACION T0] Muestras peores analizadas: " << deltasPeores.size() << std::endl
            << " | Min Delta: " << minDelta << " | Max Delta (Peor caso): " << maxDelta << std::endl
            << " | Delta Promedio: " << deltaPromedio << std::endl
            << " -> Temperatura Inicial Optima Calculada (p=0.9 para el peor caso): " << tCalculada << std::endl;

        return tCalculada;
    }

    // Con el rango definido, creamos una nueva solucion en el sistema ciclico de maniobras.
    ManiobraCiclica GenerateNeighbor(const ManiobraCiclica& solucionActual) {
        ManiobraCiclica vecino = solucionActual;

        static std::random_device rd;
        static std::mt19937 gen(rd());
        std::uniform_real_distribution<float> disDelta(-deltaRange, deltaRange);

        // 1. Identificar cuál es la maniobra activa actualmente (la que no es 0)
        int indexActivo = 0;
        for (int i = 0; i < 3; i++) {
            if (solucionActual.valores[i] != 0.0f) {
                indexActivo = i;
                break;
            }
        }

        // 2. Calcular el nuevo valor con el rango estipulado
        float delta = disDelta(gen);
        
        if (abs(delta) < 40) {
            //std::cout << "WRONG DELTA GENERATED: " << delta << std::endl;
        }

        float newValue = solucionActual.valores[indexActivo] + delta;
         //float newValue = solucionActual.valores[indexActivo] - deltaRange;
        //std::cout << "Delta: " <<delta << " valor actual "<< solucionActual.valores[indexActivo]<< " Generated next neighbour : " << newValue << std::endl;

        // Límite de magnitud máximo permitido  (entre -100 y 100)
        const float minMag = -100.0f;
        const float maxMag = 100.0f;

        // 3. Aplicar lógica de desbordamiento cíclico
        if (newValue < minMag) {
            float remainder = abs( newValue - minMag); // Lo que se pasa del límite por abajo
            vecino.valores[indexActivo] = 0.0f;  // Se anula la maniobra actual

            // Pasamos de manera cíclica a la maniobra anterior (hacia atrás)
            int siguienteIndex = (indexActivo - 1 + 3) % 3;
            vecino.valores[siguienteIndex] = maxMag - remainder; // Restamos del tope superior
        }
        else if (newValue > maxMag) {
            float remainder = newValue - maxMag; // Lo que se pasa del límite por arriba
            vecino.valores[indexActivo] = 0.0f;  // Se anula la maniobra actual

            // Pasamos de manera cíclica a la siguiente maniobra (hacia adelante)
            int siguienteIndex = (indexActivo + 1) % 3;
            vecino.valores[siguienteIndex] = minMag + remainder; // Sumamos al tope inferior
        }
        else {
            // Si no se sale de los rangos, se actualiza normalmente
            vecino.valores[indexActivo] = newValue;
        }
        //std::cout << "generated neighbourgh: [" << vecino.valores[0] << ", " << vecino.valores[1] << ", " << vecino.valores[2] <<std::endl;
        return vecino;
    }

    // Traduciomos la solucion a los valores reales de movimiento del avion, siguiendo las magnitudes de maniobra permitidas por el avion
    void ApplyManeuver(Aircraft& avion, const ManiobraCiclica& maniobra, const Aircraft& avionOriginal) {
        // Reseteamos el avión a su estado original antes de aplicar la nueva maniobra
        avion = avionOriginal;
        avion.idManeuvers = -1;
        //avion.pitchAngle = 0.0f;
        float allowedSpeedVar = 15.0f;
        float allowedHeightVar = avion.safetyHeight * 3;

        // [0]: Cambio de Velocidad
        if (maniobra.valores[0] != 0.0f) {
            float pct = maniobra.valores[0] / 100.0f;
            avion.v = (pct > 0) ? avionOriginal.v + pct * (allowedSpeedVar)
                : avionOriginal.v + pct * (allowedSpeedVar);
            avion.idManeuvers = 0;
        }
        // [1]: Cambio de Altitud
        else if (maniobra.valores[1] != 0.0f) {
            float pct = maniobra.valores[1] / 100.0f;
            avion.z = (pct > 0) ? avionOriginal.z + pct * (allowedHeightVar) + (avion.safetyHeight)
                : avionOriginal.z + pct * (allowedHeightVar) - (avion.safetyHeight);
            avion.idManeuvers = 1;
        }
        // [2]: Cambio de Rumbo (Ángulo)
        else if (maniobra.valores[2] != 0.0f) {
            float pct = maniobra.valores[2] / 100.0f;
            avion.angle = avionOriginal.angle + (pct * avionOriginal.maxAngle);
            avion.idManeuvers = 2;
        }
    }


    bool CallMH(Aircraft& nuevoAvion, const std::vector<Aircraft>& avionesActivos, AerialSector* sectorAereo) {
       
        auto a = high_resolution_clock::now();

        // Guardamos una copia del estado inicial sin maniobras del avión
        Aircraft avionOriginal = nuevoAvion;
        this->initialTemp = CalibrateInitialTemperature(avionOriginal, avionesActivos, sectorAereo);
        //this->initialTemp = 2350.0f;
        float initialDeltaRange = this->deltaRange;

        // Solución inicial: sin maniobra [0, 0, 0]
        ManiobraCiclica solucionActual;
        ManiobraCiclica mejorSolucion = solucionActual;
        float fitnessActual = 100000000000000000000000.0f;
        float mejorFitness = fitnessActual;

        float currentTemp = initialTemp;
        int iteration = 0;
        int totalAttempts = 0;
        int maxTotalAttemtps =750000;

        int numFailures = 0;
        int numSuccess = 0;

        this-> stagesNoImprovement = 0;
        this->stagesLowAcceptance = 0;
        this->acceptedInCurrentStage = 0;     // Transiciones aceptadas en el bloque actual
        this->fitnessAtStageStart = mejorFitness; // Foto de f* al iniciar el bloque

        std::vector<ManiobraCiclica> testedManeuvers;

        std::vector<RegistroProgreso> progresoLog;

        static std::random_device rd;
        static std::mt19937 gen(rd());
        std::uniform_real_distribution<float> disProb(0.0f, 1.0f);


        std::uniform_real_distribution<float> randMagn(-100.0f, 100.0f);
        std::uniform_int_distribution<int> randMan(0.0f, 2.0f);
        
        bool foundInit = false;
        ManiobraCiclica candidato;
        RegistroProgreso registro;
        // Bucle principal del Recocido Simulado
        while (!CheckTermination(iteration, mejorFitness, currentTemp) && totalAttempts < maxTotalAttemtps) {
            totalAttempts++;

            // 1. Generar nueva solución vecina de forma cíclica
            
            if (!foundInit) {
                
                int man = randMan(gen);
                int magn = randMagn(gen);
                solucionActual.valores[0] = 0.0f; solucionActual.valores[1] = 0.0f; solucionActual.valores[2] = 0.0f;
                solucionActual.valores[man] = magn;
                //std::cout << "TESTED VALUES: " << solucionActual.valores[0]  << ", " << solucionActual.valores[1] << ", " << solucionActual.valores[2] << ", " << std::endl;
                candidato = solucionActual;
                std::cout << "initial maneuver: [" << candidato.valores[0] << ", " << candidato.valores[1] << ", " << candidato.valores[2]
                    << "] aplicada sobre los valores: vel: " << nuevoAvion.v << " angle: " << nuevoAvion.angle << " zVar: " << nuevoAvion.z << std::endl;
            }
            else {
             candidato = GenerateNeighbor(solucionActual);
            }


            // 2. Aplicar la maniobra a un avión de prueba para verificar la física
            Aircraft avionCandidato = nuevoAvion;
            ApplyManeuver(avionCandidato, candidato, avionOriginal);

            // 3. Regla 3: Si la maniobra provoca conflicto, se DESCARTA automáticamente
            
            if (HayConflictos(avionCandidato, avionesActivos)) {
                numFailures++;
                continue;
            }
            numSuccess++;
            foundInit = true;
            testedManeuvers.push_back(candidato);
            // 4. Si es segura, la evaluamos 
            float candidatoFitness = EvaluateFitness(candidato, avionOriginal, avionesActivos, sectorAereo);

            // 5. Criterio de Aceptación del Recocido Simulado (Metropolis)
            float deltaCosto = candidatoFitness - fitnessActual;
            float probabilidad = 1.0f;
            bool transitionAccepted = false;

            if (deltaCosto < 0.0f) {
                // El candidato es mejor, se acepta directamente
                solucionActual = candidato;
                fitnessActual = candidatoFitness;
                transitionAccepted = true;
                // Guardamos la mejor solución absoluta libre de conflictos
                if (candidatoFitness < mejorFitness) {
                    mejorSolucion = candidato;
                    mejorFitness = candidatoFitness;
                }
            }
            else {
                // El candidato es peor, se acepta con una probabilidad termodinámica
                probabilidad = std::exp(-deltaCosto / currentTemp);
                if (disProb(gen) < probabilidad) {
                    solucionActual = candidato;
                    fitnessActual = candidatoFitness;
                    transitionAccepted = true;
                }
            }
            if (transitionAccepted) {
                acceptedInCurrentStage++;
            }
            float maniobra = -1;
            float magnitud = 0;
            for (int i = 0; i < 3; i++) {
                if (candidato.valores[i] != 0) {
                    maniobra = i;
                    magnitud = candidato.valores[i];
                    break;
                }
            }

            
            registro.iteracion = iteration;
            registro.temperatura = currentTemp;
            registro.fitnessActual = fitnessActual;
            registro.mejorFitness = mejorFitness;         // f*
            registro.candidatoFitness = candidatoFitness; // f
            registro.probabilidad = probabilidad; // p(i)
            registro.maniobra = maniobra;
            registro.magnitud = magnitud;
            progresoLog.push_back(registro);

            // 6. Actualizar Temperatura y contador (Regla 6)
            currentTemp = UpdateTemperature(currentTemp, iteration);
            iteration++;
        }
        if (totalAttempts >= maxTotalAttemtps) {
            std::cout << "max attempts reached, concluding MH" << std::endl;
        }

        std::cout << "Attempts: " << totalAttempts <<", iterations: "<< iteration <<", Failures: "<< numFailures <<", Successes: " << numSuccess << std::endl;
        std::cout << "TESTED VALUES: " << std::endl;
        std::cout << "best fitness: " << mejorFitness<< std::endl;

 
        
        // ESCRITURA EN CSV 

        /*
        // Al salir del bucle, volcamos el progreso de forma ultra rápida en un archivo
        std::ofstream archivoProgreso("ProgresoMetaheuristica.csv", std::ios::trunc);

        if (archivoProgreso.is_open()) {
            archivoProgreso << "IdVuelo,iteracion,temperatura,mejorFitness,candidatoFitness,fitnessActual,Probabilidad,Maniobra,Magnitud\n";
                
            for (const auto& reg : progresoLog) {
                // Añadimos nuevoAvion.id como primera columna
                archivoProgreso << nuevoAvion.id << ","
                    << reg.iteracion << ","
                    << reg.temperatura << ","
                    << reg.mejorFitness << ","
                    << reg.candidatoFitness << ","
                    << reg.fitnessActual << ","
                    << reg.probabilidad << ","
                    << reg.maniobra << ","
                    << reg.magnitud << "\n";

            }
            archivoProgreso.close();
        }
        */

        std::cout << "Final maneuver: [" << mejorSolucion.valores[0] << ", " << mejorSolucion.valores[1] << ", " << mejorSolucion.valores[2]
            << "] aplicada sobre los valores: vel: " << nuevoAvion.v << " angle: " << nuevoAvion.angle <<" zVar: " << nuevoAvion.z << std::endl;

        // Al terminar el algoritmo, aplicamos la mejor maniobra encontrada al avión real
        ApplyManeuver(nuevoAvion, mejorSolucion, avionOriginal);

        // Recalculamos su salida definitiva en el sector aéreo
        getEstimatedOutPoint(&nuevoAvion.xFin, &nuevoAvion.yFin, &nuevoAvion.zFin, sectorAereo,
            nuevoAvion.angle, nuevoAvion.x, nuevoAvion.y, nuevoAvion.z);

        std::cout << "after maneuver: [" << mejorSolucion.valores[0] << ", " << mejorSolucion.valores[1] <<", " << mejorSolucion.valores[2] 
            << "] con resultado de: vel: " << nuevoAvion.v << " angle: " << nuevoAvion.angle << " zVar: " << nuevoAvion.z << std::endl;

   /*
        for (size_t i = 0; i < testedManeuvers.size(); i++) {
            std::cout << "Maneuver values: ["
                << testedManeuvers[i].valores[0] << ", "
                << testedManeuvers[i].valores[1] << ", "
                << testedManeuvers[i].valores[2] << "]\n";
        }
        */
        // Devolvemos true si la solución final mantiene el cielo seguro
        auto b = high_resolution_clock::now();
        std::cout << "timer duration: " << duration_cast<seconds>(b - a).count() << " seconds" << std::endl;
        return !HayConflictos(nuevoAvion, avionesActivos);
    }
};


 