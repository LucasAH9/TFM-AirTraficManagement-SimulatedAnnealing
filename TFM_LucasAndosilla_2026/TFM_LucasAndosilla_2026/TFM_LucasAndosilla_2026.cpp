
#include <iostream>
#include <vector>
#include <cmath>
#include <random>
#include <algorithm>
#include <fstream>
#include <iomanip> 

#include "aircraft_set.h"
#include "aerial_sector.h"
#include "tools.h"
#include "simulador_data.h"
#include "TFM_LucasAndosilla_2026.h"
#include "Metaheuristica.h"

using namespace std;



double tiempoSiguienteEntrada(double lambda) {
	static std::random_device rd;
	static std::mt19937 gen(rd());
	std::uniform_real_distribution<float> randtime(5, lambda);
	int time = randtime(gen);

	return time;
}

Aircraft createRandomAircraft(int id_vuelo, AerialSector* sector) {
	Aircraft nuevoAvion;

	// Generador de números aleatorios moderno de C++
	static std::random_device rd;
	static std::mt19937 gen(rd());

	nuevoAvion.id = id_vuelo;

	// 1. Extraer los límites (Bounding Box implícito) del AerialSector
	float xMin = sector->pointsX[0], xMax = sector->pointsX[0];
	float yMin = sector->pointsY[0], yMax = sector->pointsY[0];

	for (int i = 1; i < sector->nPoints; i++) {
		if (sector->pointsX[i] < xMin) xMin = sector->pointsX[i];
		if (sector->pointsX[i] > xMax) xMax = sector->pointsX[i];
		if (sector->pointsY[i] < yMin) yMin = sector->pointsY[i];
		if (sector->pointsY[i] > yMax) yMax = sector->pointsY[i];
	}

	// Límites de altitud según tus rangos de TFM
	float zMin = 8.0f;
	float zMax = 15.0f;

	// 2. Elegir una cara aleatoria (0 a 5)
	// 0: Oeste (xMin), 1: Este (xMax), 2: Sur (yMin), 3: Norte (yMax), 4: Suelo (zMin), 5: Techo (zMax)
	std::uniform_int_distribution<> disCara(0, 5);
	int cara = disCara(gen);

	std::uniform_real_distribution<float> disX(xMin, xMax);
	std::uniform_real_distribution<float> disY(yMin, yMax);
	std::uniform_real_distribution<float> disZ(zMin, zMax);

	// 3. Posicionar el avión en la cara elegida
	switch (cara) {
	case 0: nuevoAvion.x = xMin; nuevoAvion.y = disY(gen); nuevoAvion.z = disZ(gen); break;
	case 1: nuevoAvion.x = xMax; nuevoAvion.y = disY(gen); nuevoAvion.z = disZ(gen); break;
	case 2: nuevoAvion.x = disX(gen); nuevoAvion.y = yMin; nuevoAvion.z = disZ(gen); break;
	case 3: nuevoAvion.x = disX(gen); nuevoAvion.y = yMax; nuevoAvion.z = disZ(gen); break;
	case 4: nuevoAvion.x = disX(gen); nuevoAvion.y = disY(gen); nuevoAvion.z = zMin; break;
	case 5: nuevoAvion.x = disX(gen); nuevoAvion.y = disY(gen); nuevoAvion.z = zMax; break;
	}

	// Guardar posición inicial
	nuevoAvion.xIni = nuevoAvion.x;
	nuevoAvion.yIni = nuevoAvion.y;
	nuevoAvion.zIni = nuevoAvion.z;

	// 4. Asignar trayectoria hacia el centro del sector
	float centroX = (xMax + xMin) / 2.0f;
	float centroY = (yMax + yMin) / 2.0f;

	float anguloAlCentro = atan2(centroY - nuevoAvion.y, centroX - nuevoAvion.x);
	// Variación aleatoria de +/- 20 grados para no ir todos al centro exacto
	std::uniform_real_distribution<float> disVar(-0.35f, 0.35f);
	nuevoAvion.angle = anguloAlCentro + disVar(gen);
	nuevoAvion.angleIni = nuevoAvion.angle;

	// 5. Parámetros físicos
	std::uniform_real_distribution<float> disVel(15.0f, 25.0f);
	nuevoAvion.v = disVel(gen) *0.2;
	nuevoAvion.vIni = nuevoAvion.v;
	nuevoAvion.vMin = 14.0f;
	nuevoAvion.vMax = 25.0f;

	nuevoAvion.zMin = zMin;
	nuevoAvion.zMax = zMax;

	// Si no entró por arriba o abajo, le damos una altura aleatoria en el rango
	if (cara < 4) {
		std::uniform_real_distribution<float> disZAlt(zMin, zMax);
		nuevoAvion.z = disZAlt(gen);
	}

	nuevoAvion.safetyRad = 4.0234f;			// 2.5 milla aereas = 4.023 km
	nuevoAvion.safetyHeight = 0.1524f;		// 500 pies de altitud = 0.1524 km
	nuevoAvion.maxAngle = PI / 8.0f;
	nuevoAvion.idManeuvers = -1;

	// 6. Calcular punto de salida estimado usando tu función original
	getEstimatedOutPoint(&nuevoAvion.xFin, &nuevoAvion.yFin, &nuevoAvion.zFin, sector,
		nuevoAvion.angle, nuevoAvion.x, nuevoAvion.y, nuevoAvion.z);

	// 7. Tiempos
	nuevoAvion.t = 0.0f;
	float distanciaTotal = distance(nuevoAvion.xIni, nuevoAvion.yIni, nuevoAvion.xFin, nuevoAvion.yFin);

	// Evitar división por cero si la velocidad es anómala
	nuevoAvion.tFin = (nuevoAvion.v > 0) ? (distanciaTotal / nuevoAvion.v) : 0.0f;

	return nuevoAvion;
}


bool HayConflictos(Aircraft& avionA, const std::vector<Aircraft>& avionesActivos) {
	float horRisk = 0.0f;
	float verRisk = 0.0f;

	for (const auto& avionB : avionesActivos) {
		// 1. Regla de oro: No compararse consigo mismo
		if (avionA.id == avionB.id) continue;

		// 2. Creamos el AircraftSet temporal para la pareja bajo análisis
		Aircraft pareja[2] = { avionA, avionB };
		AircraftSet conjuntoTemporal;

		// initAS calcula internamente las matrices L y G de tangencia para estos dos aviones
		initAS(&conjuntoTemporal, pareja, 2);

		// getRisk calcula la pendiente relativa real (horRisk) y la invasión vertical (verRisk)
		getRisk(&horRisk, &verRisk, &conjuntoTemporal, 0, 1, 0.0f, 0.0f, NM, NM);

		// Extraemos los límites del cono de colisión calculados previamente por initAS
		float lVal = conjuntoTemporal.L[0][1];
		float gVal = conjuntoTemporal.G[0][1];

		// Liberamos la memoria dinámica del conjunto de inmediato
		freeAS(&conjuntoTemporal);

		// 3. CONDICIÓN HORIZONTAL: Comprobar si la pendiente relativa cae DENTRO del cono de colisión
		float minLimit = std::min(lVal, gVal);
		float maxLimit = std::max(lVal, gVal);
		bool hasHorConflict = (horRisk >= minLimit && horRisk <= maxLimit);

		// 4. CONDICIÓN VERTICAL: Hay conflicto si el valor de invasión de la separación es positivo
		bool hasVerConflict = (verRisk > 0.0f);

		// 5. RESOLUCIÓN 3D: Existe riesgo real únicamente si coinciden AMBOS conflictos a la vez (AND)
		if (hasHorConflict && hasVerConflict) {
			return true;
		}
	}
	return false;
}


void ActualizarPosicionesAviones(std::vector<Aircraft>& avionesActivos, double tSim) {
	for (auto& avion : avionesActivos) {
		// Calcular el diferencial de tiempo desde que el avión se actualizó por última vez
		float deltaT = (float)(tSim - avion.t);

		// Solo actualizamos si realmente ha avanzado el tiempo
		if (deltaT > 0.0001f) {
			// Ecuaciones de movimiento rectilíneo uniforme (MRU)
			avion.x += avion.v * cos(avion.angle) * deltaT;
			avion.y += avion.v * sin(avion.angle) * deltaT;

			// Al ser teleport instantáneo, Z no varía durante el vuelo rectilíneo, 
			// pero su posición X e Y se han desplazado de forma continua.

			// Sincronizamos el reloj interno del avión con el tiempo de simulación actual
			avion.t = (float)tSim;
		}
	}
}

// Exporta el histórico a un archivo CSV
void ExportarACsv(const std::vector<RegistroHistorico>& Historico, const std::string& NombreArchivo) {
	std::ofstream Archivo(NombreArchivo);

	if (!Archivo.is_open()) {
		std::cerr << "Error: No se pudo crear el archivo CSV." << std::endl;
		return;
	}

	// Cabecera del CSV
	Archivo << "IdVuelo,TiempoEntrada,X_In,Y_In,Z_In,TiempoSalidaPlan,X_SalP,Y_SalP,Z_SalP,"
		<< "HayConflicto,Resuelto,TipoManiobra,Magnitud,TiempoSalidaReal,X_SalR,Y_SalR,Z_SalR, NumAvionesAct\n";

	// Datos
	Archivo << std::fixed << std::setprecision(4); // 4 decimales para precisión
	for (const auto& Reg : Historico) {
		Archivo << Reg.id << "," << Reg.t_in << "," << Reg.x_in << "," << Reg.y_in << "," << Reg.z_in << ","
			<< Reg.t_sal_p << "," << Reg.x_sal_p << "," << Reg.y_sal_p << "," << Reg.z_sal_p << ","
			<< Reg.hay_conflicto << "," << Reg.resuelto << ","
			<< Reg.tipo_maniobra << "," << Reg.magnitud << ","
			<< Reg.t_sal_r << "," << Reg.x_sal_r << "," << Reg.y_sal_r << "," << Reg.z_sal_r <<","<<Reg.numAvionesActivos << "\n";
	}

	Archivo.close();
	std::cout << "[INFO] Archivo CSV generado exitosamente: " << NombreArchivo << std::endl;
}

// Muestra el histórico en formato JSON por consola (o archivo)
void ExportarAJson(const std::vector<RegistroHistorico>& Historico) {
	std::cout << "\n--- REGISTRO FINAL (JSON) ---\n";
	std::cout << "[\n";

	for (size_t i = 0; i < Historico.size(); ++i) {
		const auto& Reg = Historico[i];
		std::cout << "  {\n";
		std::cout << "    \"id\": " << Reg.id << ",\n";
		std::cout << "    \"tiempos\": { \"entrada\": " << Reg.t_in << ", \"salidaPlan\": " << Reg.t_sal_p << ", \"salidaReal\": " << Reg.t_sal_r << " },\n";
		std::cout << "    \"posicionEntrada\": { \"x\": " << Reg.x_in << ", \"y\": " << Reg.y_in << ", \"z\": " << Reg.z_in << " },\n";
		std::cout << "    \"posicionSalidaReal\": { \"x\": " << Reg.x_sal_r << ", \"y\": " << Reg.y_sal_r << ", \"z\": " << Reg.z_sal_r << " },\n";
		std::cout << "    \"maniobra\": { \"tipo\": " << Reg.tipo_maniobra << ", \"magnitud\": " << Reg.magnitud << " }\n";
		std::cout << "  }" << (i == Historico.size() - 1 ? "" : ",") << "\n";
	}

	std::cout << "]\n";
	std::cout << "----------------------------\n";
}

Metaheuristica initMH() {
	float mhTempInicial = 100.0f;
	float mhCoolingRate = 0.95f;
	float mhMinTemp = 0.001f;
	int mhMaxIteraciones = 5000;
	int mhCadenaMarkov = 500;
	float mhDeltaRange = 10.0f;        // Rango de vecindad de la maniobra (+/- 50%)

	// Pesos específicos de la función agregada (camelCase)
	float w1 = 1.0f;  // Magnitud
	float w2 = 0.0f;  // Inactiva
	float w3 = 0.0f;  // Inactiva
	float w4 = 1.0f;  // Retraso temporal
	float w5 = 1.0f;  // Desvío espacial
	float w6 = 0.0f;  // Inactiva

	Metaheuristica mh(mhTempInicial, mhCoolingRate, mhMinTemp, mhMaxIteraciones, mhCadenaMarkov, mhDeltaRange, w1, w2, w3, w4, w5, w6);

	return mh;

}

int Simulacion() {
	// Varaibles
	double tSim = 0.0;        // Reloj de la simulación
	int n_aviones_inst = 0;    // Contador de aviones activos en el espacio aéreo
	double H = 150000;         // Horizonte de simulación 
	double lambda = 30;    // tasa de llegada, tiempo aleatorio entre 5 y lambda		-------------------------- MODIFICAR TRAFICO
	int proximoIdVuelo = 1;

	float puntosX[] = { 0.0f, 1000.0f, 1000.0f, 0.0f };
	float puntosY[] = { 0.0f, 0.0f, 1000.0f, 1000.0f };
	AerialSector* sectorAereo = new AerialSector;
	initAerialSector(sectorAereo, puntosX, puntosY, 4);

	// Estructuras de datos
	vector<Evento> listaEventos;
	vector<RegistroHistorico> historico;
	vector<Aircraft> avionesActivos;


	Metaheuristica mh = initMH();

	// --- Bucle ---
	listaEventos.push_back({ 0.0, 0 });

	while (!listaEventos.empty()) {

		// 1. Encontrar el evento más cercano en el tiempo
		auto it_evento = std::min_element(listaEventos.begin(), listaEventos.end(),
			[](const Evento& a, const Evento& b) { return a.tiempo < b.tiempo; });

		Evento eventoActual = *it_evento;
		listaEventos.erase(it_evento); // Lo sacamos de la lista

		// 2. Saltar el reloj al tiempo del evento
		tSim = eventoActual.tiempo;
		ActualizarPosicionesAviones(avionesActivos, tSim);

		// 3. Procesar el evento
		if (eventoActual.id_vuelo == 0) {
			// --- EVENTO DE ENTRADA ---
			int conflicto = 0;
			int resuelto = 0;
			RegistroHistorico registro = {};

			// A. Crear el avión y añadirlo a activos (Llamaremos a tu lógica de generación)
			Aircraft nuevoAvion = createRandomAircraft(proximoIdVuelo, sectorAereo);
			//avionesActivos.push_back(nuevoAvion);
			
			std::cout << "[T=" << tSim << "] Entrada de vuelo ID: " << nuevoAvion.id << " en posición (" << nuevoAvion.xIni << ", " << nuevoAvion.yIni << ")" << std::endl;
			std::cout << "[T=" << tSim << "] IDs Activos: [ ";
			for (size_t i = 0; i < avionesActivos.size(); ++i) {
				std::cout << avionesActivos[i].id << (i == avionesActivos.size() - 1 ? "" : ", ");
			}
			std::cout << " ]" << std::endl;

			registro.numAvionesActivos = avionesActivos.size();

			getEstimatedOutPoint(&nuevoAvion.xFin, &nuevoAvion.yFin, &nuevoAvion.zFin, sectorAereo,
				nuevoAvion.angle, nuevoAvion.x, nuevoAvion.y, nuevoAvion.z);

			//std::cout << "  Estimated out point: " << nuevoAvion.xFin << ", " << nuevoAvion.yFin << ", " << nuevoAvion.zFin << std::endl;

			float dxReal = nuevoAvion.xFin - nuevoAvion.x;
			float dyReal = nuevoAvion.yFin - nuevoAvion.y;
			float dzReal = nuevoAvion.zFin - nuevoAvion.z;
			float distanciaReal3D = sqrt(dxReal * dxReal + dyReal * dyReal + dzReal * dzReal);

			float tiempoVuelo = distanciaReal3D / nuevoAvion.v;
			float instanteSalida = (float)tSim + tiempoVuelo;
			nuevoAvion.tFin = instanteSalida;
			registro.t_sal_p = instanteSalida;
			registro.x_sal_p = nuevoAvion.xFin; registro.y_sal_p = nuevoAvion.yFin; registro.z_sal_p = nuevoAvion.zFin;

			// Evaluar la colisión provocada artificialmente
			if (HayConflictos(nuevoAvion, avionesActivos)) {
				std::cout << "  ! CONFLICTO FRONTAL DETECTADO para avion " << nuevoAvion.id << ". Invocando Metaheuristica..." << std::endl;
				conflicto = true;

				// Llamada al Recocido Simulado 
				bool estadoConflicto = mh.CallMH(nuevoAvion, avionesActivos, sectorAereo);

				if (estadoConflicto) {
					std::cout << "  [OK] Colision frontal resuelta exitosamente para avion " << nuevoAvion.id << std::endl;
					resuelto = true;
				}
			}
			else {
				std::cout << "  + Trayectoria inicial segura para avion " << nuevoAvion.id << std::endl;
			}

			// C. Proyectar salida definitiva post-optimización de la MH
			getEstimatedOutPoint(&nuevoAvion.xFin, &nuevoAvion.yFin, &nuevoAvion.zFin, sectorAereo,
				nuevoAvion.angle, nuevoAvion.x, nuevoAvion.y, nuevoAvion.z);

			std::cout << "  Estimated out point: " << nuevoAvion.xFin << ", " << nuevoAvion.yFin << ", " << nuevoAvion.zFin << std::endl;

			dxReal = nuevoAvion.xFin - nuevoAvion.x;
			dyReal = nuevoAvion.yFin - nuevoAvion.y;
			dzReal = nuevoAvion.zFin - nuevoAvion.z;
			distanciaReal3D = sqrt(dxReal * dxReal + dyReal * dyReal + dzReal * dzReal);

			tiempoVuelo = distanciaReal3D / nuevoAvion.v;
			instanteSalida = (float)tSim + tiempoVuelo;
			std::cout << "  Estimated out dist " << distanciaReal3D << ", tardando " << tiempoVuelo << ", saliendo en instante " << instanteSalida << std::endl;
			// D. Guardar métricas de test en el Histórico
			
			registro.id = nuevoAvion.id;
			registro.t_in = tSim;
			registro.x_in = nuevoAvion.x; registro.y_in = nuevoAvion.y; registro.z_in = nuevoAvion.z;

			registro.tipo_maniobra = nuevoAvion.idManeuvers;
			registro.hay_conflicto = conflicto ? 1 : 0;
			registro.resuelto = resuelto ? 1 : 0;
			historico.push_back(registro);

			// E. Programar evento de salida real usando el ID positivo real del avión
			avionesActivos.push_back(nuevoAvion);
			listaEventos.push_back({ (double)instanteSalida, nuevoAvion.id });

			// F. Programar la siguiente entrada si no hemos superado el Horizonte
			double ProximoTiempoLlegada = tSim + tiempoSiguienteEntrada(lambda);
			if (ProximoTiempoLlegada <= H) {
				listaEventos.push_back({ ProximoTiempoLlegada, 0 });
			}
			proximoIdVuelo++;

		}
		else {
			// --- GESTIÓN DE SALIDA ---

			// Buscar el avión para cerrar su ciclo de vida
			for (auto It = avionesActivos.begin(); It != avionesActivos.end(); ++It) {
				if (It->id == eventoActual.id_vuelo) {
					// Actualizar datos reales en el histórico
					std::cout << "[T=" << tSim << "] Evento: Salida del Vuelo ID " << eventoActual.id_vuelo << " por punto " << It->xFin << ", " << It->yFin << ", " << It->z << std::endl;

					for (auto& Reg : historico) {
						if (Reg.id == It->id) {
							Reg.t_sal_r = tSim;
							Reg.x_sal_r = It->xFin;
							Reg.y_sal_r = It->yFin;
							Reg.z_sal_r = It->zFin;
							break;
						}
					}
					// Eliminar de los aviones que están actualmente en el sector
					avionesActivos.erase(It);
					break;
				}
			}
		}
	}
	std::cout << "--- Simulación Finalizada ---" << std::endl;
	std::cout << "Vuelos totales registrados: " << historico.size() << std::endl;

	ExportarACsv(historico, "historicoCsv.csv");
	//ExportarAJson(historico);
	return 0;
}

// escenario 1
int MHTest() {
	// Variables de estado del simulador 
	double tSim = 0.0;        // Reloj de la simulación
	int nAvionesInst = 0;    // Contador de aviones activos en el espacio aéreo
	double h = 200;          // Horizonte de simulación lo suficientemente amplio

	// Definición del sector de 1000x1000
	float puntosX[] = { 0.0f, 10000.0f, 10000.0f, 0.0f };
	float puntosY[] = { 0.0f, 0.0f, 10000.0f, 10000.0f };
	AerialSector* sectorAereo = new AerialSector;
	initAerialSector(sectorAereo, puntosX, puntosY, 4);

	// Estructuras de datos 
	vector<Evento> listaEventos;
	vector<RegistroHistorico> historico;
	vector<Aircraft> avionesActivos;

	Metaheuristica mh = initMH();


	// --- INYECCIÓN MANUAL DE EVENTOS DE TEST ---
	// Inyección manual de los 5 eventos de test en el motor
	listaEventos.push_back({ 0.0, -1 });
	listaEventos.push_back({ 1.0, -2 });
	listaEventos.push_back({ 2.0, -3 });
	listaEventos.push_back({ 3.0, -4 });
	listaEventos.push_back({ 4.0, -5 }); // Este activará la Metaheurística

	while (!listaEventos.empty()) {

		// 1. Encontrar el evento más cercano en el tiempo
		auto itEvento = std::min_element(listaEventos.begin(), listaEventos.end(),
			[](const Evento& a, const Evento& b) { return a.tiempo < b.tiempo; });

		Evento eventoActual = *itEvento;
		listaEventos.erase(itEvento);

		// 2. Saltar el reloj al tiempo del evento
		tSim = eventoActual.tiempo;
		ActualizarPosicionesAviones(avionesActivos, tSim);

		// 3. Procesar el evento
		if (eventoActual.id_vuelo < 0) {
			bool conflicto = false;
			bool resuelto = false;
			Aircraft nuevoAvion;

			// --- PARÁMETROS COMUNES DE RENDIMIENTO (LENTOS) ---
			nuevoAvion.t = tSim;
			nuevoAvion.v = 150.0f;
			nuevoAvion.vIni = nuevoAvion.v;
			nuevoAvion.vMin = 100.0f;
			nuevoAvion.vMax = 200.0f;
			nuevoAvion.zMin = 10.0f;
			nuevoAvion.zMax = 15.0f;
			nuevoAvion.maxAngle = (float)PI / 16.0f;  //pi/16 = 45º de POV
			//nuevoAvion.pitchAngle = 0.0f;
			nuevoAvion.safetyRad = 4.0234f;
			nuevoAvion.safetyHeight = 0.1524f;
			nuevoAvion.idManeuvers = -1;
			nuevoAvion.xFin = 0.0f;
			nuevoAvion.yFin = 0.0f;

			// --- CONFIGURACIÓN GEOMÉTRICA INDIVIDUAL ---
			if (eventoActual.id_vuelo == -1) {
				nuevoAvion.id = 1;
				nuevoAvion.x = 10.0f;   nuevoAvion.y = 2000.0f; nuevoAvion.z = 15.0f;
				nuevoAvion.angle = 0.0f; // Trayectoria: Hacia el ESTE (Derecha)
			}
			else if (eventoActual.id_vuelo == -2) {
				nuevoAvion.id = 2;
				nuevoAvion.x = 9999.0f; nuevoAvion.y = 8000.0f; nuevoAvion.z = 10.0f;
				nuevoAvion.angle = (float)PI; // Trayectoria: Hacia el OESTE (Izquierda)
			}
			else if (eventoActual.id_vuelo == -3) {
				nuevoAvion.id = 3;
				nuevoAvion.x = 2000.0f; nuevoAvion.y = 10.0f;   nuevoAvion.z = 18.0f;
				nuevoAvion.angle = (float)PI / 2.0f; // Trayectoria: Hacia el NORTE (Arriba)
			}
			else if (eventoActual.id_vuelo == -4) {
				nuevoAvion.id = 4;
				nuevoAvion.x = 8000.0f; nuevoAvion.y = 9999.0f; nuevoAvion.z = 18.0f;
				nuevoAvion.angle = (float)-PI / 2.0f; // Trayectoria: Hacia el SUR (Abajo)
			}
			else if (eventoActual.id_vuelo == -5) {
				nuevoAvion.id = 5;
				// CHOQUE FRONTAL: Entra por el Este en el mismo canal horizontal y altitud del Avión 1
				nuevoAvion.x = 9999.0f; nuevoAvion.y = 2000.0f; nuevoAvion.z = 15.0f;
				nuevoAvion.angle = (float)PI; // Trayectoria: Hacia el OESTE (Izquierda)
			}

			nuevoAvion.xIni = nuevoAvion.x;
			nuevoAvion.yIni = nuevoAvion.y;
			nuevoAvion.zIni = nuevoAvion.z;
			nuevoAvion.angleIni = nuevoAvion.angle;

			// [El resto de tu lógica del bucle de simulación continúa igual...]
			
			std::cout << "[T=" << tSim << "] Entrada de vuelo TEST ID: " << nuevoAvion.id << " en posición (" << nuevoAvion.xIni << ", " << nuevoAvion.yIni << ")" << std::endl;

			getEstimatedOutPoint(&nuevoAvion.xFin, &nuevoAvion.yFin, &nuevoAvion.zFin, sectorAereo,
				nuevoAvion.angle, nuevoAvion.x, nuevoAvion.y, nuevoAvion.z);

			//std::cout << "  Estimated out point: " << nuevoAvion.xFin << ", " << nuevoAvion.yFin << ", " << nuevoAvion.zFin << std::endl;

			float dxReal = nuevoAvion.xFin - nuevoAvion.x;
			float dyReal = nuevoAvion.yFin - nuevoAvion.y;
			float dzReal = nuevoAvion.zFin - nuevoAvion.z;
			float distanciaReal3D = sqrt(dxReal * dxReal + dyReal * dyReal + dzReal * dzReal);

			float tiempoVuelo = distanciaReal3D / nuevoAvion.v;
			float instanteSalida = (float)tSim + tiempoVuelo;
			nuevoAvion.tFin = instanteSalida;

			// Evaluar la colisión provocada artificialmente
			if (HayConflictos(nuevoAvion, avionesActivos)) {
				std::cout << "  ! CONFLICTO FRONTAL DETECTADO para avion " << nuevoAvion.id << ". Invocando Metaheuristica..." << std::endl;
				conflicto = true;

				// Llamada al Recocido Simulado 
				bool estadoConflicto = mh.CallMH(nuevoAvion, avionesActivos, sectorAereo);

				if (estadoConflicto) {
					std::cout << "  [OK] Colision frontal resuelta exitosamente para avion " << nuevoAvion.id << std::endl;
					resuelto = true;
				}
			}
			else {
				std::cout << "  + Trayectoria inicial segura para avion " << nuevoAvion.id << std::endl;
			}

			// C. Proyectar salida definitiva post-optimización de la MH
			getEstimatedOutPoint(&nuevoAvion.xFin, &nuevoAvion.yFin, &nuevoAvion.zFin, sectorAereo,
				nuevoAvion.angle, nuevoAvion.x, nuevoAvion.y, nuevoAvion.z);

			std::cout << "  Estimated out point: " << nuevoAvion.xFin << ", " << nuevoAvion.yFin << ", " << nuevoAvion.zFin << std::endl;

			 dxReal = nuevoAvion.xFin - nuevoAvion.x;
			 dyReal = nuevoAvion.yFin - nuevoAvion.y;
			 dzReal = nuevoAvion.zFin - nuevoAvion.z;
			 distanciaReal3D = sqrt(dxReal * dxReal + dyReal * dyReal + dzReal * dzReal);

			 tiempoVuelo = distanciaReal3D / nuevoAvion.v;
			 instanteSalida = (float)tSim + tiempoVuelo;
			std::cout << "  Estimated out dist " << distanciaReal3D << ", tardando " << tiempoVuelo << ", saliendo en instante " << instanteSalida << std::endl;
			// D. Guardar métricas de test en el Histórico
			RegistroHistorico registro = {};
			registro.id = nuevoAvion.id;
			registro.t_in = tSim;
			registro.x_in = nuevoAvion.x; registro.y_in = nuevoAvion.y; registro.z_in = nuevoAvion.z;
			registro.t_sal_p = instanteSalida;
			registro.x_sal_p = nuevoAvion.xFin; registro.y_sal_p = nuevoAvion.yFin; registro.z_sal_p = nuevoAvion.zFin;
			registro.tipo_maniobra = nuevoAvion.idManeuvers;
			registro.hay_conflicto = conflicto ? 1 : 0;
			registro.resuelto = resuelto ? 1 : 0;
			historico.push_back(registro);

			// E. Programar evento de salida real usando el ID positivo real del avión
			avionesActivos.push_back(nuevoAvion);
			listaEventos.push_back({ (double)instanteSalida, nuevoAvion.id });
		}
		else {
			// --- GESTIÓN DE EVENTO: SALIDA ---
			for (auto it = avionesActivos.begin(); it != avionesActivos.end(); ++it) {
				if (it->id == eventoActual.id_vuelo) {
					std::cout << "[T=" << tSim << "] Evento: Salida del Vuelo TEST ID " << eventoActual.id_vuelo << " por (" << it->xFin << ", " << it->yFin << ", " << it->zFin << ")  tiempo de salida: " << eventoActual.tiempo  << std::endl;

					for (auto& reg : historico) {
						if (reg.id == it->id) {
							reg.t_sal_r = tSim;
							reg.x_sal_r = it->xFin;
							reg.y_sal_r = it->yFin;
							reg.z_sal_r = it->zFin;
							break;
						}
					}
					avionesActivos.erase(it);
					break;
				}
			}
		}
	}
	std::cout << "--- Simulación de Test Finalizada ---" << std::endl;
	ExportarACsv(historico, "historicoTestCsv.csv");

	delete sectorAereo;
	return 0;
}

// Escenario 2
int MHDiagTest() {
	// Variables de estado del simulador (camelCase)
	double tSim = 0.0;        // Reloj de la simulación
	int nAvionesInst = 0;    // Contador de aviones activos en el espacio aéreo
	double h = 200;          // Horizonte de simulación lo suficientemente amplio

	// Definición del sector ampliado a 10000x10000
	float puntosX[] = { 0.0f, 10000.0f, 10000.0f, 0.0f };
	float puntosY[] = { 0.0f, 0.0f, 10000.0f, 10000.0f };
	AerialSector* sectorAereo = new AerialSector;
	initAerialSector(sectorAereo, puntosX, puntosY, 4);

	// Estructuras de datos (camelCase)
	vector<Evento> listaEventos;
	vector<RegistroHistorico> historico;
	vector<Aircraft> avionesActivos;

	Metaheuristica mh = initMH();

	// --- INYECCIÓN MANUAL DE EVENTOS DE TEST ---
	listaEventos.push_back({ 0.0, -1 });
	listaEventos.push_back({ 1.0, -2 });
	listaEventos.push_back({ 2.0, -3 });
	listaEventos.push_back({ 3.0, -4 });
	listaEventos.push_back({ 4.0, -5 }); // Este activará la Metaheurística (Conflicto Diagonal)

	while (!listaEventos.empty()) {

		// 1. Encontrar el evento más cercano en el tiempo
		auto itEvento = std::min_element(listaEventos.begin(), listaEventos.end(),
			[](const Evento& a, const Evento& b) { return a.tiempo < b.tiempo; });

		Evento eventoActual = *itEvento;
		listaEventos.erase(itEvento);

		// 2. Saltar el reloj al tiempo del evento y sincronizar posiciones reales
		tSim = eventoActual.tiempo;
		ActualizarPosicionesAviones(avionesActivos, tSim);

		// 3. Procesar el evento
		if (eventoActual.id_vuelo < 0) {
			bool conflicto = false;
			bool resuelto = false;
			Aircraft nuevoAvion;

			// --- PARÁMETROS COMUNES DE RENDIMIENTO (LENTOS) ---
			nuevoAvion.t = tSim;
			nuevoAvion.v = 150.0f;
			nuevoAvion.vIni = nuevoAvion.v;
			nuevoAvion.vMin = 100.0f;
			nuevoAvion.vMax = 200.0f;
			nuevoAvion.zMin = 10.0f;
			nuevoAvion.zMax = 15.0f;
			nuevoAvion.maxAngle = (float)PI / 16.0f;  // Ángulo máximo de maniobra
			nuevoAvion.safetyRad = 4.0234f;
			nuevoAvion.safetyHeight = 0.1524f;
			nuevoAvion.idManeuvers = -1;
			nuevoAvion.xFin = 0.0f;
			nuevoAvion.yFin = 0.0f;

			// --- CONFIGURACIÓN GEOMÉTRICA INDIVIDUAL ---
			if (eventoActual.id_vuelo == -1) {
				nuevoAvion.id = 1;
				nuevoAvion.x = 10.0f;   nuevoAvion.y = 2000.0f; nuevoAvion.z = 15.0f;
				nuevoAvion.angle = 0.0f; // Trayectoria: Hacia el ESTE (Derecha)
			}
			else if (eventoActual.id_vuelo == -2) {
				nuevoAvion.id = 2;
				nuevoAvion.x = 9999.0f; nuevoAvion.y = 8000.0f; nuevoAvion.z = 10.0f;
				nuevoAvion.angle = (float)PI; // Trayectoria: Hacia el OESTE (Izquierda)
			}
			else if (eventoActual.id_vuelo == -3) {
				nuevoAvion.id = 3;
				nuevoAvion.x = 2000.0f; nuevoAvion.y = 10.0f;   nuevoAvion.z = 18.0f;
				nuevoAvion.angle = (float)PI / 2.0f; // Trayectoria: Hacia el NORTE (Arriba)
			}
			else if (eventoActual.id_vuelo == -4) {
				nuevoAvion.id = 4;
				nuevoAvion.x = 8000.0f; nuevoAvion.y = 9999.0f; nuevoAvion.z = 18.0f;
				nuevoAvion.angle = (float)-PI / 2.0f; // Trayectoria: Hacia el SUR (Abajo)
			}
			else if (eventoActual.id_vuelo == -5) {
				nuevoAvion.id = 5;
				// CONFIGURACIÓN DIAGONAL :
				// Entra en T=4.0 en trayectoria SE (Suroeste a Noreste) interceptando al Avión 1 en un punto exacto.
				nuevoAvion.x = 873.6f;
				nuevoAvion.y = 2636.4f;
				nuevoAvion.z = 15.0f; // Misma altitud que Avión 1
				nuevoAvion.angle = (float)-PI / 4.0f; // Ángulo de -45 grados (Diagonal hacia abajo/derecha)
			}

			nuevoAvion.xIni = nuevoAvion.x;
			nuevoAvion.yIni = nuevoAvion.y;
			nuevoAvion.zIni = nuevoAvion.z;
			nuevoAvion.angleIni = nuevoAvion.angle;

			std::cout << "[T=" << tSim << "] Entrada de vuelo TEST ID: " << nuevoAvion.id << " en posición (" << nuevoAvion.xIni << ", " << nuevoAvion.yIni << ")" << std::endl;

			getEstimatedOutPoint(&nuevoAvion.xFin, &nuevoAvion.yFin, &nuevoAvion.zFin, sectorAereo,
				nuevoAvion.angle, nuevoAvion.x, nuevoAvion.y, nuevoAvion.z);

			float dxReal = nuevoAvion.xFin - nuevoAvion.x;
			float dyReal = nuevoAvion.yFin - nuevoAvion.y;
			float dzReal = nuevoAvion.zFin - nuevoAvion.z;
			float distanciaReal3D = sqrt(dxReal * dxReal + dyReal * dyReal + dzReal * dzReal);

			float tiempoVuelo = distanciaReal3D / nuevoAvion.v;
			float instanteSalida = (float)tSim + tiempoVuelo;
			nuevoAvion.tFin = instanteSalida;

			// Evaluar la colisión provocada artificialmente
			if (HayConflictos(nuevoAvion, avionesActivos)) {
				std::cout << "  ! CONFLICTO DIAGONAL DETECTADO para avion " << nuevoAvion.id << ". Invocando Metaheuristica..." << std::endl;
				conflicto = true;

				// Llamada al Recocido Simulado 
				bool estadoConflicto = mh.CallMH(nuevoAvion, avionesActivos, sectorAereo);

				if (estadoConflicto) {
					std::cout << "  [OK] Colision diagonal resuelta exitosamente para avion " << nuevoAvion.id << std::endl;
					resuelto = true;
				}
			}
			else {
				std::cout << "  + Trayectoria inicial segura para avion " << nuevoAvion.id << std::endl;
			}

			// Proyectar salida definitiva post-optimización de la MH
			getEstimatedOutPoint(&nuevoAvion.xFin, &nuevoAvion.yFin, &nuevoAvion.zFin, sectorAereo,
				nuevoAvion.angle, nuevoAvion.x, nuevoAvion.y, nuevoAvion.z);

			std::cout << "  Estimated out point: " << nuevoAvion.xFin << ", " << nuevoAvion.yFin << ", " << nuevoAvion.zFin << std::endl;

			dxReal = nuevoAvion.xFin - nuevoAvion.x;
			dyReal = nuevoAvion.yFin - nuevoAvion.y;
			dzReal = nuevoAvion.zFin - nuevoAvion.z;
			distanciaReal3D = sqrt(dxReal * dxReal + dyReal * dyReal + dzReal * dzReal);

			tiempoVuelo = distanciaReal3D / nuevoAvion.v;
			instanteSalida = (float)tSim + tiempoVuelo;
			std::cout << "  Estimated out dist " << distanciaReal3D << ", tardando " << tiempoVuelo << ", saliendo en instante " << instanteSalida << std::endl;

			// Guardar métricas de test en el Histórico
			RegistroHistorico registro = {};
			registro.id = nuevoAvion.id;
			registro.t_in = tSim;
			registro.x_in = nuevoAvion.x; registro.y_in = nuevoAvion.y; registro.z_in = nuevoAvion.z;
			registro.t_sal_p = instanteSalida;
			registro.x_sal_p = nuevoAvion.xFin; registro.y_sal_p = nuevoAvion.yFin; registro.z_sal_p = nuevoAvion.zFin;
			registro.tipo_maniobra = nuevoAvion.idManeuvers;
			registro.hay_conflicto = conflicto ? 1 : 0;
			registro.resuelto = resuelto ? 1 : 0;
			historico.push_back(registro);

			// Programar evento de salida real usando el ID positivo real del avión
			avionesActivos.push_back(nuevoAvion);
			listaEventos.push_back({ (double)instanteSalida, nuevoAvion.id });
		}
		else {
			// --- GESTIÓN DE EVENTO: SALIDA ---
			for (auto it = avionesActivos.begin(); it != avionesActivos.end(); ++it) {
				if (it->id == eventoActual.id_vuelo) {
					std::cout << "[T=" << tSim << "] Evento: Salida del Vuelo TEST ID " << eventoActual.id_vuelo << " por (" << it->xFin << ", " << it->yFin << ", " << it->zFin << ")  tiempo de salida: " << eventoActual.tiempo << std::endl;

					for (auto& reg : historico) {
						if (reg.id == it->id) {
							reg.t_sal_r = tSim;
							reg.x_sal_r = it->xFin;
							reg.y_sal_r = it->yFin;
							reg.z_sal_r = it->zFin;
							break;
						}
					}
					avionesActivos.erase(it);
					break;
				}
			}
		}
	}
	std::cout << "--- Simulación de Test Finalizada ---" << std::endl;
	ExportarACsv(historico, "historicoTestCsv.csv");

	delete sectorAereo;
	return 0;
}

int main() {
	Simulacion();
	//MHTest();
	//MHDiagTest();

}
