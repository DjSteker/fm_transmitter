/*
    FM Transmitter - use Raspberry Pi as FM transmitter

    Copyright (c) 2022, Marcin Kondej
    All rights reserved.

    See https://github.com/markondej/fm_transmitter

    Redistribution and use in source and binary forms, with or without modification, are
    permitted provided that the following conditions are met:

    1. Redistributions of source code must retain the above copyright notice, this list
    of conditions and the following disclaimer.

    2. Redistributions in binary form must reproduce the above copyright notice, this
    list of conditions and the following disclaimer in the documentation and/or other
    materials provided with the distribution.

    3. Neither the name of the copyright holder nor the names of its contributors may be
    used to endorse or promote products derived from this software without specific
    prior written permission.

    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY
    EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
    OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT
    SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
    INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED
    TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR
    BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
    CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY
    WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

/*
 * Transmitter.hpp
 *
 *  Created on: 1 oct 2026
 *      Author: DjSteker
 */

#ifndef TRANSMITTER_HPP_
#define TRANSMITTER_HPP_

#include "WaveReader.hpp"
#include <condition_variable>
#include <mutex>
#include <vector>

class ClockOutput;

/**
 * @file Transmitter.hpp
 * @brief Declaración de la clase Transmitter, transmisor de FM para Raspberry Pi.
 */

/**
 * @class Transmitter
 * @brief Transmite audio WAVE en FM modulando el divisor de reloj de un pin GPIO.
 *
 * La portadora se genera con el reloj general purpose de la Raspberry Pi
 * (GPCLK0 en GPIO4, o GPCLK1 en GPIO21 si se define GPIO21). La modulación se
 * consigue variando el divisor fraccional del reloj al ritmo de las muestras
 * de audio. Existen dos modos de funcionamiento:
 *  - **DMA**: el controlador DMA actualiza el divisor y sincroniza con el PWM.
 *  - **CPU**: un hilo dedicado actualiza el divisor temporizando por software.
 *
 * Requiere permisos de root (acceso a /dev/mem y /dev/vcio).
 *
 * @note La clase no es copiable ni movible.
 */
class Transmitter {
public:
	/**
	 * @brief Constructor. Deja el transmisor inactivo y sin portadora.
	 */
	Transmitter();

	/**
	 * @brief Destructor.
	 *
	 * Espera a que finalice cualquier transmisión en curso y libera la salida
	 * de reloj (apaga la portadora).
	 */
	virtual ~Transmitter();

	/** @brief Copia deshabilitada. */
	Transmitter(const Transmitter&) = delete;

	/** @brief Movimiento deshabilitado. */
	Transmitter(Transmitter&&) = delete;

	/** @brief Asignación por copia deshabilitada. */
	Transmitter& operator=(const Transmitter&) = delete;

	/**
	 * @brief Transmite el contenido de un lector WAVE.
	 *
	 * Bloquea hasta que termina el archivo, se invoca Stop() o se produce un error.
	 *
	 * @param reader          Lector WAVE del que se obtienen las muestras.
	 * @param frequency       Frecuencia de la portadora en MHz.
	 * @param bandwidth       Ancho de banda de la modulación en kHz.
	 * @param dmaChannel      Canal DMA a utilizar (0 - 15). El valor 0xff selecciona
	 *                        el modo CPU en lugar de DMA.
	 * @param preserveCarrier Si es true, la portadora se mantiene encendida al
	 *                        terminar (útil al encadenar varios archivos).
	 *
	 * @throw std::runtime_error Si el canal DMA está fuera de rango, falla el
	 *                           acceso a los periféricos o la reserva de memoria.
	 */
	void Transmit(WaveReader &reader, float frequency, float bandwidth, unsigned dmaChannel, bool preserveCarrier);

	/**
	 * @brief Solicita detener la transmisión en curso.
	 *
	 * Es seguro invocarla desde otro hilo.
	 */
	void Stop();

private:
	/**
	 * @brief Transmisión mediante CPU.
	 *
	 * Lanza un hilo (CpuTxThread()) que modula el divisor mientras este hilo
	 * lee las muestras del archivo.
	 *
	 * @param reader       Lector WAVE de origen.
	 * @param sampleRate   Frecuencia de muestreo del audio en Hz.
	 * @param bufferSize   Número de muestras por bloque.
	 * @param clockDivisor Divisor de reloj correspondiente a la portadora.
	 * @param divisorRange Desviación máxima del divisor para la modulación.
	 */
	void TxViaCpu(WaveReader &reader, unsigned sampleRate, unsigned bufferSize, unsigned clockDivisor, unsigned divisorRange);

	/**
	 * @brief Transmisión mediante DMA.
	 *
	 * Construye una cadena de bloques de control DMA que alterna la escritura
	 * del divisor de reloj y el volcado al FIFO del PWM (que marca el ritmo).
	 *
	 * @param reader       Lector WAVE de origen.
	 * @param sampleRate   Frecuencia de muestreo del audio en Hz.
	 * @param bufferSize   Número de muestras por bloque.
	 * @param clockDivisor Divisor de reloj correspondiente a la portadora.
	 * @param divisorRange Desviación máxima del divisor para la modulación.
	 * @param dmaChannel   Canal DMA (0 - 15).
	 *
	 * @throw std::runtime_error Si el canal DMA está fuera de rango.
	 */
	void TxViaDma(WaveReader &reader, unsigned sampleRate, unsigned bufferSize, unsigned clockDivisor, unsigned divisorRange, unsigned dmaChannel);

	/**
	 * @brief Cuerpo del hilo de transmisión en modo CPU.
	 *
	 * @param sampleRate   Frecuencia de muestreo del audio en Hz.
	 * @param clockDivisor Divisor de reloj correspondiente a la portadora.
	 * @param divisorRange Desviación máxima del divisor para la modulación.
	 * @param sampleOffset [out] Posición de muestra alcanzada, usada para
	 *                     reposicionar la lectura y no perder la sincronía.
	 * @param samples      Buffer compartido de muestras pendientes de emitir.
	 * @param stop         Indicador de parada compartido con TxViaCpu().
	 */
	void CpuTxThread(unsigned sampleRate, unsigned clockDivisor, unsigned divisorRange, unsigned *sampleOffset, std::vector<Sample> *samples, bool *stop);

	std::condition_variable cv; ///< Variable de condición para sincronizar hilos.
	ClockOutput *output;        ///< Salida de reloj (portadora); nullptr si está apagada.
	std::mutex mtx;             ///< Mutex que protege @ref enable y los buffers compartidos.
	bool enable;                ///< true mientras hay una transmisión activa.
};

#endif /* TRANSMITTER_HPP_ */
