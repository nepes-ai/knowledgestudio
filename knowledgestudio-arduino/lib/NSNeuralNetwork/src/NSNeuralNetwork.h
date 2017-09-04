/*
 * NSNeuralNetwork.cpp - NeuroShield driver
 * Copyright (c) 2017, nepes corp, All rights reserved.
 * Original Source: Copyright (c) 2016, General Vision Inc, All rights reserved
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its contributors
 * may be used to endorse or promote products derived from this software without
 * specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 */

#ifndef _NSNEURONS_H
#define _NSNEURONS_H

#include "SPI.h"

extern "C" {
  #include <stdint.h>
}

class NSNeuralNetwork
{
	public:

	NSNeuralNetwork();
	uint16_t begin();

	void getNetworkInfo(uint16_t* neuronMemorySize, uint32_t* neuronSize, uint16_t* version);

	uint16_t getVersion();
	uint32_t getTotalNeuronCount();
	uint32_t getNeuronCount();

	void setNetworkType(uint8_t networkType) ;
	uint8_t getNetworkType();
	void getNetworkStatus(uint8_t* networkType, uint32_t* networkUsed, uint16_t* context, uint8_t* norm);

	void setContext(uint16_t context, uint8_t norm, uint16_t minif, uint16_t maxif);
	void getContext(uint16_t* context, uint8_t* norm, uint16_t* minif, uint16_t* maxif);

	uint8_t classify(uint8_t vector[], uint16_t length, uint8_t k, uint16_t distance[], uint16_t category[], uint32_t nid[]);
	uint8_t classify(uint8_t vector[], uint16_t length, uint8_t k, uint32_t nid[]);
	uint8_t getClassifyStatus();

	uint8_t learn(uint8_t vector[], uint16_t length, uint16_t category);

	void forget();
	void reset();
	void reset(uint8_t module);

	uint16_t read(uint8_t reg);
	uint16_t read(uint8_t module, uint8_t reg);
	void readNeuronsByID(uint32_t nid[], uint8_t length, uint16_t ncr[], uint16_t aif[], uint16_t minif[], uint16_t category[]);
	void write(uint8_t reg, uint16_t data);
	void write(uint8_t module, uint8_t reg, uint16_t data);
	void broadcast(uint8_t vector[], uint16_t length);
};
#endif
