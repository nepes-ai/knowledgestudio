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

#include <NSNeuralNetwork.h>
#include <SPI.h>

extern "C" {
  #include <stdint.h>
}

#define NM500 0x01
#define FPGA 0x02

#define NM_NCR 0x00
#define NM_COMP 0x01
#define NM_LCOMP 0x02
#define NM_DIST 0x03
#define NM_IDX 0x03
#define NM_CAT 0x04
#define NM_AIF 0x05
#define NM_MINIF 0x06
#define NM_MAXIF 0x07
#define NM_TESTCOMP 0x08
#define NM_TESTCAT 0x09
#define NM_NID 0x0A
#define NM_GCR 0x0B
#define NM_RSTCHAIN 0x0C
#define NM_NSR 0x0D
#define NM_NCOUNT 0x0F
#define NM_FORGET 0x0F

#define NEURON_MEMORY 256
#define NEURON_SIZE 576

#define FPGA_RECONF_CS 5
#define DEFAULT_CS 7

NSNeuralNetwork::NSNeuralNetwork() {}

/*
 * Initialize the SPI communication and verify proper interface
 * to the NM500 by reading the default Minimum Influence Field value
 */
uint16_t NSNeuralNetwork::begin()
{
	SPI.begin();
	SPI.setClockDivider(SPI_CLOCK_DIV8);

	// Set default CS & FPGA CS pins
	pinMode (DEFAULT_CS, OUTPUT);
	digitalWrite(DEFAULT_CS, HIGH);

	pinMode (FPGA_RECONF_CS, OUTPUT);
	digitalWrite(FPGA_RECONF_CS, LOW);

	delay(500);

	for (int i = 0; i < 10; i++) {
		reset(FPGA);
		delay(100);
		write(NM_FORGET, 0);
		delay(50);
		// Return 2 if NM500 present and SPI were successful
		if (read(NM_MINIF) == 2) {
			return (0);
		}
		delay(50);
	}

	return (1);
}

/*
 * Get the neural network capacity information
 */
void NSNeuralNetwork::getNetworkInfo(uint16_t* neuronMemorySize, uint32_t* neuronSize, uint16_t* version)
{
	*neuronMemorySize = NEURON_MEMORY;
	*neuronSize = getTotalNeuronCount();
	*version = getVersion();
}

/*
 * Get the size of neurons in network
 */
uint32_t NSNeuralNetwork::getTotalNeuronCount()
{
	uint32_t neurons = 0;

	int tempNSR = read(NM_NSR);

	write(NM_FORGET, 0);
	write(NM_NSR, 0x0010);
	write(NM_TESTCAT, 0x0001);
	write(NM_RSTCHAIN, 0);

	uint16_t tmpCat;

	while (1) {
		tmpCat = read(NM_CAT);
		if (tmpCat == 0xFFFF) break;
		neurons++;
	}

	write(NM_NSR, tempNSR);
	forget();

	return (neurons);
}

/*
 * Read the number of committed neurons
 */
uint32_t NSNeuralNetwork::getNeuronCount()
{
	return (read(NM_NCOUNT));
}

/*
 * Get the version information of FPGA (or Board)
 */
uint16_t NSNeuralNetwork::getVersion()
{
	return (read(FPGA, 0x01));
}

/*
 * Get the current network type (RBF | KNN)
 */
uint8_t NSNeuralNetwork::getNetworkType()
{
	uint16_t nsr = read(NM_NSR);

	return (((uint8_t)nsr) >> 5) & 1;
}

/*
 * Set the network type (RBF:0, KNN:1)
 */
void NSNeuralNetwork::setNetworkType(uint8_t networkType)
{
	if (networkType == 0) {
		int tempNSR = read(NM_NSR);
		write(NM_NSR, tempNSR & 0xDF);
	}
	else {
		int tempNSR=read(NM_NSR);
		write(NM_NSR, tempNSR | 0x20);
	}
}

/*
 * Reset the state and maximum influence field (0x4000)
 */
void NSNeuralNetwork::forget()
{
	write(NM_FORGET, 0);
}

/*
* Get the current context and associated minimum and maximum influence fields
*/
void NSNeuralNetwork::getContext(uint16_t* context, uint8_t* norm, uint16_t* minif, uint16_t* maxif)
{
	*context = read(NM_GCR);
	*norm = (((uint8_t)*context) >> 7) & 1;
	*context = *context & 0xBF;
	*minif = read(NM_MINIF);
	*maxif = read(NM_MAXIF);
}

/*
* Set the current context and associated minimum and maximum influence fields
*/
void NSNeuralNetwork::setContext(uint16_t context, uint8_t norm, uint16_t minif, uint16_t maxif)
{
	if (norm == 1) {
		context = context | 0x80;
	}
	else {
		context = context & 0x7F;
	}
	write(NM_GCR, context);
	write(NM_MINIF, minif);
	write(NM_MAXIF, maxif);
}

/*
 * Get the network status
 */
void NSNeuralNetwork::getNetworkStatus(uint8_t* networkType, uint32_t* networkUsed, uint16_t* context, uint8_t* norm)
{
	*networkType = getNetworkType();
	*networkUsed = read(NM_NCOUNT);

	*context = read(NM_GCR);
	*norm = (((uint8_t)*context) >> 7) & 1;

	*context = *context & 0x7F;
}

/*
 * Learn a vector with current context (GCR)
 */
uint8_t NSNeuralNetwork::learn(uint8_t vector[], uint16_t length, uint16_t category)
{
	uint8_t learned = 0;
	uint16_t prevCount = read(NM_NCOUNT);

	broadcast(vector, length);
	write(NM_CAT, category);

	if (prevCount < read(NM_NCOUNT)) {
		learned++;
	}

	return (learned);
}

/*
 * Recognize a vector and return the best match, or the
 * category, distance and identifier of the top firing neuron
 */
uint8_t NSNeuralNetwork::classify(uint8_t vector[], uint16_t length, uint8_t k, uint16_t distance[], uint16_t category[], uint32_t nid[])
{
	int matchedCount = 0;
	broadcast(vector, length);
	for (int i = 0; i < k; i++) {
		distance[i] = read(NM_DIST);
		if (distance[i] == 0xFFFF) {
			category[i] = 0xFFFF;
			nid[i] = 0xFFFF;
			break;
		}
		else {
			matchedCount++;
			// if degenerated bit(15) is masked
			category[i] = read(NM_CAT) & 0x7FFF;
			nid[i] = read(NM_NID);
		}
	}
	return (matchedCount);
}

/*
 * Recognize a vector and return identifier of the firing neuron
 */
uint8_t NSNeuralNetwork::classify(uint8_t vector[], uint16_t length, uint8_t k, uint32_t nid[])
{
	int matchedCount = 0;
	broadcast(vector, length);
	for (int i = 0; i < k; i++) {
		if (read(NM_DIST) == 0xFFFF) {
			break;
		}
		else {
			matchedCount++;
			read(NM_CAT);
			nid[i] = read(NM_NID);
		}
	}
	return (matchedCount);
}

/**
 * Get the status of recognition (4|8)
 */
uint8_t NSNeuralNetwork::getClassifyStatus()
{
	return read(NM_NSR);
}

/*
 * Read the register value of NM500 through SPI
 */
uint16_t NSNeuralNetwork::read(uint8_t reg)
{
	return (read(NM500, reg));
}

/*
 * Read the register value of specific module through SPI
 */
uint16_t NSNeuralNetwork::read(uint8_t module, uint8_t reg)
{
	digitalWrite(DEFAULT_CS, LOW);

	SPI.transfer(1);
	SPI.transfer(module);
	SPI.transfer(0);
	SPI.transfer(0);
	SPI.transfer(reg);
	SPI.transfer(0);
	SPI.transfer(0);
	SPI.transfer(1);

	uint16_t data = SPI.transfer(0);
	data = (data << 8) + SPI.transfer(0);

	digitalWrite(DEFAULT_CS, HIGH);

	return (data);
}

/*
 * Read the contents of the neuron pointed by index in the chain of neurons
 * starting at index 1
 */
void NSNeuralNetwork::readNeuronsByID(uint32_t nid[], uint8_t length, uint16_t ncr[], uint16_t aif[], uint16_t minif[], uint16_t cat[])
{
	if (length == 0) return;

	int TempNSR = read(NM_NSR);

	write(NM_NSR, 0x0010);

	for (int i = 0; i < length; i++) {
		write(NM_RSTCHAIN, 0);
		for (int j = 0; j < nid[i] - 1; j++) read(NM_CAT);
		ncr[i] = read(NM_NCR);
		aif[i] = read(NM_AIF);
		minif[i] = read(NM_MINIF);
		cat[i] = read(NM_CAT);
	}

	write(NM_NSR, TempNSR);
}

/*
 * Write the register value through SPI
 */
void NSNeuralNetwork::write(uint8_t reg, uint16_t data)
{
	write(NM500, reg, data);
}

/*
 * Write the register value to specific module through SPI
 */
void NSNeuralNetwork::write(uint8_t module, uint8_t reg, uint16_t data)
{
	digitalWrite(DEFAULT_CS, LOW);

	SPI.transfer(1);
	SPI.transfer((uint8_t)(module + 0x80));
	SPI.transfer(0);
	SPI.transfer(0);
	SPI.transfer(reg);
	SPI.transfer(0);
	SPI.transfer(0);
	SPI.transfer(1);
	SPI.transfer((uint8_t)(data >> 8));
	SPI.transfer((uint8_t)(data & 0x00FF));

	digitalWrite(DEFAULT_CS, HIGH);
}

/*
 * Broadcast a vector to the neurons
 */
void NSNeuralNetwork::broadcast(uint8_t vector[], uint16_t length)
{
	// write to the COMP register
	if(length > 1) {
		digitalWrite(DEFAULT_CS, LOW);
		SPI.transfer(1);
		SPI.transfer((uint8_t)(NM500 + 0x80));
		SPI.transfer(0);
		SPI.transfer(0);
		SPI.transfer(NM_COMP);
		SPI.transfer(0);
		SPI.transfer((uint8_t)(length >> 8));
		SPI.transfer((uint8_t)(length & 0x00FF));
		for (int i = 0; i < length - 1; i++) {
			SPI.transfer(0);
			SPI.transfer(vector[i]);
		}
		digitalWrite(DEFAULT_CS, HIGH);
	}
	// write last value to the LCOMP register
	write(NM_LCOMP, vector[length - 1]);
}

/*
 * Initialize all the neurons, set the state with ready to learn.
 * (Maximum Influence Field: 0x4000, RBF)
 */
void NSNeuralNetwork::reset()
{
	int tempNSR = read(NM_NSR);
	write(NM_FORGET, 0);
	write(NM_NSR, 0x10);
	for (int i = 0; i < NEURON_MEMORY; i++) {
		write(NM_TESTCOMP, 0);
	}
	write(NM_NSR, tempNSR & 0xDF);
}

/*
 * Initialize module by force
 */
void NSNeuralNetwork::reset(uint8_t module)
{
	digitalWrite(DEFAULT_CS, LOW);
	SPI.transfer(1);
	SPI.transfer((uint8_t)(module + 0x80));
	SPI.transfer(0);
	SPI.transfer(0);
	SPI.transfer(2);
	SPI.transfer(0);
	SPI.transfer(0);
	SPI.transfer(1);
	SPI.transfer(0);
	SPI.transfer(0);
	digitalWrite(DEFAULT_CS, HIGH);
}
