/*
 * KnowledgeStudio.cpp - Protocol implementation for communicating with Knowledge Studio
 * Copyright (c) 2017, nepes corp, All rights reserved.
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
#include <Arduino.h>
#include <NSNeuralNetwork.h>
#include <string.h>

#define SERIAL_BUFFER_SIZE 512

#define MSG_RES_IDENTIFER 0x90000000
#define MSG_GET_NETWORK_INFO 0x00000001
#define MSG_GET_NETWORK_STATUS 0x00000002
#define MSG_RESET 0x00000003
#define MSG_SET_NETWORK_TYPE 0x00000004
#define MSG_SET_CONTEXT 0x00000005
#define MSG_GET_CONTEXT 0x00000006
#define MSG_CLASSIFY 0x00000101
#define MSG_LEARN 0x00000102
#define MSG_BROADCAST_STOP_SERVICE 0x80000000
#define MSG_BROADCAST 0x80000001

void getNetworkInfo();
void getNetworkStatus();
void doReset();
void setNetworkType();
void setContext();
void getContext();
void doClassify();
void doLearn();
void broadcast(const char str[]);
void broadcastStopService(const char str[]);

void iton(uint8_t buf[], uint32_t value);
void ston(uint8_t buf[], uint16_t value);
void siton(uint8_t buf[], uint16_t value);

void append(char log[], const char str[]);
size_t digitLen(uint32_t value);
void printHex(uint8_t b[], size_t len);

typedef union _BYTE4 {
	uint32_t value;
	uint8_t buf[4];
} BYTE4;

typedef union _BYTE2 {
	uint16_t value;
	uint8_t buf[2];
} BYTE2;

NSNeuralNetwork nsNN;
BYTE4 msgType, msgLen;

void setup() {
	Serial.begin(115200);
	while (!Serial);

	if (nsNN.begin() == 0) {
		broadcast("Welcome!! Connected to NeuroShield");
	}
	else {
		broadcastStopService("NeuroShield and NM500 were not initialized properly. Please reconnect.");
		while(1);
	}
}

void loop()
{
	Serial.flush();

	// read message type
	while (Serial.available() < 4) {}
	for (int i = sizeof(msgType) - 1; i >= 0; i--) {
		msgType.buf[i] = Serial.read();
	}

	// read body length
	while (Serial.available() < 4) {}
	for (int i = sizeof(msgLen) - 1; i >= 0; i--) {
		msgLen.buf[i] = Serial.read();
	}

	switch (msgType.value) {
	case MSG_GET_NETWORK_INFO:
		getNetworkInfo();
		break;
	case MSG_GET_NETWORK_STATUS:
		getNetworkStatus();
		break;
	case MSG_RESET:
		doReset();
		break;
	case MSG_SET_NETWORK_TYPE:
		setNetworkType();
		break;
	case MSG_SET_CONTEXT:
		setContext();
		break;
	case MSG_GET_CONTEXT:
		getContext();
		break;
	case MSG_CLASSIFY:
		doClassify();
		break;
	case MSG_LEARN:
		doLearn();
		break;
	default:
		broadcast("received UNKNOWN ");
		break;
	}
}

void getNetworkInfo()
{
	while (Serial.available() < msgLen.value) {}

	uint16_t neuronMemory;
	uint32_t neuronCount;
	uint16_t version;

	nsNN.getNetworkInfo(&neuronMemory, &neuronCount, &version);

	// Send response message
	// message header
	// message type
	uint8_t resMsgType[4];
	iton(resMsgType, msgType.value + MSG_RES_IDENTIFER);
	Serial.write(resMsgType, sizeof(resMsgType));

	// message length
	uint8_t resMsgLen[4];
	iton(resMsgLen, 8);
	Serial.write(resMsgLen, sizeof(resMsgLen));

	// message body
	// Memory Size:2, Neuron Size:4, Version:2
	// neuron memory size
	uint8_t resNeuronMemory[2];
	ston(resNeuronMemory, neuronMemory);
	Serial.write(resNeuronMemory, sizeof(resNeuronMemory));

	// network neuron size
	uint8_t resNeuronCount[4];
	iton(resNeuronCount, neuronCount);
	Serial.write(resNeuronCount, sizeof(resNeuronCount));

	// version
	uint8_t resVersion[2];
	ston(resVersion, version);
	Serial.write(resVersion, sizeof(resVersion));

	Serial.flush();
}

void getNetworkStatus()
{
	while (Serial.available() < msgLen.value) {}

	uint8_t networkType;
	uint32_t networkUsed;
	uint16_t context;
	uint8_t norm;

	nsNN.getNetworkStatus(&networkType, &networkUsed, &context, &norm);

	// send response message
	// message header
	// message type
	uint8_t resMsgType[4];
	iton(resMsgType, msgType.value + MSG_RES_IDENTIFER);
	Serial.write(resMsgType, sizeof(resMsgType));

	// message length
	uint8_t resMsgLen[4];
	iton(resMsgLen, 8);
	Serial.write(resMsgLen, sizeof(resMsgLen));

	// message body
	// type:1, used:4, context:2, norm:1
	// network type
	Serial.write(networkType);

	// network used
	uint8_t resUsed[4];
	iton(resUsed, networkUsed);
	Serial.write(resUsed, sizeof(resUsed));

	// context
	uint8_t resContext[2];
	ston(resContext, context);
	Serial.write(resContext, sizeof(resContext));

	// norm
	Serial.write(norm);

	Serial.flush();
}

void doReset()
{
	while (Serial.available() < msgLen.value) {}

	uint8_t networkType;
	uint32_t networkUsed;
	uint16_t context;
	uint8_t norm;

	nsNN.reset();

	nsNN.getNetworkStatus(&networkType, &networkUsed, &context, &norm);

	// send response message
	// message header
	// message type
	uint8_t resMsgType[4];
	iton(resMsgType, msgType.value + MSG_RES_IDENTIFER);
	Serial.write(resMsgType, sizeof(resMsgType));

	// message length
	uint8_t resMsgLen[4];
	iton(resMsgLen, 8);
	Serial.write(resMsgLen, sizeof(resMsgLen));

	// message body
	// type:1, used:4, context:2, norm:1
	// network type
	Serial.write(networkType);

	// network used
	uint8_t resUsed[4];
	iton(resUsed, networkUsed);
	Serial.write(resUsed, sizeof(resUsed));

	// context
	uint8_t resContext[2];
	ston(resContext, context);
	Serial.write(resContext, sizeof(resContext));

	// norm
	Serial.write(norm);

	Serial.flush();
}

void setNetworkType()
{
	while (Serial.available() < msgLen.value) {}

	uint8_t networkType = Serial.read();

	nsNN.setNetworkType(networkType);

	// Send response message
	// message header
	// message type
	uint8_t resMsgType[4];
	iton(resMsgType, msgType.value + MSG_RES_IDENTIFER);
	Serial.write(resMsgType, sizeof(resMsgType));

	// message length
	uint8_t resMsgLen[4];
	iton(resMsgLen, 0);
	Serial.write(resMsgLen, sizeof(resMsgLen));

	Serial.flush();
}

void setContext()
{
	while (Serial.available() < msgLen.value) {}

	BYTE2 context;
	for (int i = sizeof(context) - 1; i >= 0; i--) {
		context.buf[i] = Serial.read();
	}

	uint8_t norm = Serial.read();

	BYTE2 minif;
	for (int i = sizeof(minif) - 1; i >= 0; i--) {
		minif.buf[i] = Serial.read();
	}

	BYTE2 maxif;
	for (int i = sizeof(maxif) - 1; i >= 0; i--) {
		maxif.buf[i] = Serial.read();
	}

	// set the context
	nsNN.setContext(context.value, norm, minif.value, maxif.value);
	// get the context
	nsNN.getContext(&context.value, &norm, &minif.value, &maxif.value);

	// send response message
	// message header
	// message type
	uint8_t resMsgType[4];
	iton(resMsgType, msgType.value + MSG_RES_IDENTIFER);
	Serial.write(resMsgType, sizeof(resMsgType));

	// message length
	uint8_t resMsgLen[4];
	iton(resMsgLen, 7);
	Serial.write(resMsgLen, sizeof(resMsgLen));

	// message body
	// context
	uint8_t resContext[2];
	ston(resContext, context.value);
	Serial.write(resContext, sizeof(resContext));

	// norm
	Serial.write(norm);

	// minimum influence field
	uint8_t resMinIF[2];
	ston(resMinIF, minif.value);
	Serial.write(resMinIF, sizeof(resMinIF));

	// maximum influence field
	uint8_t resMaxIF[2];
	ston(resMaxIF, maxif.value);
	Serial.write(resMaxIF, sizeof(resMaxIF));

	Serial.flush();
}

void getContext()
{
	while (Serial.available() < msgLen.value) {}

	uint16_t context;
	uint8_t norm;
	uint16_t minif;
	uint16_t maxif;

	// get the context
	nsNN.getContext(&context, &norm, &minif, &maxif);

	// send response message
	// message header
	// message type
	uint8_t resMsgType[4];
	iton(resMsgType, msgType.value + MSG_RES_IDENTIFER);
	Serial.write(resMsgType, sizeof(resMsgType));

	// message length
	uint8_t resMsgLen[4];
	iton(resMsgLen, 7);
	Serial.write(resMsgLen, sizeof(resMsgLen));

	// message body
	// context
	uint8_t resContext[2];
	ston(resContext, context);
	Serial.write(resContext, sizeof(resContext));

	// norm
	Serial.write(norm);

	// minimum influence field
	uint8_t resMinif[2];
	ston(resMinif, minif);
	Serial.write(resMinif, sizeof(resMinif));

	// maximum influence field
	uint8_t resMaxif[2];
	ston(resMaxif, maxif);
	Serial.write(resMaxif, sizeof(resMaxif));

	Serial.flush();
}

void doClassify()
{
	while (Serial.available() < 1) {}

	uint8_t queryCount = Serial.read();
	uint16_t vectorLen = 256;

	uint8_t vector[vectorLen];
	for (int i = 0 ; i < vectorLen ; i++) {
		while (Serial.available() < 1) {}
		vector[i] = Serial.read();
	}

	uint16_t distance[queryCount];
	uint16_t category[queryCount];
	uint32_t nid[queryCount];

	uint8_t matchedCount = nsNN.classify(vector, vectorLen, queryCount, distance, category, nid);
	uint8_t matchedStatus = nsNN.getClassifyStatus();

	matchedCount = min(queryCount, matchedCount);

	// send response message
	// message header
	// message type
	uint8_t resMsgType[4];
	iton(resMsgType, msgType.value + MSG_RES_IDENTIFER);
	Serial.write(resMsgType, sizeof(resMsgType));

	// message length
	uint8_t resMsgLen[4];
	iton(resMsgLen, 2 + (matchedCount * 8));
	Serial.write(resMsgLen, sizeof(resMsgLen));

	// message body
	// matched status [4|8]
	Serial.write(matchedStatus);

	// number of matched K
	Serial.write(matchedCount);

	// matched list
	for (int i = 0; i < matchedCount; i++) {
		// neuron id
		uint8_t resNID[4];
		iton(resNID, nid[i]);
		Serial.write(resNID, sizeof(resNID));
		// category
		uint8_t resCategory[2];
		ston(resCategory, category[i]);
		Serial.write(resCategory, sizeof(resCategory));
		// distance
		uint8_t resDistance[2];
		ston(resDistance, distance[i]);
		Serial.write(resDistance, sizeof(resDistance));
	}

	Serial.flush();
}

void doLearn()
{
	while (Serial.available() < 1) {}

	uint8_t isQueryAffected = Serial.read();
	uint16_t vectorLen = 256;

	uint8_t vector[vectorLen];
	for (int i = 0 ; i < vectorLen ; i++) {
		while (Serial.available() < 1) {}
		vector[i] = Serial.read();
	}

	BYTE2 category;
	for (int i = sizeof(category) - 1; i >= 0; i--) {
		while (Serial.available() < 1) {}
		category.buf[i] = Serial.read();
	}

	if (isQueryAffected == 1) {

		uint8_t queryCount = 20;
		uint32_t nid[queryCount];

		// to retrieve affected neurons, do classify first
		uint8_t matchedCount = nsNN.classify(vector, vectorLen, queryCount, nid);
		matchedCount++;

		// learn a vector
		uint8_t learnedStatus = nsNN.learn(vector, vectorLen, category.value);

		uint16_t ncr[matchedCount];
		uint16_t aif[matchedCount];
		uint16_t minif[matchedCount];
		uint16_t cat[matchedCount];

		// add learned neuron to affected list
		if (learnedStatus > 0) {
			nid[matchedCount - 1] = nsNN.getNeuronCount();
		}
		else {
			matchedCount--;
		}

		// retrieve all neurons affected by learn process
		nsNN.readNeuronsByID(nid, matchedCount, ncr, aif, minif, cat);

		// send response message
		// message header
		// message type
		uint8_t resMsgType[4];
		iton(resMsgType, msgType.value + MSG_RES_IDENTIFER);
		Serial.write(resMsgType, sizeof(resMsgType));

		// message length
		uint8_t resMsgLen[4];
		iton(resMsgLen, 3 + (matchedCount * 12));
		Serial.write(resMsgLen, sizeof(resMsgLen));

		// message body
		// learn status [0|1]
		Serial.write(learnedStatus);

		// number of affected neurons
		uint8_t resAffectedCount[2];
		ston(resAffectedCount, matchedCount);
		Serial.write(resAffectedCount, sizeof(resAffectedCount));

		// affected neurons list
		for (int i = 0; i < matchedCount; i++) {
			// neuron id
			uint8_t resNID[4];
			iton(resNID, nid[i]);
			Serial.write(resNID, sizeof(resNID));

			// neuron context register
			uint8_t resNCR[2];
			ston(resNCR, ncr[i]);
			Serial.write(resNCR, sizeof(resNCR));

			// active influence field
			uint8_t resActiveIF[2];
			ston(resActiveIF, aif[i]);
			Serial.write(resActiveIF, sizeof(resActiveIF));

			// minimum influence field
			uint8_t resMinIF[2];
			ston(resMinIF, minif[i]);
			Serial.write(resMinIF, sizeof(resMinIF));

			// category
			uint8_t resCategory[2];
			ston(resCategory, cat[i]);
			Serial.write(resCategory, sizeof(resCategory));
		}
	}
	else {
		uint8_t learnedStatus = nsNN.learn(vector, vectorLen, category.value);

		// send response message
		// message header
		// message type
		uint8_t resMsgType[4];
		iton(resMsgType, msgType.value + MSG_RES_IDENTIFER);
		Serial.write(resMsgType, sizeof(resMsgType));

		// message length
		uint8_t resMsgLen[4];
		iton(resMsgLen, 3);
		Serial.write(resMsgLen, sizeof(resMsgLen));

		// message body
		// learn status [0|1]
		Serial.write(learnedStatus);

		// number of affected neurons
		uint8_t resAffectedCount[2];
		ston(resAffectedCount, 0);
		Serial.write(resAffectedCount, sizeof(resAffectedCount));
	}

	Serial.flush();
}

void append(char log[], const char str[]) {
	const char sp[] = " ";
	strncat(log, sp, strlen(sp));
	strncat(log, str, strlen(str));
}

void iton(uint8_t buf[], uint32_t value)
{
	buf[0] = value >> 24;
	buf[1] = value >> 16;
	buf[2] = value >> 8;
	buf[3] = value;
}

void ston(uint8_t buf[], uint16_t value)
{
	buf[0] = value >> 8;
	buf[1] = value;
}

void siton(uint8_t buf[], uint16_t value)
{
	buf[0] = 0x00;
	buf[1] = 0x00;
	buf[2] = value >> 8;
	buf[3] = value;
}

size_t digitLen(uint32_t value)
{
	size_t len = 1;
	while (value > 9) {
		len++;
		value /=10;
	}
	return len;
}

void broadcast(const char str[])
{
	// Send response message
	// message type
	uint8_t broMsgType[4];
	iton(broMsgType, MSG_BROADCAST);
	Serial.write(broMsgType, sizeof(broMsgType));

	// message length
	uint8_t broMsgLen[4];
	siton(broMsgLen, strlen(str));
	Serial.write(broMsgLen, 4);

	Serial.write(str, strlen(str));

	Serial.flush();
}

void broadcastStopService(const char str[])
{
	// Send response message
	// message type
	uint8_t broMsgType[4];
	iton(broMsgType, MSG_BROADCAST_STOP_SERVICE);
	Serial.write(broMsgType, sizeof(broMsgType));

	// message length
	uint8_t broMsgLen[4];
	siton(broMsgLen, strlen(str));
	Serial.write(broMsgLen, 4);

	Serial.write(str, strlen(str));

	Serial.flush();
}

void printHex(uint8_t b[], size_t len)
{
	for (size_t i = 0; i < len; i++) {
		if (b[i] < 9) Serial.print("0");
		Serial.print(b[i], HEX);
		Serial.print(" ");
	}
}
