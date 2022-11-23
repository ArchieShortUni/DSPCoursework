#include "ReworkedCircularBuffer.h"

RWCircularBuffer::RWCircularBuffer(long buffSize) {
	bufferLength = buffSize;
	buffer = new float[buffSize] {};
	tail = 0;
}

RWCircularBuffer::~RWCircularBuffer()
{
	delete buffer;
}

// return the array value given an arbitrary position, using the
	// modulus "%" division command
//Works with negative numbers for at runtime
float RWCircularBuffer::AtPosition(int index) {
	if (index < 0) {
		//If the number is negative it uses this equation to loop round to the relavent position
		//depening on the bufferLength
		return (bufferLength - 1) - ((-index - 1) % bufferLength);
	}
	else {
		return buffer[index % bufferLength];
	}
}

//Returns the position the sample was placed to make sure its fully in sync
int RWCircularBuffer::Place(float value) {
	buffer[tail % bufferLength] = value;
	tail++;
	return tail - 1;
}
