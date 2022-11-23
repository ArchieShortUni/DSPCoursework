#pragma once
class RWCircularBuffer
{
public:
	RWCircularBuffer(long buffSize);
	~RWCircularBuffer();
	float AtPosition(int index);
	int Place(float value);


private:

	int bufferLength;
	float* buffer;
	int tail;

};
#pragma once
