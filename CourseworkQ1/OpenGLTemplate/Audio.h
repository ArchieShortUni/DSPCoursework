#pragma once
#include <windows.h>									// Header File For The Windows Library
#include "./include/fmod_studio/fmod.hpp"
#include "./include/fmod_studio/fmod_errors.h"
#include "ReworkedCircularBuffer.h"
#include <string>


class CAudio
{
public:
	CAudio();
	~CAudio();
	bool Initialise();
	bool LoadEventSound(char *filename);
	bool PlayEventSound();
	bool LoadMusicStream(char *filename);
	bool PlayMusicStream();
	void Update();


	//Delay
	//These two functions are to change the size of the delay in the callback function
	void increase_delay();
	void decrease_delay();
	//This swaps between regular delay and feedback delay
	void swap_delay_mode();
	//returns if the delaymode is regular or feedback
	std::string get_delay_mode(); 
	//These are just to get the values of the delay interval to display on screen
	int get_delay_interval(); 
	float get_delay_interval_seconds(); 

private:

	int interval = 1; 
	FMOD_RESULT result;
	FMOD::System *m_FmodSystem;	// the global variable for talking to FMOD
	FMOD::Sound *m_eventSound;

	FMOD::Sound *m_music;
	FMOD::Channel *m_musicChannel;

	FMOD::DSP *m_dsp;


};
