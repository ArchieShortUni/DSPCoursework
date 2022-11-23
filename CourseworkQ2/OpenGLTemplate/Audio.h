#pragma once
#include <windows.h>									// Header File For The Windows Library
#include "./include/fmod_studio/fmod.hpp"
#include "./include/fmod_studio/fmod_errors.h"
#include "./include/glm/gtc/type_ptr.hpp"
#include "Camera.h"



class CAudio
{
public:
	CAudio();
	~CAudio();
	bool Initialise();
	bool LoadMusicStream(char *filename);
	bool PlayMusicStream();
	void ToggleMusicFilter();
	void IncreaseMusicVolume();
	void DecreaseMusicVolume();
	void Update(CCamera *cam,  glm::vec3 obj_pos,glm::vec3 obj_velocity);
	float get_freqCutoff() { return freqCutoff; }
	float get_distance() { return distance; }

private:
		

	void FmodErrorCheck(FMOD_RESULT result);
	void ToFMODVector(glm::vec3 &glVec3, FMOD_VECTOR *fmodVec);


	FMOD_RESULT result;
	FMOD::System *m_FmodSystem;	// the global variable for talking to FMOD

	//This is to show the distance of the 
	float distance = 0;
	//Lowpass distance filter variables
	//This variable takes the distance between the camera and object and converts it to a frequency to be cut off by the low pass
	float freqCutoff = 2200;
	//This is the range used to calculate if the lowpass filter  is needed and then what frequency it should cut off
	float maxFilterDistance = 220;
	float minFilterDistance = 30;
	//Upper and lower bounds of the filter
	float maxFilterFrequency = 2200;
	float minFilterFrequency = 700;

	//This is the ratio between a single distance unit to how much hz that represents 
	float distanceToFreqRatio = (maxFilterFrequency - minFilterFrequency) / (maxFilterDistance - minFilterDistance); 
	
	FMOD::Sound *m_music;
	FMOD::DSP *m_musicFilter;
	bool m_musicFilterActive;
	FMOD::Channel *m_musicChannel;
	FMOD::DSP *m_musicDSPHead;
	FMOD::DSP *m_musicDSPHeadInput;
	float m_musicVolume = 0.2f;

	FMOD_VECTOR camPos;
	FMOD_VECTOR objPos; 
	FMOD_VECTOR objVel;

	FMOD_VECTOR eventPos;
	FMOD_VECTOR eventVel;

};
