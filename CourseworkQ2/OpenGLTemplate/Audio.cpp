#include "Audio.h"

#pragma comment(lib, "lib/fmod_vc.lib")

CAudio::CAudio()
{}

CAudio::~CAudio()
{}



bool CAudio::Initialise()
{
	// Create an FMOD system
	result = FMOD::System_Create(&m_FmodSystem);
	FmodErrorCheck(result);
	if (result != FMOD_OK) 
		return false;

	// Initialise the system
	result = m_FmodSystem->init(32, FMOD_INIT_NORMAL, 0);
	FmodErrorCheck(result);
	if (result != FMOD_OK) 
		return false;

	// 3) Set 3D settings
	result = m_FmodSystem->set3DSettings(1.0f, 1.0f, 1.0f);
	FmodErrorCheck(result);
	if (result != FMOD_OK)
		return false;

	return true;
	
}




// Load a music stream
bool CAudio::LoadMusicStream(char *filename)
{
	result = m_FmodSystem->createStream(filename, NULL | FMOD_LOOP_NORMAL, 0, &m_music);
	FmodErrorCheck(result);

	if (result != FMOD_OK)
		return false;

	// create a low-pass filter DSP object
	result = m_FmodSystem->createDSPByType(FMOD_DSP_TYPE_LOWPASS, &m_musicFilter);

	if (result != FMOD_OK)
		return false;

	// you can start the DSP in an inactive state
	m_musicFilter->setActive(false);

	return true;

	result = m_musicChannel->setVolume(m_musicVolume * 40);
	FmodErrorCheck(result);
	

}

// Play a music stream
bool CAudio::PlayMusicStream()
{
	result = m_FmodSystem->playSound(m_music, NULL, false, &m_musicChannel);
	FmodErrorCheck(result);

	if (result != FMOD_OK)
		return false;

	//Initialise 3d Sound
	m_musicChannel->setMode(FMOD_3D);
	//setting the rolloff and doppler settings, I found these worked best so you can actually hear the effect 
	m_FmodSystem->set3DSettings(5, 2, 1);
	FmodErrorCheck(result);

	//on my computer this volume worked best so i stuck with it
	result = m_musicChannel->setVolume(m_musicVolume * 40);
	FmodErrorCheck(result);

	// connecting the music filter to the music stream
	// Get the DSP head and it's input
	m_musicChannel->getDSP(FMOD_CHANNELCONTROL_DSP_HEAD, &m_musicDSPHead);
	m_musicDSPHead->getInput(0, &m_musicDSPHeadInput, NULL);
	// Disconnect them
	m_musicDSPHead->disconnectFrom(m_musicDSPHeadInput);
	// Add input to the music head from the filter
	result = m_musicDSPHead->addInput(m_musicFilter);
	FmodErrorCheck(result);

	if (result != FMOD_OK)
		return false;

	// Add input to the filter head music DSP head input
	result = m_musicFilter->addInput(m_musicDSPHeadInput);
	FmodErrorCheck(result);

	if (result != FMOD_OK)
		return false;

	// set the DSP object to be active
	m_musicFilter->setActive(true);
	// initially set the cutoff to a high value
	m_musicFilter->setParameterFloat(FMOD_DSP_LOWPASS_CUTOFF, 22000);
	// this state is used for toggling
	m_musicFilterActive = false;

	return true;
}

// Check for error
void CAudio::FmodErrorCheck(FMOD_RESULT result)
{						
	if (result != FMOD_OK) {
		const char *errorString = FMOD_ErrorString(result);
		// MessageBox(NULL, errorString, "FMOD Error", MB_OK);
		// Warning: error message commented out -- if headphones not plugged into computer in lab, error occurs
	}
}

void CAudio::Update(CCamera *camera, glm::vec3 obj_pos, glm::vec3 obj_velocity)
{
	//converting all of the vectors to fmod vectors
	ToFMODVector(camera->GetPosition(), &camPos);
	ToFMODVector(obj_pos, &objPos);
	ToFMODVector(obj_velocity, &objVel);

	//setting the position of the car in fmod space
	result = m_musicChannel->set3DAttributes(&objPos, &objVel);
	FmodErrorCheck(result);

	//setting the position of the camera
	result = m_FmodSystem->set3DListenerAttributes(0, &camPos, NULL, NULL, NULL);
	FmodErrorCheck(result);

	//calculating the lowpass cutoff for distance
	//get distance first between the car and camera
	 distance = glm::distance(camera->GetPosition(), obj_pos); 
	 //it only changes the cutoff if within a certain distance range
	if ((distance >= minFilterDistance )&& (distance <= maxFilterDistance)) {
		//this uses the parameters set in the .h to calculate what the frequency cutoff should be linearly 
		//decreasing as the car gets further away
	    freqCutoff = maxFilterFrequency - (distanceToFreqRatio * (distance - minFilterDistance)); 
		//sets the cutoff
		m_musicFilter->setParameterFloat(FMOD_DSP_LOWPASS_CUTOFF, freqCutoff);
	}


	m_FmodSystem->update();


}

void CAudio::ToggleMusicFilter()
{
	// called externally from Game::ProcessEvents
	// toggle the effect on/off
	m_musicFilterActive = !m_musicFilterActive;
	if (m_musicFilterActive) {
		// set the parameter to a low value
		m_musicFilter->setParameterFloat(FMOD_DSP_LOWPASS_CUTOFF, 700);
	}
	else {
		// set the parameter to a high value
		// you could also use m_musicFilter->setBypass(true) instead...
		m_musicFilter->setParameterFloat(FMOD_DSP_LOWPASS_CUTOFF, 22000);
	}
}

void CAudio::IncreaseMusicVolume()
{
	// called externally from Game::ProcessEvents
	// increment the volume
	m_musicVolume += 0.05f;
	if (m_musicVolume > 1)
		m_musicVolume = 1.0f;
	m_musicChannel->setVolume(m_musicVolume);
}

void CAudio::DecreaseMusicVolume()
{
	// called externally from Game::ProcessEvents
	// deccrement the volume
	m_musicVolume -= 0.05f;
	if (m_musicVolume < 0)
		m_musicVolume = 0.0f;
	m_musicChannel->setVolume(m_musicVolume);
}

void CAudio::ToFMODVector(glm::vec3 &glVec3, FMOD_VECTOR *fmodVec)
{
	fmodVec->x = glVec3.x;
	fmodVec->y = glVec3.y;
	fmodVec->z = glVec3.z;
}