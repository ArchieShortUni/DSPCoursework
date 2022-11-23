#include "Audio.h"

#pragma comment(lib, "lib/fmod_vc.lib")

/*
I've made these two functions non-member functions
*/

typedef struct
{
	//The value of the delay
	int delay_interval;
	//This is what type of delay is being used 0 is regular 1 is feedback
	int feedback_delay;
} mydsp_data_t;

// Check for error
void FmodErrorCheck(FMOD_RESULT result)
{
	if (result != FMOD_OK) {
		const char *errorString = FMOD_ErrorString(result);
		// MessageBox(NULL, errorString, "FMOD Error", MB_OK);
		// Warning: error message commented out -- if headphones not plugged into computer in lab, error occurs
	}
}

int numChannels = 2;

//This size allows just over 2 seconds of delay
RWCircularBuffer buffer(120*1024*numChannels);
//initialises the circular buffer position to 0 for keeping track of where the tail is within the callback
int position = 0;

FMOD_RESULT F_CALLBACK DSPCallbackDelay(FMOD_DSP_STATE* dsp_state, float* inbuffer, float* outbuffer,
	unsigned int length, int inchannels, int* outchannels)
{
	//retrive the delay interval data and feedback_delay type to use below
	mydsp_data_t* data = (mydsp_data_t*)dsp_state->plugindata; 


	for (unsigned int samp = 0; samp < length; samp++)
	{
		for (int chan = 0; chan < *outchannels; chan++)
		{
			//This if statement chooses which delay type is being used
			if (data->feedback_delay == 0) {
				//Regular delay 

				//Place into the buffer first using only the input channels
				position = buffer.Place(inbuffer[samp * inchannels + chan]);
				//Take the current position in the circular buffer and a position behind relative to the delay interval and add them together
				//This gives an effect of the same sound being played back twice at a delay rather than the echo of feedback
				outbuffer[(samp * *outchannels) + chan] = (0.5 * buffer.AtPosition(position)) + (0.5 * buffer.AtPosition(position - (data->delay_interval * length*numChannels)));
			}
			else {
				//Feedback delay
			//set the outbuffer first using the inbuffer + the current position in the circular buffer behind by the delay interval
			//the circular buffer is initialised to 0 so when it begins you can only hear the inbuffer until it catches up
			outbuffer[(samp * *outchannels) + chan] = (0.5 * inbuffer[samp * inchannels + chan]) + (0.5 * buffer.AtPosition(position - (data->delay_interval * length* numChannels)));
			//by inserting the outbuffer into the circular buffer and using it above it creates a loop where its constantly feeding back into itself
			//because the buffer is multiplyed by 0.5 the sound echo sound diminishes over time rather than staying 
			position = buffer.Place(outbuffer[(samp * *outchannels) + chan]);
			}
		}
	}
	return FMOD_OK;
}

//48000 is the samplerate


FMOD_RESULT F_CALLBACK myDSPCreateCallback(FMOD_DSP_STATE* dsp_state)
{
	unsigned int blocksize;
	int sampleRate;

	FMOD_RESULT result;
	//This finds the sampleRate which is useful to know for figuring out the delay in seconds
	result = dsp_state->callbacks->getblocksize(dsp_state, &blocksize);
	result = dsp_state->callbacks->getsamplerate(dsp_state, &sampleRate);

	mydsp_data_t* data = (mydsp_data_t*)calloc(sizeof(mydsp_data_t), 1);
	if (!data)
	{
		return FMOD_ERR_MEMORY;
	}
	dsp_state->plugindata = data; 
	//Initialise it to have 0 delay and be using the feedback delay
	data->delay_interval = 0;
	data->feedback_delay = 1;

	
	return FMOD_OK;
}


//Setting the paramers using the int callback lets you choose between two options
//an index of 0 changes the delay interval and an index of 1 or anything else changes the mode 
FMOD_RESULT F_CALLBACK myDSPSetParameterIntCallback(FMOD_DSP_STATE* dsp_state, int index, int value)
{
	
		mydsp_data_t* mydata = (mydsp_data_t*)dsp_state->plugindata;

		if (index == 0) {
			mydata->delay_interval = value;
		}
		else {
			mydata->feedback_delay = value;
		}

		return FMOD_OK;
	
}
//This retrieves the delay interaval and feedback delay bool 
FMOD_RESULT F_CALLBACK myDSPGetParameterIntCallback(FMOD_DSP_STATE* dsp_state, int index, int* value, char* valstr)
{
	
		mydsp_data_t* mydata = (mydsp_data_t*)dsp_state->plugindata;

		
		if (index == 0) {
			*value = mydata->delay_interval;
		}
		else {
			*value = mydata->feedback_delay;
		}

		return FMOD_OK;

}



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

	
	// Create the DSP effect
	{
		FMOD_DSP_DESCRIPTION dspdesc;
		memset(&dspdesc, 0, sizeof(dspdesc));

		//Sets up the parameters of the input data
		FMOD_DSP_PARAMETER_DESC delay_interval;
		FMOD_DSP_PARAMETER_DESC feedback_delay;

		FMOD_DSP_PARAMETER_DESC* paramdesc[2] =
		{
			&delay_interval,  
			&feedback_delay
		};

		FMOD_DSP_INIT_PARAMDESC_INT(delay_interval, "interval", "", "Time interval for how long the delay should be", 0, 200, 10, false, NULL);
		FMOD_DSP_INIT_PARAMDESC_INT(feedback_delay, "isFeedback", "", "Used to choose between feedback delay and regular delay", 0, 1, 1, false, NULL);

		strncpy_s(dspdesc.name, "My first DSP unit", sizeof(dspdesc.name));
		dspdesc.numinputbuffers = 1;
		dspdesc.numoutputbuffers = 1;
		dspdesc.read = DSPCallbackDelay;
		dspdesc.create = myDSPCreateCallback;

		//providing the setters and getters for the plugindata ints 
		dspdesc.setparameterint = myDSPSetParameterIntCallback;
		dspdesc.getparameterint = myDSPGetParameterIntCallback;
		//This is two because it is the interval and delay mode
		dspdesc.numparameters = 2;
		dspdesc.paramdesc = paramdesc;
		m_dsp->setUserData(&delay_interval); 
		m_dsp->setUserData(&feedback_delay);

		result = m_FmodSystem->createDSP(&dspdesc, &m_dsp);
		
		FmodErrorCheck(result);

		
		if (result != FMOD_OK)
			return false;
	}
	return true;
}

// Load an event sound
bool CAudio::LoadEventSound(char *filename)
{
	result = m_FmodSystem->createSound(filename, FMOD_LOOP_OFF, 0, &m_eventSound);
	FmodErrorCheck(result);
	if (result != FMOD_OK) 
		return false;

	return true;
}

// Play an event sound
bool CAudio::PlayEventSound()
{
	result = m_FmodSystem->playSound(m_eventSound, NULL, false, NULL);
	FmodErrorCheck(result);
	if (result != FMOD_OK)
		return false;
	return true;
}

//gets the current value of the delay then increases it and sets
void CAudio::increase_delay() {
	int delay;
	result = m_dsp->getParameterInt(0, &delay, 0, 0);
	if (delay <= 95) {
		delay += 5;
	}
	result = m_dsp->setParameterInt(0, delay);
}
//gets the current value of the delay then decreases it and sets

void CAudio::decrease_delay() {
	int delay;
	result = m_dsp->getParameterInt(0, &delay, 0, 0);
	if (delay >= 0) {
		delay -= 5;
	}
	result = m_dsp->setParameterInt(0, delay);

}
//swaps to whatever the mode it currently isnt is between 0 and 1, works like a bool 
void CAudio::swap_delay_mode() {
	int mode;
	result = m_dsp->getParameterInt(1, &mode, 0, 0);
	if (mode == 0) {
		mode = 1;
	}
	else { mode = 0; }
	result = m_dsp->setParameterInt(1, mode);

}

//This is a function to display the current mode on the screen, it doesnt have any other functionality 
std::string CAudio::get_delay_mode() {
	int mode = 0;
	result = m_dsp->getParameterInt(1, &mode, 0, 0);
	if (mode == 0) {
		return "Regular Delay";
	}
	if (mode == 1) {
		return "Feedback Delay";
	}
	else {
		return "Error";
	}

}

int CAudio::get_delay_interval() {
	int delay = 0;
	result = m_dsp->getParameterInt(0, &delay, 0, 0);
	return delay;
}
//This uses the samplerate to return how much the delay interval is in seconds

float CAudio::get_delay_interval_seconds() {
	int delay = 0;
	result = m_dsp->getParameterInt(0, &delay, 0, 0);
	float inSeconds = (delay * 1024)/48000.0 ;
	return inSeconds;
}


// Load a music stream
bool CAudio::LoadMusicStream(char *filename)
{
	result = m_FmodSystem->createStream(filename, NULL | FMOD_LOOP_NORMAL, 0, &m_music);
	FmodErrorCheck(result);

	
	if (result != FMOD_OK) 
		return false;

	return true;
	

}

// Play a music stream
bool CAudio::PlayMusicStream()
{
	result = m_FmodSystem->playSound(m_music, NULL, false, &m_musicChannel);
	FmodErrorCheck(result);

	if (result != FMOD_OK)
		return false;
	
	m_musicChannel->addDSP(0, m_dsp);

	return true;
}

void CAudio::Update()
{
	m_FmodSystem->update();
}