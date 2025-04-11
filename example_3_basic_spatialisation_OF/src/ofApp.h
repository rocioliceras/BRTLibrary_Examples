#pragma once

#define LISTENER_ID "listener1"

#define SAMPLERATE 44100

#define SOURCE1_FILEPATH "resources/stereo_test.wav"
#define HRTFRESAMPLINGSTEP 15

#define SOURCE1_INITIAL_AZIMUTH 90
#define SOURCE1_INITIAL_ELEVATION 0
#define SOURCE1_INITIAL_DISTANCE 2

#define SOURCE2_INITIAL_AZIMUTH -90
#define SOURCE2_INITIAL_ELEVATION 0
#define SOURCE2_INITIAL_DISTANCE 2
#define SOURCE2_INITIAL_SPEED 0.001

#include "ConfigurationA.hpp"
#include "ConfigurationB.hpp"
#include "ConfigurationC.hpp"
#include "ofMain.h"
#include "ofxAudioFile.h"
#include "ofxBRT.h"
#include <BRTLibrary.h>
#include <cstdio>
#include <cstring>
#include <ofxAudioFile.h>

Common::CGlobalParameters globalParameters; // Class where the global BRT parameters are defined.
BRTBase::CBRTManager brtManager; // BRT global manager interface
std::shared_ptr<BRTBase::CListener> listener; // Pointer to listener mode
ofSoundStream soundStream;

std::shared_ptr<BRTSourceModel::CSourceSimpleModel> source1BRT; // Pointers to each audio source model
std::shared_ptr<BRTSourceModel::CSourceSimpleModel> source2BRT; // Pointers to each audio source model

CConfigurationA configurationA; // Configuration class for the example A
CConfigurationB configurationB; // Configuration class for the example B
CConfigurationC configurationC; // Configuration class for the example C

float source2Azimuth;
float source2Elevation;
float source2Distance;
int showSource2PositionCounter;

Common::CEarPair<CMonoBuffer<float>> outputBufferStereo; // Stereo buffer containing processed audio
std::vector<float> samplesVectorSource1; // Storages the audio from the wav files
std::vector<float> samplesVectorSource2; // Storages the audio from the wav files

unsigned int wavSamplePositionSource1; // Storages, respectively, the starting and ending position of the frame being rendered for each source
unsigned int positionEndFrameSource1;
unsigned int wavSamplePositionSource2;
unsigned int positionEndFrameSource2;

class ofApp : public ofBaseApp {

public:
	void setup();
	void update();
	void draw();

	void keyPressed(int key);
	void keyReleased(int key);
	void mouseMoved(int x, int y);
	void mouseDragged(int x, int y, int button);
	void mousePressed(int x, int y, int button);
	void mouseReleased(int x, int y, int button);
	void mouseEntered(int x, int y);
	void mouseExited(int x, int y);
	void windowResized(int w, int h);
	void dragEvent(ofDragInfo dragInfo);
	void gotMessage(ofMessage msg);

	void ShowIntroduction();
	void AudioSetup();
	ofSoundDevice ShowSelectAudioDeviceMenu();
	char ShowConfigurationMenu();

	void audioOut(ofSoundBuffer & buffer);
	void audioProcess(Common::CEarPair<CMonoBuffer<float>> & bufferOutput, int uiBufferSize);
	void FillBuffer(CMonoBuffer<float> & output, unsigned int & position, unsigned int & endFrame, std::vector<float> & samplesVector);
	void LoadWav(std::vector<float> & samplesVector1, std::vector<float> & samplesVector2, const char * stringIn);
	std::shared_ptr<BRTSourceModel::CSourceSimpleModel> CreateSimpleSoundSource(std::string _soundSourceID);
	void MoveSource_CircularHorizontalPath();
	Common::CVector3 Spherical2Cartesians(float azimuth, float elevation, float radius);
	void ShowSource2Position();
	float rad2deg(float rad);

private:
	void setRealTimePriority() {
		HANDLE hThread = GetCurrentThread();
		SetThreadPriority(hThread, THREAD_PRIORITY_TIME_CRITICAL);
	}
};
