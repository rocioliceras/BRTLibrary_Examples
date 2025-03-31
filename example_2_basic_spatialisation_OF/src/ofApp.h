#pragma once

#include "ofMain.h"
#include <cstdio>
#include <cstring>
#include "ofxBRT.h"
#include "ofxAudioFile.h"
#include <BRTLibrary.h>
#include "ConfigurationA.hpp"
#include "ConfigurationB.hpp"
#include "ConfigurationC.hpp"

#define LISTENER_ID "listener1"

#define SAMPLERATE 44100


Common::CGlobalParameters globalParameters; // Class where the global BRT parameters are defined.
BRTBase::CBRTManager brtManager; // BRT global manager interface
std::shared_ptr<BRTBase::CListener> listener; // Pointer to listener mode
ofSoundStream soundStream;


/* Common::CEarPair<CMonoBuffer<float>> outputBufferStereo; // Stereo buffer containing processed audio
std::vector<float> samplesVectorSource1; // Storages the audio from the wav files
std::vector<float> samplesVectorSource2; // Storages the audio from the wav files
*/

class ofApp : public ofBaseApp {

	public:
		void setup();
		void update();
		void draw();

		void keyPressed(int key);
		void keyReleased(int key);
		void mouseMoved(int x, int y );
		void mouseDragged(int x, int y, int button);
		void mousePressed(int x, int y, int button);
		void mouseReleased(int x, int y, int button);
		void mouseEntered(int x, int y);
		void mouseExited(int x, int y);
		void windowResized(int w, int h);
		void dragEvent(ofDragInfo dragInfo);
		void gotMessage(ofMessage msg);

		//void audioOut(ofSoundBuffer & buffer);	

		void ShowIntroduction();
		void AudioSetup();
		ofSoundDevice ShowSelectAudioDeviceMenu();
		char ShowConfigurationMenu();
		void ShowSource2Position();
		


private:
		
		void setRealTimePriority() {
			HANDLE hThread = GetCurrentThread();
			SetThreadPriority(hThread, THREAD_PRIORITY_TIME_CRITICAL);
		}
		
};


