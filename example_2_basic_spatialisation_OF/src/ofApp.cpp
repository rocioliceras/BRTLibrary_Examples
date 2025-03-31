#include "ofApp.h"

#if defined(__linux__) || defined(linux)
	#include <bits/stdc++.h>
#endif

int iBufferSize;

//--------------------------------------------------------------
void ofApp::setup(){
	ofSetLogLevel(OF_LOG_NOTICE);


	/// Configure BRT Error handler
	BRT_ERRORHANDLER.SetVerbosityMode(VERBOSITYMODE_ERRORSANDWARNINGS);
	BRT_ERRORHANDLER.SetErrorLogStream(&std::cout, true);

	// /// Show introduction message
	ShowIntroduction();

	// /// Select Buffer Size
	 std::cout << "Insert wished buffer size (256, 512, 1024, 2048, 4096...)\n(2048 at least recommended for linux)\t: ";
	 std::cin >> iBufferSize;
	 std::cin.ignore();

	 /// AUDIO setup
	 AudioSetup();

	 /// BRT Global Parametert setup
	 globalParameters.SetSampleRate(SAMPLERATE); // Setting sample rate
	 globalParameters.SetBufferSize(iBufferSize); // Setting buffer size

	// //////////////////////////
	// // Listener Setup
	// //////////////////////////
	 brtManager.BeginSetup();
	 listener = brtManager.CreateListener<BRTBase::CListener>(LISTENER_ID);
	 brtManager.EndSetup();
	 Common::CTransform listenerPosition = Common::CTransform();		 // Setting listener in (0,0,0)
	 listenerPosition.SetPosition(Common::CVector3(0, 0, 0));
	 listener->SetListenerTransform(listenerPosition);

}

//--------------------------------------------------------------
void ofApp::update(){

}

//--------------------------------------------------------------
void ofApp::draw(){

}

//--------------------------------------------------------------
void ofApp::keyPressed(int key){

}

//--------------------------------------------------------------
void ofApp::keyReleased(int key){

}

//--------------------------------------------------------------
void ofApp::mouseMoved(int x, int y ){

}

//--------------------------------------------------------------
void ofApp::mouseDragged(int x, int y, int button){

}

//--------------------------------------------------------------
void ofApp::mousePressed(int x, int y, int button){

}

//--------------------------------------------------------------
void ofApp::mouseReleased(int x, int y, int button){

}

//--------------------------------------------------------------
void ofApp::mouseEntered(int x, int y){

}

//--------------------------------------------------------------
void ofApp::mouseExited(int x, int y){

}

//--------------------------------------------------------------
void ofApp::windowResized(int w, int h){

}

//--------------------------------------------------------------
void ofApp::gotMessage(ofMessage msg){

}

//--------------------------------------------------------------
void ofApp::dragEvent(ofDragInfo dragInfo){ 

}

/* void ofApp::audioOut(ofSoundBuffer & buffer) {
	//Overflow/underflow
	if (buffer.getNumFrames() == 0) {
		std::cout << "Stream underflow detected!" << std::endl;
	}

	// buffer initialization
	// Initializes buffer with zeros
	buffer.clear();
	
	audioProcess(buffer, buffer.getNumFrames());

	// Si tienes fuentes o posiciones que se mueven, actualízalas aquí
	MoveSource_CircularHorizontalPath();
	ShowSource2Position();
}*/


///////////////////////
// MENU
///////////////////////
// Function to display the introduction message
void ofApp::ShowIntroduction() {
	std::cout << "============================================\n";
	std::cout << "     Welcome to the BRT Library\n";
	std::cout << "============================================\n\n";
	std::cout << "BRT is a modular library designed to provide highly configurable rendering adaptable to various needs.\n";
	std::cout << "Thanks to its flexible structure, it allows multiple modules to be interconnected to optimize performance according to the user's specific requirements.\n\n";
	std::cout << "This example demonstrates different ways to instantiate and configure the modules, although many more possibilities are not explored here, such as rendering with multiple listeners.\n\n";
	std::cout << "The purpose of this example is to serve as an introductory guide to using the library. You are free to use, copy, or modify the code as needed.\n\n";

	std::cout << "As a demonstration, in this example, you will hear a woman's voice coming from your left while footsteps move around you simultaneously.\n";
	std::cout << "The screen will display the position of these footsteps in real-time.\n\n";

	std::cout << "============================================\n\n";
}

/// Audio output configuration, using ofxAudioFile
void ofApp::AudioSetup() {

	// Setting the output parameters
	ofSoundStreamSettings outputParameters;
	outputParameters.numOutputChannels = 2;

	//outputParameters.getOutDevice();
	ofSoundDevice device = ShowSelectAudioDeviceMenu();
	outputParameters.setOutDevice(device); //Aqui he elegido mis auriculares porque el por defecto no se escuchaba

	// Setting real-time audio output.
	setRealTimePriority();
	outputParameters.numBuffers = 4;

	outputParameters.sampleRate = SAMPLERATE; 
	outputParameters.bufferSize = iBufferSize;

	outputParameters.setOutListener(this);
	

	try {
		ofSoundStreamSetup(outputParameters);
	} catch (const std::exception & e) {
		std::cout << "\nERROR: Unable to setup sound stream." << '\n'
				  << "Exception: " << e.what() << std::endl;
	
	}
}

	/// Function to show the user a menu to choose the audio output device
	ofSoundDevice ofApp::ShowSelectAudioDeviceMenu() {

		auto devices = soundStream.getDeviceList();
		std::cout << "Number of audio devices found: " << devices.size() << std::endl;
		int connectedAudioDevices = soundStream.getNumOutputChannels();

		std::cout << std::endl
				  << std::endl;
		std::cout << "----------------------------------------" << std::endl;
		std::cout << "     List of available audio outputs" << std::endl;
		std::cout << "----------------------------------------" << std::endl
				  << std::endl;

		for (int i = 0; i < devices.size(); ++i) {
			std::cout << "Device #" << i << ": " << devices[i].name << std::endl;
			std::cout << "Output channels: " << devices[i].outputChannels << " ";
			std::cout << "Input channels: " << devices[i].inputChannels << " "; 
			std::cout << "Default output: " << (devices[i].isDefaultOutput ? "Yes" : "No") << std::endl;
			std::cout << std::endl;
		}

		std::cout << std::endl;
		int selectAudioDevice;
		do {
			std::cout << "Please choose which audio output you wish to use: ";
			std::cin >> selectAudioDevice;
			std::cin.clear();
			std::cin.ignore(INT_MAX, '\n');
		} while (!(selectAudioDevice > -1 && selectAudioDevice <= devices.size()));
		std::cout << std::endl;
		return devices[selectAudioDevice];
	}

