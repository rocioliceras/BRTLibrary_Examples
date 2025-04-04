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




	 /////////////////////////////////////
	 // Create and connnect BRT modules
	 /////////////////////////////////////
	 char selection = ShowConfigurationMenu();
	 if (selection == 'A') {
		 configurationA.Setup(&brtManager, LISTENER_ID);
		 configurationA.LoadResources(&brtManager, LISTENER_ID);
	 } else if (selection == 'B') {
		 configurationB.Setup(&brtManager, LISTENER_ID);
		 configurationB.LoadResources(&brtManager, LISTENER_ID);
		 configurationB.ConfigureFreeFieldEnviromentModel(&brtManager, true, true);
	 } else if (selection == 'C') {
		 configurationC.Setup(&brtManager, LISTENER_ID);
		 configurationC.LoadResources(&brtManager, LISTENER_ID);
	 } else {
		 std::cout << "Invalid option. Exiting program.\n";
		
	 }

	 /////////////////////
	 // Create Sources
	 /////////////////////
	 source1BRT = CreateSimpleSoundSource("speech");
	 source2BRT = CreateSimpleSoundSource("steps");

	 if (selection == 'A') {
		 configurationA.ConnectSoundSource(&brtManager, "speech");
		 configurationA.ConnectSoundSource(&brtManager, "steps");
	 } else if (selection == 'B') {
		 configurationB.ConnectSoundSource(&brtManager, "speech");
		 configurationB.ConnectSoundSource(&brtManager, "steps");
	 } else if (selection == 'C') {
		 configurationC.ConnectSoundSource(&brtManager, "speech");
		 configurationC.ConnectSoundSource(&brtManager, "steps");
	 }

	 /////////////////////
	 // Setup Sources
	 /////////////////////
	 Common::CTransform source1 = Common::CTransform();
	 source1.SetPosition(Spherical2Cartesians(SOURCE1_INITIAL_AZIMUTH, SOURCE1_INITIAL_ELEVATION, SOURCE1_INITIAL_DISTANCE));
	 source1BRT->SetSourceTransform(source1);

	 source2Azimuth = SOURCE2_INITIAL_AZIMUTH;
	 source2Elevation = SOURCE2_INITIAL_ELEVATION;
	 source2Distance = SOURCE2_INITIAL_DISTANCE;
	 Common::CTransform source2 = Common::CTransform();
	 source2.SetPosition(Spherical2Cartesians(source2Azimuth, source2Elevation, source2Distance));
	 source2BRT->SetSourceTransform(source2); // Set source2 position
	 showSource2PositionCounter = 0;

	 // /////////////////////
	 // Load Wav Files
	 /////////////////////
	 LoadWav(samplesVectorSource1, SOURCE1_FILEPATH); // Loading .wav file
	 LoadWav(samplesVectorSource2, SOURCE2_FILEPATH); // Loading .wav file

	  /////////////////////
	 // Start AUDIO Render
	 /////////////////////
	 // Declaration and initialization of stereo buffer
	 outputBufferStereo.left.resize(iBufferSize);
	 outputBufferStereo.right.resize(iBufferSize);

	 // Informing user by the console to press any key to end the execution
	 std::cout << std::endl<< std::endl;
	 std::cout << "Press any key to start and later press ENTER to finish..." << std::endl;
	 std::cout << std::endl<< std::endl;
	 std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n'); // Clear input buffer
	 std::cin.get(); // Wait for a key press

	 // Starting the stream
	 soundStream.start();

	 // Wait enter to finish
	 std::cin.ignore();
	 char temp = getchar();

	 // Stopping and closing the stream
	 soundStream.stop();
	 soundStream.close();

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

	std::cout << "this en setup(): " << this << std::endl;
	std::cout << "soundStream antes de setOutListener: " << &soundStream << std::endl;
	outputParameters.setOutListener(this);

	

	try {
		soundStream.setup(outputParameters);
		soundStream.stop();
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

void ofApp::audioOut(ofSoundBuffer & buffer) {
	unsigned int uiBufferSize = (unsigned int)iBufferSize;
	// Setting the output buffer as float
	std::vector<float> & bufferData = buffer.getBuffer();

	// Obtén un puntero al primer elemento del vector
	float * floatOutputBuffer = &bufferData[0]; // O también bufferData.data()

	//Overflow/underflow
	if (buffer.getNumFrames() == 0) {
		std::cout << "Stream underflow detected!" << std::endl;
	}

	// Initializes buffer with zeros
	outputBufferStereo.left.Fill(uiBufferSize, 0.0f);
	outputBufferStereo.right.Fill(uiBufferSize, 0.0f);

	audioProcess(outputBufferStereo, uiBufferSize);

	// Declaration and initialization of interlaced audio vector for correct stereo output
	CStereoBuffer<float> iOutput;
	iOutput.Interlace(outputBufferStereo.left, outputBufferStereo.right);

	// Buffer filling loop
	for (auto it = iOutput.begin(); it != iOutput.end(); it++) {
		floatOutputBuffer[0] = *it; // Setting of value in actual buffer position
		floatOutputBuffer = &floatOutputBuffer[1]; // Updating pointer to next buffer position
	}

	
	MoveSource_CircularHorizontalPath();
	ShowSource2Position();
}

	/// Function to process audio
void ofApp::audioProcess(Common::CEarPair<CMonoBuffer<float>> & bufferOutput, int uiBufferSize) {
	// Declaration, initialization and filling mono buffers
	CMonoBuffer<float> source1Input(uiBufferSize);
	FillBuffer(source1Input, wavSamplePositionSource1, positionEndFrameSource1, samplesVectorSource1);
	CMonoBuffer<float> source2Input(uiBufferSize);
	FillBuffer(source2Input, wavSamplePositionSource2, positionEndFrameSource2, samplesVectorSource2);

	// Declaration of stereo buffer
	Common::CEarPair<CMonoBuffer<float>> bufferProcessed;

	source1BRT->SetBuffer(source1Input); // Set samples in the sound source
	source2BRT->SetBuffer(source2Input); // Set samples in the sound source
	brtManager.ProcessAll(); // Process all
	listener->GetBuffers(bufferProcessed.left, bufferProcessed.right); // Get out buffers

	bufferOutput.left += bufferProcessed.left;
	bufferOutput.right += bufferProcessed.right;
}

/// Function to fill buffer with audio from the wav file
void ofApp::FillBuffer(CMonoBuffer<float> & output, unsigned int & position, unsigned int & endFrame, std::vector<float> & samplesVector) {
	position = endFrame + 1; // Set starting point as next sample of the end of last frame
	if (position >= samplesVector.size()) // If the end of the audio is met, the position variable must return to the beginning
		position = 0;

	endFrame = position + output.size() - 1; // Set ending point as starting point plus frame size
	for (int i = 0; i < output.size(); i++) {
		if ((position + i) < samplesVector.size())
			output[i] = (samplesVector[position + i]); // Fill with audio
		else
			output[i] = 0.0f; // Fill with zeros if the end of the audio is met
	}
}

void ofApp::LoadWav(std::vector<float> & samplesVector, const char * stringIn) {
	ofxAudioFile audioFile;
	audioFile.load(stringIn);

	// Check if the WAV file was successfully loaded
	if (!audioFile.loaded()) {
		std::cerr << "Failed to load WAV file: " << stringIn << std::endl;
		return;
	}

	// Get the total number of samples per channel
	size_t numSamples = audioFile.length();

	// Clear the vector before filling it
	samplesVector.clear();
	samplesVector.reserve(numSamples); // Reserve memory to avoid reallocations

	// Extract samples (only from the left channel if stereo)
	for (size_t i = 0; i < numSamples; i++) {
		samplesVector.push_back(audioFile.sample(i, 0)); // 0 = left channel
	}
}


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



	// Function to display the configuration menu and get the user’s choice
	char ofApp::ShowConfigurationMenu() {
		char option;

		do {
			std::cout << "===== CONFIGURATION MENU =====\n";
			std::cout << "A) Sources --> Listener HRTF Model (Nearfield + Convolution) --> Listener\n";
			std::cout << "B) Sources --> Environment Model (FreeField) --> ListenerHRTFModel (Nearfield + Convolution) --> Listener\n";
			std::cout << "C) Sources|--> Listener HRTF Model (Nearfield + Convolution) --> |Listener\n";
			std::cout << "          |--> Listener BRIR Model (Ambisonic)               --> |\n";
			std::cout << "D) No yet implemented\n";
			std::cout << "Select an option (A-D): ";
			std::cin >> option;

			// Convert to uppercase to avoid issues with lowercase input
			option = std::toupper(option);

			if (option < 'A' || option > 'D') {
				std::cout << "❌ Invalid option. Please try again.\n";
			}
		} while (option < 'A' || option > 'D'); // Repeat until a valid option is chosen

		return option;
	}

	
	///////////////////////
	// SOURCE MOVEMENT
	///////////////////////
	void ofApp::MoveSource_CircularHorizontalPath() {

		Common::CVector3 newPosition;
		source2Azimuth += SOURCE2_INITIAL_SPEED;
		if (source2Azimuth > 2 * M_PI) source2Azimuth = 0;
		newPosition = Spherical2Cartesians(source2Azimuth, source2Elevation, source2Distance);

		Common::CTransform sourcePosition = source2BRT->GetSourceTransform();
		sourcePosition.SetPosition(newPosition);
		source2BRT->SetSourceTransform(sourcePosition);
	}


	Common::CVector3 ofApp::Spherical2Cartesians(float azimuth, float elevation, float radius) {

		float x = radius * cos(azimuth) * cos(elevation);
		float y = radius * sin(azimuth) * cos(elevation);
		float z = radius * sin(elevation);

		Common::CVector3 pos = listener->GetListenerTransform().GetPosition();

		pos.x = pos.x + x;
		pos.y = pos.y + y;
		pos.z = 0.0f;

		return pos;
	}

	void ofApp::ShowSource2Position() {

		showSource2PositionCounter++;
		if (showSource2PositionCounter == 25) {
			showSource2PositionCounter = 0;
			std::cout << "Source 2 --> Azimuth (" << rad2deg(source2Azimuth) << "), Elevation (" << rad2deg(source2Elevation) << "), Distance (" << source2Distance << ")." << std::endl;
		}
	}
	float ofApp::rad2deg(float rad) {

		return (rad * 180.0) / M_PI;
	}

	///////////////////////
	// BRT SETUP
	///////////////////////
	std::shared_ptr<BRTSourceModel::CSourceSimpleModel> ofApp::CreateSimpleSoundSource(std::string _soundSourceID) {
		brtManager.BeginSetup();
		std::shared_ptr<BRTSourceModel::CSourceSimpleModel> _brtSoundSource = brtManager.CreateSoundSource<BRTSourceModel::CSourceSimpleModel>(_soundSourceID);
		brtManager.EndSetup();
		if (_brtSoundSource == nullptr) {
			std::cout << "Error creating sound source" << std::endl;
		}
		return _brtSoundSource;
	}
