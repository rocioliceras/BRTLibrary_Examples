/**
* \class ofApp
*
* \date	April 2025
*
* \authors 3DI-DIANA Research Group (University of Malaga), in alphabetical order: M. Cuevas-Rodriguez, D. Gonzalez-Toledo, L. Molina-Tanco, F. Morales-Benitez ||
* Coordinated by , A. Reyes-Lecuona (University of Malaga)||
* \b Contact: areyes@uma.es
*
* \b Copyright: University of Malaga
* 
* \b Contributions: R. Liceras-Ramirez
*
* \b Project: SONICOM ||
* \b Website: https://www.sonicom.eu/
*
* \b Acknowledgement: This project has received funding from the European Union�s Horizon 2020 research and innovation programme under grant agreement no.101017743
* 
* \b Licence: This program is free software, you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
*/

#include "ofApp.h"

#if defined(__linux__) || defined(linux)
	#include <bits/stdc++.h>
#endif

int iBufferSize;

//--------------------------------------------------------------
void ofApp::setup() {
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
	Common::CTransform listenerPosition = Common::CTransform(); // Setting listener in (0,0,0)
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

	Common::CTransform source2 = Common::CTransform();
	source2.SetPosition(Spherical2Cartesians(SOURCE2_INITIAL_AZIMUTH, SOURCE2_INITIAL_ELEVATION, SOURCE2_INITIAL_DISTANCE));
	source2BRT->SetSourceTransform(source2); 

	// /////////////////////
	// Load Wav Files
	/////////////////////
	LoadWav(samplesVectorSource1, SOURCE1_FILEPATH); // Loading .wav file
	LoadWav(samplesVectorSource2, SOURCE2_FILEPATH); // Loading .wav file

	// /////////////////////
	// Load Images
	/////////////////////

	AzimuthImage.load("azimuth.jpg");
	ElevationImage.load("elevation.jpg");
	VoiceImage.load("voice.png");
	StepsImage.load("steps.png");

	if (!AzimuthImage.isAllocated()) {
		std::cerr << "ERROR: No se pudo cargar la imagen." << std::endl;
	} else {
		std::cout << "Imagen cargada correctamente: "
				  << AzimuthImage.getWidth() << "x" << AzimuthImage.getHeight() << std::endl;
	}
	ofSetWindowShape(AzimuthImage.getWidth()+ElevationImage.getWidth(), AzimuthImage.getHeight());

	center.set(AzimuthImage.getWidth() / 2, AzimuthImage.getHeight() / 2);
	dragPoint.set(center); // Inicialmente en el centro
	elevationPoint.set(AzimuthImage.getWidth() + ElevationImage.getWidth() / 2, AzimuthImage.getHeight() / 2);

	azimuthX = AzimuthImage.getWidth() / 2.0;
	azimuthY = AzimuthImage.getHeight() / 2.0;

	azimuthX2 = AzimuthImage.getWidth() / 2.0;
	azimuthY2 = AzimuthImage.getHeight() / 2.0;


	elevationX = ElevationImage.getWidth() / 2.0;
	elevationY = ElevationImage.getHeight() / 2.0;

	elevationX2 = ElevationImage.getWidth() / 2.0;
	elevationY2 = ElevationImage.getHeight() / 2.0;

	/////////////////////
	// Start AUDIO Render
	/////////////////////
	// Declaration and initialization of stereo buffer
	outputBufferStereo.left.resize(iBufferSize);
	outputBufferStereo.right.resize(iBufferSize);

	// Informing user by the console to press any key to end the execution
	std::cout << std::endl
			  << std::endl;
	std::cout << "Press any key to start and later press ENTER to finish..." << std::endl;
	std::cout << std::endl
			  << std::endl;
	std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n'); // Clear input buffer
	std::cin.get(); // Wait for a key press

	// Starting the stream
	soundStream.start();

	//Esto se deberá hacer en otro sitio porque si no el código se queda bloqueado y no salta el interfaz
	/*// Wait enter to finish
	std::cin.ignore();
	char temp = getchar();

	// Stopping and closing the stream
	soundStream.stop();
	soundStream.close();*/
}

//--------------------------------------------------------------
void ofApp::update() {
	ofBackground(255, 255, 255);

	Common::CTransform source1 = Common::CTransform();
	source1.SetPosition(Spherical2Cartesians(azimuth1, elevation1, distance1));
	if (source1BRT) {
		source1BRT->SetSourceTransform(source1);
	}

	Common::CTransform source2 = Common::CTransform();
	source2.SetPosition(Spherical2Cartesians(azimuth2, elevation2, distance2));
	if (source2BRT) {
		source2BRT->SetSourceTransform(source2);
	}

	ofSoundUpdate();
}

//--------------------------------------------------------------
void ofApp::draw() {
	ofBackground(255);

	// Dibuja las imágenes de azimuth y elevación
	AzimuthImage.draw(0, 0);
	ElevationImage.draw(AzimuthImage.getWidth(), 0);

	int iconSize = 60;

	// Azimuth - voice
	VoiceImage.draw(azimuthX - iconSize / 2, azimuthY - iconSize / 2, iconSize, iconSize);

	// Azimuth - steps
	StepsImage.draw(azimuthX2 - iconSize / 2, azimuthY2 - iconSize / 2, iconSize, iconSize);

	// Elevation - voice
	VoiceImage.draw(AzimuthImage.getWidth() + elevationX,
		ElevationImage.getHeight() - elevationY - iconSize / 2, iconSize, iconSize);

	// Elevation - steps
	StepsImage.draw(AzimuthImage.getWidth() + elevationX2,
		ElevationImage.getHeight() - elevationY2 - iconSize / 2, iconSize, iconSize);
}


//--------------------------------------------------------------
void ofApp::keyPressed(int key) {

}

//--------------------------------------------------------------
void ofApp::keyReleased(int key) {
	
}

//--------------------------------------------------------------
void ofApp::mouseMoved(int x, int y) {
}

//--------------------------------------------------------------
void calcular_azimuth(int x, int y) {
	float centerX = AzimuthImage.getWidth() / 2.0f;
	float centerY = AzimuthImage.getHeight() / 2.0f;

	float dx = x - centerX;
	float dy = centerY - y;

	float angleRad = atan2(dy, dx);
	float angleDeg = ofRadToDeg(angleRad) - 90.0f;

	if (angleDeg < 0) angleDeg += 360.0f;
	if (angleDeg >= 360.0f) angleDeg -= 360.0f;

	if (dragging == AZIMUTH_VOICE || dragging == ELEVATION_VOICE) {
		azimuthX = x;
		azimuthY = y;
		azimuth1 = ofDegToRad(angleDeg);
	} else if (dragging == AZIMUTH_STEPS || dragging == ELEVATION_STEPS) {
		azimuthX2 = x;
		azimuthY2 = y;
		azimuth2 = ofDegToRad(angleDeg);
	}
}


void calcular_elevacion(int x, int y) {
	// Convertir coordenadas globales del mouse a locales de ElevationImage
	int localX = x - AzimuthImage.getWidth();
	int localY = y;

	float centerX = ElevationImage.getWidth() / 2.0f;
	float centerY = ElevationImage.getHeight() / 2.0f;

	float dx = localX - centerX;
	float dy = centerY - localY; // Invertimos Y: hacia arriba es positivo

	if (dx == 0 && dy == 0) dx = 0.01f;

	float angleRad = atan2(dy, dx);
	float angleDeg = ofRadToDeg(angleRad);

	// Aquí es donde hacemos la diferencia:
	// En lugar de mapear todo de 0 a 360°, dejamos los valores negativos.
	// Resultado:
	//   Derecha → 0°
	//   Arriba  → +90°
	//   Abajo   → -90°
	//   Izquierda → ±180°

	if (dragging == AZIMUTH_VOICE || dragging == ELEVATION_VOICE) {
		elevationX = localX;
		elevationY = ElevationImage.getHeight() - localY;
		elevation1 = ofDegToRad(angleDeg);
	} else if (dragging == AZIMUTH_STEPS || dragging == ELEVATION_STEPS) {
		elevationX2 = localX;
		elevationY2 = ElevationImage.getHeight() - localY;
		elevation2 = ofDegToRad(angleDeg);
	}

	// Debug
	std::cout << "Ángulo elevación: " << angleDeg << "°" << std::endl;
}


void ofApp::mouseDragged(int x, int y, int button) {
	switch (dragging) {
	case AZIMUTH_VOICE: {
		// 1. Actualizamos la posición en Azimuth
		azimuthX = x;
		azimuthY = y;

		// 2. Recalculamos la posición en Azimuth
		calcular_azimuth(azimuthX, azimuthY);

		// 3. Recalculamos la posición compartida en Elevación
		// Elevación es calculada en función de azimuthY
		elevationX = AzimuthImage.getHeight() - azimuthY;
		calcular_elevacion(AzimuthImage.getWidth() + elevationX, ElevationImage.getHeight() - elevationY);

		break;
	}

	case ELEVATION_VOICE: {
		// 1. Actualizamos la posición en Elevación
		elevationX = x - AzimuthImage.getWidth();
		elevationY = ElevationImage.getHeight() - y;

		// 2. Recalculamos la posición en Elevación
		calcular_elevacion(AzimuthImage.getWidth() + elevationX, ElevationImage.getHeight() - elevationY);

		// 3. Recalculamos la posición compartida en Azimuth
		// Azimuth es calculado en función de elevationX
		azimuthY = AzimuthImage.getHeight() - elevationX;
		calcular_azimuth(azimuthX, azimuthY);

		break;
	}

	case AZIMUTH_STEPS: {
		// 1. Actualizamos la posición de los pasos en Azimuth
		azimuthX2 = x;
		azimuthY2 = y;

		// 2. Recalculamos la posición de los pasos en Azimuth
		calcular_azimuth(azimuthX2, azimuthY2);

		// 3. Recalculamos la posición compartida en Elevación
		elevationX2 = AzimuthImage.getHeight() - azimuthY2;
		calcular_elevacion(AzimuthImage.getWidth() + elevationX2, ElevationImage.getHeight() - elevationY2);

		break;
	}

	case ELEVATION_STEPS: {
		// 1. Actualizamos la posición de los pasos en Elevación
		elevationX2 = x - AzimuthImage.getWidth();
		elevationY2 = ElevationImage.getHeight() - y;

		// 2. Recalculamos la posición de los pasos en Elevación
		calcular_elevacion(AzimuthImage.getWidth() + elevationX2, ElevationImage.getHeight() - elevationY2);

		// 3. Recalculamos la posición compartida en Azimuth
		azimuthY2 = AzimuthImage.getHeight() - elevationX2;
		calcular_azimuth(azimuthX2, azimuthY2);

		break;
	}
	}
}



//--------------------------------------------------------------
void ofApp::mousePressed(int x, int y, int button) {
	float minDistance = 20.0f; // Radio de detección de los íconos

	ofVec2f mouse(x, y);
	ofVec2f azVoice(azimuthX, azimuthY);
	ofVec2f azSteps(azimuthX2, azimuthY2);

	// Coordenadas de los íconos de elevación
	ofVec2f elVoice(AzimuthImage.getWidth() + elevationX, ElevationImage.getHeight() - elevationY);
	ofVec2f elSteps(AzimuthImage.getWidth() + elevationX2, ElevationImage.getHeight() - elevationY2);

	// Verifica si el clic está dentro del área de la fuente en la elevación
	if (mouse.distance(elVoice) < minDistance) {
		dragging = ELEVATION_VOICE;
		std::cout << "Clic en voz elevación" << std::endl;
	} else if (mouse.distance(elSteps) < minDistance) {
		dragging = ELEVATION_STEPS;
		std::cout << "Clic en pasos elevación" << std::endl;
	} else if (mouse.distance(azVoice) < minDistance) {
		dragging = AZIMUTH_VOICE;
		std::cout << "Clic en voz azimuth" << std::endl;
	} else if (mouse.distance(azSteps) < minDistance) {
		dragging = AZIMUTH_STEPS;
		std::cout << "Clic en pasos azimuth" << std::endl;
	} else {
		dragging = NONE;
	}
}


void ofApp::mouseReleased(int x, int y, int button) {
	dragging = NONE; // Cuando se suelta el ratón, dejamos de arrastrar
}


//--------------------------------------------------------------
void ofApp::mouseEntered(int x, int y) {
}

//--------------------------------------------------------------
void ofApp::mouseExited(int x, int y) {
}

//--------------------------------------------------------------
void ofApp::windowResized(int w, int h) {
}

//--------------------------------------------------------------
void ofApp::gotMessage(ofMessage msg) {
}

//--------------------------------------------------------------
void ofApp::dragEvent(ofDragInfo dragInfo) {
}

/// Audio output configuration, using ofxAudioFile
void ofApp::AudioSetup() {

	// Setting the output parameters
	ofSoundStreamSettings outputParameters;
	outputParameters.numOutputChannels = 2;

	//outputParameters.getOutDevice();
	ofSoundDevice device = ShowSelectAudioDeviceMenu();
	outputParameters.setOutDevice(device);

	// Setting real-time audio output.
	setRealTimePriority();
	outputParameters.numBuffers = 4;

	outputParameters.sampleRate = SAMPLERATE;
	outputParameters.bufferSize = iBufferSize;

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

	//Pointer to the first element of the vector
	float * floatOutputBuffer = &bufferData[0];

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
	std::cout << "This example allows the user to move two different sources along two axes: azimuth and elevation.\n\n";
	std::cout << "The purpose of this example is to demonstrate user interaction with OpenFrameworks.\n\n";

	std::cout << "As a demonstration, in this example, you will hear a woman's voice and some footsteps.\n";
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


 Common::CVector3 ofApp::Spherical2Cartesians(float azimuth, float elevation, float radius) {

	// Suavizado de la elevación, solo si el valor es distinto al anterior
	 float alpha = 0.1f; // Suavizado de la elevación
	 if (prevElevation != elevation) {
		 // Aplicar suavizado solo si la elevación es diferente
		 elevation = prevElevation + alpha * (elevation - prevElevation);
	 }

	 // Limitar la elevación entre -90 y 90 grados
	 elevation = std::max(-89.9f, std::min(89.9f, elevation));
	 prevElevation = elevation; // Actualizar la elevación previa

	 // Convertir a coordenadas cartesianas
	 float x = radius * cos(azimuth) * cos(elevation);
	 float y = radius * sin(azimuth) * cos(elevation);
	 float z = radius * sin(elevation);

	 // Obtener la posición actual del oyente
	 Common::CVector3 pos = listener->GetListenerTransform().GetPosition();

	 // Suavizado entre la posición actual y la nueva
	 pos.x = pos.x + 0.1f * (x - pos.x);
	 pos.y = pos.y + 0.1f * (y - pos.y);
	 pos.z = pos.z + 0.1f * (z - pos.z);

	 return pos;
}

void ofApp::ShowSource2Position() {

	showSource2PositionCounter++;
	if (showSource2PositionCounter == 25) {
		showSource2PositionCounter = 0;
		std::cout << "Source 2 --> Azimuth (" << rad2deg(azimuth1) << "), Elevation (" << rad2deg(elevation1) << "), Distance (" << distance1 << ")." << std::endl;
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
