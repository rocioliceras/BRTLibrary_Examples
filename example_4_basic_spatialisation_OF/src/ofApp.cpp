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

	// /////////////////////
	// Load Images
	/////////////////////

	AzimuthImage.load("azimuth.png");
	if (!AzimuthImage.isAllocated()) {
		std::cerr << "ERROR: No se pudo cargar la imagen." << std::endl;
	} else {
		std::cout << "Imagen cargada correctamente: "
				  << AzimuthImage.getWidth() << "x" << AzimuthImage.getHeight() << std::endl;
	}
	ofSetWindowShape(AzimuthImage.getWidth(), AzimuthImage.getHeight());

	center.set(AzimuthImage.getWidth() / 2, AzimuthImage.getHeight() / 2);
	dragPoint.set(center); // Inicialmente en el centro
	currentX = AzimuthImage.getWidth() / 2;
	currentY = AzimuthImage.getHeight() / 2;

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

	// Aquí solo se actualizan otras transformaciones generales
	Common::CTransform source1 = Common::CTransform();
	source1.SetPosition(Spherical2Cartesians(azimuth1, elevation1, distance1));
	source1BRT->SetSourceTransform(source1);

	// Aquí podrías hacer otras actualizaciones generales sin tocar el cálculo del ángulo
	ofSoundUpdate();
}

//--------------------------------------------------------------
void ofApp::draw() {
	ofBackground(0);

	// Dibuja la imagen centrada
	AzimuthImage.draw(0, 0);

	// Centro de la cabeza
	ofVec2f center(AzimuthImage.getWidth() / 2, AzimuthImage.getHeight() / 2);

	// Punto actual (el que mueves tú)
	ofVec2f point(currentX, currentY); // asegúrate de que estos valores estén dentro de la imagen

	// Dibuja los puntos
	ofSetColor(255, 0, 0);
	ofDrawCircle(point, 5); // tu punto móvil

	// Calcular ángulo con respecto al eje horizontal
	float angleRad = atan2(point.y - center.y, point.x - center.x);
	float angleDeg = ofRadToDeg(angleRad); // -180 a 180

	// Ajustar a tu sistema:
	// Frente (arriba) = 0°, Izquierda = 90°, Atrás = 180°, Derecha = 270°
	float customAngle = fmod(450 - angleDeg, 360); // esto lo mantiene entre 0 y 360

	if (customAngle < 0) customAngle += 360; // por si fmod devuelve negativo

	// Muestra el ángulo en pantalla
	ofSetColor(255);
	ofDrawBitmapString("Ángulo personalizado: " + ofToString(customAngle, 2) + "°", 20, 20);
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
void ofApp::mouseDragged(int x, int y, int button) {
	// Actualiza la posición del ratón
	currentX = x;
	currentY = y;

	// Calcular el centro de la imagen
	float centerX = AzimuthImage.getWidth() / 2.0;
	float centerY = AzimuthImage.getHeight() / 2.0;

	// Calcular las diferencias de las coordenadas del ratón
	float dx = currentX - centerX;
	float dy = centerY - currentY;

	// Calcular el ángulo en radianes
	float angleRad = atan2(dy, dx);

	// Convertir a grados
	float angleDeg = ofRadToDeg(angleRad);

	// Ajustar para que el "frente" (0°) esté hacia arriba
	angleDeg = angleDeg - 90;

	// Asegurarse de que el ángulo esté en el rango de 0–360 grados
	if (angleDeg < 0) {
		angleDeg += 360;
	}
	if (angleDeg >= 360) {
		angleDeg -= 360;
	}

	azimuth1 = 3.14/180*angleDeg;
	// Si lo necesitas, imprime el ángulo o actualiza la escena con este valor
	std::cout << "Ángulo: " << angleDeg << " grados" << std::endl;
}

//--------------------------------------------------------------
void ofApp::mousePressed(int x, int y, int button) {
}

//--------------------------------------------------------------
void ofApp::mouseReleased(int x, int y, int button) {
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
	std::cout << "The purpose of this example is to complement the first example using OpenFrameworks. You are free to use, copy, or modify the code as needed.\n\n";

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
		std::cout << "Source 2 --> Azimuth (" << azimuth1 << "), Elevation (" << rad2deg(elevation1) << "), Distance (" << distance1 << ")." << std::endl;
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
