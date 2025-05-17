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
	AzimuthImage.load("azimuth_head.png");
	ElevationImage.load("elevation_head.png");
	VoiceImage.load("voice2.png");
	StepsImage.load("steps2.png");
	AxisImage.load("axis_head.png");

	if (!AxisImage.isAllocated()) {
		std::cerr << "ERROR: No se pudo cargar la imagen." << std::endl;
	} else {
		std::cout << "Imagen cargada correctamente: "
				  << AxisImage.getWidth() << "x" << AxisImage.getHeight() << std::endl;
	}
	ofSetWindowShape(2*Width,Height);

	center.set(Width / 2, Height / 2);
	dragPoint.set(center);
	elevationPoint.set(Width + Width / 2, Height / 2);

	azimuthX = Width / 2.0;
	azimuthY = Height / 2.0;

	azimuthX2 = Width/ 2.0;
	azimuthY2 = Height / 2.0;

	elevationX = Width/ 2.0;
	elevationY = Height / 2.0;

	elevationX2 = Width / 2.0;
	elevationY2 = Height / 2.0;

	axisX = (ofGetWidth() - AxisImage.getWidth()) / 2;
	axisY = 0;

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

ofPoint project3DTo2D(float x, float y, float z, ofPoint center, float scale) {
	float px = center.x + (x - y) * scale * cos(PI / 6); // 30°
	float py = center.y - (x + y) * scale * sin(PI / 6) - z * scale;
	return ofPoint(px, py);
}

void drawDashedLine(ofPoint a, ofPoint b, float dashLength = 5, float gapLength = 3) {
	ofSetColor(0); 
	float totalLength = a.distance(b);
	ofPoint dir = (b - a).getNormalized();
	for (float i = 0; i < totalLength; i += dashLength + gapLength) {
		ofPoint start = a + dir * i;
		ofPoint end = a + dir * MIN(i + dashLength, totalLength);
		ofDrawLine(start, end);
	}
}

//--------------------------------------------------------------
void ofApp::draw() {
	ofBackground(200, 200, 200); // Fondo gris claro

	AxisImage.draw(axisX, axisY);

	int iconSize = 60;

	// Parámetro de animación basado en tiempo
	float scaleAnim = 1.0 + 0.1 * sin(ofGetElapsedTimef() * 3.0);
	float scaleAnimInv = 1.0 + 0.1 * sin(ofGetElapsedTimef() * 3.0 + PI);

	// --- Centros ---
	float azimuthCenterX = Width / 2.0f;
	float azimuthCenterY = Height / 2.0f;
	float elevationCenterX = Width + Width / 2.0f;
	float elevationCenterY = Height / 2.0f;

	// --- Ejes y etiquetas ---
	float lineHalfLength = 220;
	ofSetColor(255);
	ofSetLineWidth(2);

	// Azimuth
	ofDrawLine(azimuthCenterX, azimuthCenterY - lineHalfLength, azimuthCenterX, azimuthCenterY + lineHalfLength); // Y
	ofDrawLine(azimuthCenterX - lineHalfLength, azimuthCenterY, azimuthCenterX + lineHalfLength, azimuthCenterY); // X
	ofDrawBitmapStringHighlight("+X", azimuthCenterX - 8, azimuthCenterY - lineHalfLength - 15);
	ofDrawBitmapStringHighlight("-X", azimuthCenterX - 8, azimuthCenterY + lineHalfLength + 5);
	ofDrawBitmapStringHighlight("-Y", azimuthCenterX + lineHalfLength + 8, azimuthCenterY + 4);
	ofDrawBitmapStringHighlight("+Y", azimuthCenterX - lineHalfLength - 28, azimuthCenterY + 4);

	// Elevation
	ofDrawLine(elevationCenterX, elevationCenterY - lineHalfLength, elevationCenterX, elevationCenterY + lineHalfLength); // Z
	ofDrawLine(elevationCenterX - lineHalfLength, elevationCenterY, elevationCenterX + lineHalfLength, elevationCenterY); // X
	ofDrawBitmapStringHighlight("+Z", elevationCenterX - 8, elevationCenterY - lineHalfLength - 15);
	ofDrawBitmapStringHighlight("-Z", elevationCenterX - 8, elevationCenterY + lineHalfLength + 5);
	ofDrawBitmapStringHighlight("+X", elevationCenterX + lineHalfLength + 8, elevationCenterY + 4);
	ofDrawBitmapStringHighlight("-X", elevationCenterX - lineHalfLength - 28, elevationCenterY + 4);

	ofSetLineWidth(1);

	// --- Cabezas (debajo de los iconos) ---
	float headScale = 0.25f;
	float azimuthW = AzimuthImage.getWidth() * headScale;
	float azimuthH = AzimuthImage.getHeight() * headScale;
	AzimuthImage.draw(azimuthCenterX - azimuthW / 2, azimuthCenterY - azimuthH / 2, azimuthW, azimuthH);

	float elevationW = ElevationImage.getWidth() * headScale;
	float elevationH = ElevationImage.getHeight() * headScale;
	ElevationImage.draw(elevationCenterX - elevationW / 2, elevationCenterY - elevationH / 2, elevationW, elevationH);

	// --- Dibujo 3D proyectado ---
	ofPoint coordCenter = ofPoint(axisX + AxisImage.getWidth() / 2, axisY + AxisImage.getHeight() * 0.65);
	float scale = 0.55f;

	ofPoint Px = project3DTo2D(x_AXIS, 0, 0, coordCenter, scale);
	ofPoint Py = project3DTo2D(0, y_AXIS, 0, coordCenter, scale);
	ofPoint Pz = project3DTo2D(0, 0, z_AXIS, coordCenter, scale);

	ofPoint Pxy = project3DTo2D(x_AXIS, y_AXIS, 0, coordCenter, scale);
	ofPoint Pxz = project3DTo2D(x_AXIS, 0, z_AXIS, coordCenter, scale);
	ofPoint Pyz = project3DTo2D(0, y_AXIS, z_AXIS, coordCenter, scale);
	ofPoint Pxyz = project3DTo2D(x_AXIS, y_AXIS, z_AXIS, coordCenter, scale);

	ofSetColor(255, 0, 0);
	ofDrawCircle(Pxyz, 6);

	ofSetColor(255);
	drawDashedLine(Px, Pxy);
	drawDashedLine(Pxy, Pxyz);
	drawDashedLine(Px, Pxz);
	drawDashedLine(Pxz, Pxyz);

	drawDashedLine(Py, Pxy);
	drawDashedLine(Py, Pyz);
	drawDashedLine(Pyz, Pxyz);

	drawDashedLine(Pz, Pxz);
	drawDashedLine(Pz, Pyz);

	// --- Voice y Steps (encima de las cabezas) ---
	float azVoiceSize = iconSize * scaleAnim;
	VoiceImage.draw(azimuthX - azVoiceSize / 2, azimuthY - azVoiceSize / 2, azVoiceSize, azVoiceSize);

	float azStepsSize = iconSize * scaleAnimInv;
	StepsImage.draw(azimuthX2 - azStepsSize / 2, azimuthY2 - azStepsSize / 2, azStepsSize, azStepsSize);

	float evVoiceSize = iconSize * scaleAnim;
	VoiceImage.draw(Width + elevationX - evVoiceSize / 2,
		Height - elevationY - evVoiceSize / 2, evVoiceSize, evVoiceSize);

	float evStepsSize = iconSize * scaleAnimInv;
	StepsImage.draw(Width + elevationX2 - evStepsSize / 2,
		Height - elevationY2 - evStepsSize / 2, evStepsSize, evStepsSize);

	ofSetColor(255); // reset
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
	float centerX = Width / 2.0f;
	float centerY = Height / 2.0f;

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
		x_AXIS = -dy;
		y_AXIS = dx;
	} else if (dragging == AZIMUTH_STEPS || dragging == ELEVATION_STEPS) {
		azimuthX2 = x;
		azimuthY2 = y;
		azimuth2 = ofDegToRad(angleDeg);
	}
}


void calcular_elevacion(int x, int y) {
	
	int localX = x - Width; 
	int localY = y;

	float centerX = Width / 2.0f; 
	float centerY = Height / 2.0f; 

	float dx = localX - centerX; 
	float dy = centerY - localY;

	if (dx == 0 && dy == 0) dx = 0.01f;

	float angleRad = atan2(dy, dx); 
	float angleDeg = ofRadToDeg(angleRad); 

	if (dx >= 0 && dy >= 0) {
		angleDeg = ofClamp(angleDeg, 0.0f, 90.0f);
	} else if (dx < 0 && dy >= 0) {
		angleDeg = 180 - angleDeg;
	} else if (dx < 0 && dy < 0) {
		angleDeg = -180 - angleDeg;
	} else if (dx >= 0 && dy < 0) {
		angleDeg = ofClamp(angleDeg, -90.0f, 0.0f);
	}

	if (dragging == AZIMUTH_VOICE || dragging == ELEVATION_VOICE) {
		elevationX = localX;
		elevationY = Height - localY; 
		elevation1 = ofDegToRad(angleDeg);
		x_AXIS = -dx;
		z_AXIS = dy;
	} else if (dragging == AZIMUTH_STEPS || dragging == ELEVATION_STEPS) {
		elevationX2 = localX;
		elevationY2 = Height - localY;
		elevation2 = ofDegToRad(angleDeg);
	}
}



void ofApp::mouseDragged(int x, int y, int button) {
	switch (dragging) {
	case AZIMUTH_VOICE: {
		azimuthX = x;
		azimuthY = y;

		calcular_azimuth(azimuthX, azimuthY);

		elevationX = Height - azimuthY;

		prevDragging = dragging;
		dragging = ELEVATION_VOICE;
		calcular_elevacion(Width + elevationX, Height - elevationY);
		dragging = prevDragging;

		break;
	}

	case ELEVATION_VOICE: {
		elevationX = x - Width;
		elevationY = Height - y;

		calcular_elevacion(Width + elevationX, Height - elevationY);

		azimuthY = Height - elevationX;

		prevDragging = dragging;
		dragging = AZIMUTH_VOICE;
		calcular_azimuth(azimuthX, azimuthY);
		dragging = prevDragging;

		break;
	}

	case AZIMUTH_STEPS: {
		azimuthX2 = x;
		azimuthY2 = y;

		calcular_azimuth(azimuthX2, azimuthY2);

		elevationX2 = Height - azimuthY2;

		prevDragging = dragging;
		dragging = ELEVATION_STEPS;
		calcular_elevacion(Width + elevationX2, Height - elevationY2);
		dragging = prevDragging;

		break;
	}

	case ELEVATION_STEPS: {
		elevationX2 = x - Width;
		elevationY2 = Height - y;

		calcular_elevacion(Width + elevationX2, Height - elevationY2);

		azimuthY2 = Height - elevationX2;

		prevDragging = dragging;
		dragging = AZIMUTH_STEPS;
		calcular_azimuth(azimuthX2, azimuthY2);
		dragging = prevDragging;

		break;
	}
	}
}





//--------------------------------------------------------------
void ofApp::mousePressed(int x, int y, int button) {
	float minDistance = 20.0f;
	int iconSize = 60;

	ofVec2f mouse(x, y);

	ofVec2f azVoice(azimuthX, azimuthY);
	ofVec2f azSteps(azimuthX2, azimuthY2);

	ofVec2f elVoice(Width + elevationX,
		Height - elevationY);
	ofVec2f elSteps(Width + elevationX2,
		Height - elevationY2);

	if (mouse.distance(elVoice) < minDistance) {
		dragging = ELEVATION_VOICE;
	} else if (mouse.distance(elSteps) < minDistance) {
		dragging = ELEVATION_STEPS;
	} else if (mouse.distance(azVoice) < minDistance) {
		dragging = AZIMUTH_VOICE;
	} else if (mouse.distance(azSteps) < minDistance) {
		dragging = AZIMUTH_STEPS;
	} else {
		dragging = NONE;
	}
}



void ofApp::mouseReleased(int x, int y, int button) {
	dragging = NONE;
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
		std::cout << "Source 2 --> Azimuth (" << rad2deg(azimuth2) << "), Elevation (" << rad2deg(elevation2) << "), Distance (" << distance2 << ")." << std::endl;
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
