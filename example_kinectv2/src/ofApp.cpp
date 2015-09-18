#include "ofApp.h"

#define DEPTH_WIDTH 512
#define DEPTH_HEIGHT 424
#define DEPTH_SIZE DEPTH_WIDTH * DEPTH_HEIGHT


//--------------------------------------------------------------
void ofApp::setup() {
	ofBackground(255);
	ofSetVerticalSync(true);

	kinect.open();
	kinect.initDepthSource();
	kinect.getSensor()->get_CoordinateMapper(&mapper);

    depthImg.allocate(DEPTH_WIDTH, DEPTH_HEIGHT, OF_IMAGE_GRAYSCALE);
    depthImg.setColor(ofColor::black);
    depthImg.update();

	tracker.setResolution(BF_HIGH_RES);
    
    camera.setPosition(ofGetWidth()/4.0, ofGetHeight()/2.0, 1000.);
	camera.lookAt(ofVec3f(0));
	mesh.setMode(OF_PRIMITIVE_POINTS);

	bDrawRawMesh.set("Draw Raw Mesh", false);
	thresholdNear.set("Threshold Near", 0.1, 0.1, 5);
	thresholdFar.set("Threshold Far", 1.5, 0.1, 8);

	boxMin.set("boxMin", ofVec3f(-8), ofVec3f(-10), ofVec3f( 0));
	boxMax.set("boxMax", ofVec3f( 8), ofVec3f(  0), ofVec3f(10));
	thresh3D.set("thresh3D", ofVec3f(.05), ofVec3f(0), ofVec3f(1));

	thresh2D.set("thresh2D", 1, 0, 10);
	minVol.set("minVol", 0.01, 0, 10);
	maxVol.set("maxVol", 2, 0, 100);
	minPoints.set("minPoints", 50, 0, 500);
	maxBlobs.set("maxBlobs", 10, 0, 100);
	trackedBlobs.set("trackdBlobs", 0, 0, maxBlobs);

	gui.setup("Settings", "settings.xml");
	gui.add(bDrawRawMesh);
	gui.add(thresholdNear);
	gui.add(thresholdFar);
	gui.add(boxMin);
	gui.add(boxMax);
	gui.add(thresh3D);
	gui.add(thresh2D);
	gui.add(minVol);
	gui.add(maxVol);
	gui.add(minPoints);
	gui.add(maxBlobs);
	gui.add(trackedBlobs);
	gui.loadFromFile("settings.xml");

	visible = true;
	save = false;
    
}

//--------------------------------------------------------------
void ofApp::update(){

    static bool bTrackerInit = false;
    
	kinect.update();
    
    // kinect 2 init is async, so just chill till it's ready, OK?
    if (kinect.getDepthSource()->isFrameNew()) {
		if (!bTrackerInit) {
			bTrackerInit = true;
			tracker.init(&kinect, false);
			tracker.setScale(ofVec3f(0.001));
		}

		updateDepthImage();
		updateMesh();
    }
    
    if ( tracker.isInited() ){
        tracker.findBlobs(&mesh, boxMin, boxMax, thresh3D, thresh2D, minVol, maxVol, minPoints, maxBlobs );
		trackedBlobs = tracker.nBlobs;
    }

}

//--------------------------------------------------------------
void ofApp::updateDepthImage() {
	if (!depthImg.isAllocated()) {
		depthImg.allocate(DEPTH_WIDTH, DEPTH_HEIGHT, OF_IMAGE_GRAYSCALE);
	}

	ofShortPixels pix;
	pix.allocate(DEPTH_WIDTH, 1, OF_IMAGE_GRAYSCALE);

	auto& depthPix = kinect.getDepthSource()->getPixels();
	for (int y = 0; y < DEPTH_HEIGHT; y++) {
		for (int x = 0; x < DEPTH_WIDTH; x++) {
			int index = x + (y*DEPTH_WIDTH);

			// Build the 8bit, thresholded image for drawing to screen
			if (depthPix.getWidth() && depthPix.getHeight()) {
				// Multiply thresholds by 1000 because the values are in meters in world
				// space but in mm in the depthPix array
				float depth = depthPix[index];
				float val = depth == 0 ? 0 : ofMap(depth, thresholdNear * 1000, thresholdFar * 1000, 255, 0, true);
				depthImg.setColor(x, y, ofColor(val));
			}
		}
	}

	depthImg.update();

}

//--------------------------------------------------------------
void ofApp::updateMesh() {

	// Map depth frame to world space
	vector<ofVec3f> worldCoords;
	worldCoords.resize(DEPTH_SIZE);

	auto& depthPix = kinect.getDepthSource()->getPixels();
	mapper->MapDepthFrameToCameraSpace(DEPTH_SIZE, (UINT16* )depthPix.getPixels(), DEPTH_SIZE, (CameraSpacePoint*) worldCoords.data());

	// Build the mesh
	mesh.clear();
	for (int y = 0; y < DEPTH_HEIGHT; y += tracker.getResolution()) {
		for (int x = 0; x < DEPTH_WIDTH; x += tracker.getResolution()) {
			int index = x + (y*DEPTH_WIDTH);
			float z = worldCoords[index].z;

			// Set an empty point (but don't discard it) if  depth value is
			// outside of thresholds
			if (z == 0 || z < thresholdNear || z > thresholdFar) {
				mesh.addVertex(ofVec3f::zero());
			} else {
				mesh.addVertex(worldCoords[index]);
			}
		}
	}
}

//--------------------------------------------------------------
void ofApp::draw(){
    
	ofSetColor(ofColor::white);

    if ( tracker.isInited() ){
        
        depthImg.draw(ofGetWidth()-DEPTH_WIDTH/2., 0, DEPTH_WIDTH/2., DEPTH_HEIGHT/2.);
        
        glPointSize(2.0);
        ofPushMatrix();
        camera.begin();
        ofScale(100., 100., 100.);
        ofEnableDepthTest();

		if (bDrawRawMesh) {
			ofSetColor(ofColor::blueSteel);
			mesh.draw();
		}
		else {
			// draw blobs
			for (unsigned int i = 0; i < tracker.blobs.size(); i++) {
				ofPushMatrix();
				ofColor color;
				color.setSaturation(200);
				color.setBrightness(225);
				color.setHue(ofMap(i, 0, tracker.blobs.size(), 0, 255));

				ofSetColor(color);
				// draw blobs
				tracker.blobs[i].draw();

				ofPopMatrix();

				ofVec3f bbMax = tracker.blobs[i].boundingBoxMax;
				ofVec3f bbMin = tracker.blobs[i].boundingBoxMin;

				ofNoFill();
				ofDrawBox(tracker.blobs[i].centroid, tracker.blobs[i].dimensions.x, tracker.blobs[i].dimensions.y, tracker.blobs[i].dimensions.z);
				ofFill();
			}
		}

        ofDisableDepthTest();
        camera.end();
        ofPopMatrix();
        
		if (visible) gui.draw();
        
    }
    
}

//--------------------------------------------------------------
void ofApp::keyPressed(int key){
//    return; // currently effed in 0.9.0 on github!
    
    if ( key == 'g' ){
		visible ^= 1;
    }
    
    if ( key == 's' ){
        //save = false;
        //gui->saveSettings("settings.xml");
    }
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
