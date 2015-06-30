#include "ofApp.h"

ofVec3f boxMin(-5000,-5000,-5000);
ofVec3f boxMax(5000,5000,5000);
ofVec3f thresh3D(0.2,.2,.3);

int thresh2D = 100;
float minVol = 0.;
float maxVol = 50000.;

int minPoints = 0;
int maxBlobs = 10;

ofEasyCam camera;


//--------------------------------------------------------------
void ofApp::setup(){
    kinect.setup();
    maskImage.allocate(512, 424, OF_IMAGE_GRAYSCALE);
    maskImage.setColor(ofColor(255,0,0));
    maskImage.update();
    
    float sqrResolution = tracker.getResolution();
    sqrResolution *= sqrResolution;
//    minPoints = (int)(0.001*(float)(512 * 424)/sqrResolution);
    
//    camera.setScale(200.);
    camera.setPosition(ofGetWidth()/2.0, ofGetHeight()/2.0, 1000.);
}

//--------------------------------------------------------------
void ofApp::update(){
    static bool bTrackerInit = false;
    
    bool bNewFrame = kinect.update();
    
    if ( bNewFrame && !bTrackerInit ){
        bTrackerInit = true;
        tracker.init(&kinect.getKinect(), true);
    }
    
    if ( tracker.isInited() ){
//        thresh2D = ofMap(mouseX, 0, ofGetWidth(), 0, 255);
        thresh3D.x = ofMap(mouseX, 0, ofGetWidth(), 0, 1.0);
        thresh3D.y = ofMap(mouseX, 0, ofGetWidth(), 0, 1.0);
        thresh3D.z = ofMap(mouseY, 0, ofGetHeight(), 0, 1.0);
        tracker.findBlobs(&maskImage, boxMin, boxMax, thresh3D, thresh2D, minVol, maxVol, minPoints, maxBlobs );
    }
}

//--------------------------------------------------------------
void ofApp::draw(){
    if ( ofGetKeyPressed('k'))
        kinect.draw();
    
    ofSetColor(255);
    
    if ( tracker.isInited() ){
        
        ofPushMatrix();
        camera.begin();
//        ofTranslate(ofGetWidth()/2.0, 0.);
//        ofScale(100., 100.,100.);
        ofEnableDepthTest();
//        glPointSize(1);
        // draw blobs
        for (unsigned int i=0; i < tracker.blobs.size(); i++) {
            ofSetColor(25*i,25*i,255-25*i);
            // draw blobs
            tracker.blobs[i].draw();
        }
        ofSetColor(255);
        ofDisableDepthTest();
        camera.end();
        ofPopMatrix();
        
        ofDrawBitmapString(ofToString(tracker.blobs.size()), 50, 60);
        
    }
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
