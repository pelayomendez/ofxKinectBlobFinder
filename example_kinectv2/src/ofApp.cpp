#include "ofApp.h"

ofVec3f boxMin(-8,-8,-8);
ofVec3f boxMax(8,8,8);
ofVec3f thresh3D(0.2,.2,.3);

int thresh2D = 100;
float minVol = 0.;
float maxVol = 50000.;

int minPoints = 0;
int maxBlobs = 10;

ofEasyCam camera;


//--------------------------------------------------------------
void ofApp::setup(){
    ofBackground(255);
    ofSetVerticalSync(true);
    kinect.setup();
    maskImage.allocate(512, 424, OF_IMAGE_GRAYSCALE);
    maskImage.setColor(ofColor(255,0,0));
    maskImage.update();
    
    float sqrResolution = tracker.getResolution();
    sqrResolution *= sqrResolution;
//    minPoints = (int)(0.001*(float)(512 * 424)/sqrResolution);
    
    camera.setPosition(ofGetWidth()/4.0, ofGetHeight()/2.0, 1000.);
    
    gui = new ofxUICanvas(0,0, ofGetWidth()/2.0, ofGetHeight());
    
    gui->addSlider("boundsMin.x", -8, 16., &boxMin.x);
    gui->addSlider("boundsMin.y", -8, 16., &boxMin.y);
    gui->addSlider("boundsMin.z", -8, 16., &boxMin.z);
    
    gui->addSlider("boundsMax.x", 0, 16., &boxMax.x);
    gui->addSlider("boundsMax.y", 0, 16., &boxMax.y);
    gui->addSlider("boundsMax.z", 0, 16., &boxMax.z);
    
    
    gui->addSlider("thresh3D.x", 0, 1., &thresh3D.x);
    gui->addSlider("thresh3D.y", 0, 1., &thresh3D.y);
    gui->addSlider("thresh3D.z", 0, 1., &thresh3D.z);
    
    gui->addIntSlider("thresh2D", 0, 255, &thresh2D);
    
    gui->addSlider("minVol", 0, 100, &minVol);
    gui->addSlider("maxVol", 0, 100, &maxVol);
    
    gui->addIntSlider("minPoints", 0, 500, &minPoints);
    gui->addIntSlider("maxBlobs", 0, 100, &maxBlobs);
    
    gui->loadSettings("settings.xml");
    
//    camera.disableMouseInput();
}




bool visible = false;
bool save = false;

//--------------------------------------------------------------
void ofApp::update(){
    static bool bTrackerInit = false;
    
    bool bNewFrame = kinect.update();
    
    // kinect 2 init is async, so just chill till it's ready, OK?
    if ( bNewFrame && !bTrackerInit ){
        bTrackerInit = true;
        tracker.init(&kinect.getKinect(), false);
    }
    
    if ( tracker.isInited() ){
        tracker.findBlobs(&maskImage, boxMin, boxMax, thresh3D, thresh2D, minVol, maxVol, minPoints, maxBlobs );
    }
}

//--------------------------------------------------------------
void ofApp::draw(){
    if ( ofGetKeyPressed('k'))
        kinect.draw();
    
    ofSetColor(255);
    
    if ( tracker.isInited() ){
        
        glPointSize(2.0);
        
        ofPushMatrix();
        camera.begin();
//        ofTranslate(ofGetWidth()/2.0, 0.);
        ofScale(100., 100.,100.);
        ofEnableDepthTest();
//        glPointSize(1);
        // draw blobs
        for (unsigned int i=0; i < tracker.blobs.size(); i++) {
            ofPushMatrix();
            ofColor color;
            color.setSaturation(255);
            color.setBrightness(150);
            color.setHue(ofMap(i,0, tracker.blobs.size(), 0, 255));
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
        ofSetColor(255);
        ofDisableDepthTest();
        camera.end();
        ofPopMatrix();
        
        ofDrawBitmapString(ofToString(tracker.blobs.size()), 50, 60);
        
    }
    
}

//--------------------------------------------------------------
void ofApp::keyPressed(int key){
//    return; // currently effed in 0.9.0 on github!
    
    if ( key == 'g' ){
        visible = false;
        gui->toggleVisible();
    }
    
    if ( key == 's' ){
        save = false;
        
        gui->saveSettings("settings.xml");
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
