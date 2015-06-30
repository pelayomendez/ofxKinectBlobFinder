//
//  Kinect2Utils.h
//  example_kinectv2
//
//  THESE ARE FROM THE OFXMULTIKINECTV2 LIB
//  I JUST GET TIRED OF ADDING TO ofApp.h, OK?

#pragma once

#include "ofMain.h"
#include "ofxMultiKinectV2.h"

#define STRINGIFY(x) #x

static string depthFragmentShader =
STRINGIFY(
          uniform sampler2DRect tex;
          void main()
          {
              vec4 col = texture2DRect(tex, gl_TexCoord[0].xy);
              float value = col.r;
              float low1 = 500.0;
              float high1 = 5000.0;
              float low2 = 1.0;
              float high2 = 0.0;
              float d = clamp(low2 + (value - low1) * (high2 - low2) / (high1 - low1), 0.0, 1.0);
              if (d == 1.0) {
                  d = 0.0;
              }
              gl_FragColor = vec4(vec3(d), 1.0);
          }
          );

static string irFragmentShader =
STRINGIFY(
          uniform sampler2DRect tex;
          void main()
          {
              vec4 col = texture2DRect(tex, gl_TexCoord[0].xy);
              float value = col.r / 65535.0;
              gl_FragColor = vec4(vec3(value), 1.0);
          }
          );

class Kinect2Manager {
public:
    
    void setup(){
        
        depthShader.setupShaderFromSource(GL_FRAGMENT_SHADER, depthFragmentShader);
        depthShader.linkProgram();
        
        irShader.setupShaderFromSource(GL_FRAGMENT_SHADER, irFragmentShader);
        irShader.linkProgram();
        
        kinect0.open(true, true, 0);
        
        // Note :
        // Default OpenCL device might not be optimal.
        // e.g. Intel HD Graphics will be chosen instead of GeForce.
        // To avoid it, specify OpenCL device index manually like following.
        // kinect1.open(true, true, 0, 2); // GeForce on MacBookPro Retina
        
        kinect0.start();
    }
    
    // return isFrameNew val
    bool update(){
        kinect0.update();
        if (kinect0.isFrameNew()) {
            colorTex0.loadData(kinect0.getColorPixelsRef());
            depthTex0.loadData(kinect0.getDepthPixelsRef());
            irTex0.loadData(kinect0.getIrPixelsRef());
            return true;
        }
        return false;
    }
    
    // debug draw
    void draw()
    {
        if (colorTex0.isAllocated()) {
            colorTex0.draw(0, 0, 640, 360);
        }
        if (depthTex0.isAllocated()) {
            depthShader.begin();
            depthTex0.draw(640, 0, 512, 424);
            depthShader.end();
            irShader.begin();
            irTex0.draw(1152, 0, 512, 424);
            irShader.end();
        }
        
        ofDrawBitmapStringHighlight(ofToString(ofGetFrameRate()), 10, 20);
        ofDrawBitmapStringHighlight("Device Count : " + ofToString(ofxMultiKinectV2::getDeviceCount()), 10, 40);
    }
    
    // getters
    
    ofShader & getDepthShader(){
        return depthShader;
    }
    
    ofShader & getIrShader(){
        return irShader;
    }
    
    ofxMultiKinectV2 & getKinect(){
        return kinect0;
    }
    
    ofTexture & getColorTexture(){
        return colorTex0;
    }
    
    
    ofTexture & getDepthTexture(){
        return depthTex0;
    }
    
    ofTexture & getIrTexture(){
        return irTex0;
    }
    
protected:
    
    
    ofxMultiKinectV2 kinect0;
    ofTexture colorTex0;
    ofTexture depthTex0;
    ofTexture irTex0;
    
    ofShader depthShader;
    ofShader irShader;
};