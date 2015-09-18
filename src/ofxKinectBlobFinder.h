/****************************************************************************
*
* ofxKinectBlobFinder.h
*
* Purpose: 3D blob finder to be used with ofxKinect and the kinect 3D sensor
*
* Author: David Sanz Kirbis, January 2012
*
* Comments:
*
* Part of the cvCinema project: www.cvcinema.com
*
* The technique used is Euclidean Cluster Extraction, as explained in:
* http://www.pointclouds.org/documentation/tutorials/cluster_extraction.php
*
******************************************************************************/
#pragma once

#include "ofMain.h"
#include "ofxKinectForWindows2.h"
#include "ofxKinectBlob.h"


enum ofxKinectBlobFinderResolution { BF_HIGH_RES = 1, BF_MEDIUM_RES = 2, BF_LOW_RES = 4 };


struct p2D3 {
  int flag;
  ofVec3f pos;
} ;

class ofxKinectBlobFinder {
    public:
      ofxKinectBlobFinder();
      void init(ofxKFW2::Device *newKinect, bool standarized);
      bool isInited();
      void setRotation(const ofVec3f newRotation);
      void setTranslation(const ofVec3f newTranslation);
      void setScale(const ofVec3f newScale);
      bool setResolution(enum ofxKinectBlobFinderResolution newResolution);
      ofVec3f getRotation();
      ofVec3f getTranslation();
      ofVec3f getScale();
      enum ofxKinectBlobFinderResolution getResolution();
      bool findBlobs(   ofMesh* mesh,
                        const ofVec3f cropBoxMin, const ofVec3f cropBoxMax,
                        const ofVec3f thresh3D, const int thresh2D,
                        const float minVol, const float maxVol,
                        const int minPoints, const unsigned int maxBlobs  );
      vector<ofxKinectBlob>    lastBlobs;
      vector<ofxKinectBlob>    blobs;
      int nBlobs;


    protected:

		ofxKFW2::Device * kinectPtr;
		ICoordinateMapper * mapper;
		enum ofxKinectBlobFinderResolution resolution;

        ofVec3f rotation;
        ofVec3f translation;
        ofVec3f scale;
        int width;
        int height;

        int kWidth;
        int kHeight;
        int kNPix;
        int nPix;

        vector<p2D3> p3DCloud;
        bool bFinderInited;
        bool bStandarized;

        bool createCloud(ofMesh* mesh, const ofVec3f cropBoxMin, const ofVec3f cropBoxMax);
};
