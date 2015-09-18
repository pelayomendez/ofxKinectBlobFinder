/****************************************************************************
*
* ofxKinectBlobFinder.cpp
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

#include "ofxKinectBlobFinder.h"

#define FLAG_OFF_THRESHOLD -5
#define FLAG_BACKGROUND -4
#define FLAG_IDLE -3
#define FLAG_QUEUED -2
#define FLAG_PROCESSED -1

#define __DEFAULT_RESOLUTION BF_LOW_RES

#define K_RANGE_MIN 0.5f
#define K_RANGE_MAX 5.00f

static double fx_d = 1.0 / 5.9421434211923247e+02;
static double fy_d = 1.0 / 5.9104053696870778e+02;
static float cx_d = 3.3930780975300314e+02;
static float cy_d = 2.4273913761751615e+02;

// ***************************************************************************
//                                CONSTRUCTORS
// ***************************************************************************
ofxKinectBlobFinder::ofxKinectBlobFinder() {
	ofLog(OF_LOG_VERBOSE, "ofxKinectBlobFinder: creating");
	setResolution(BF_MEDIUM_RES);
	setRotation(ofVec3f::zero());
	setTranslation(ofVec3f::zero());
	nBlobs = 0;
}

// ***************************************************************************
//                                INIT
// ***************************************************************************
void ofxKinectBlobFinder::init(int width, int height, bool standarized) {
	ofLog(OF_LOG_VERBOSE, "ofxKinectBlobFinder: init");
	bStandarized = standarized;
	kWidth = width;
	kHeight = height;
	kNPix = kWidth*kHeight;
	bFinderInited = setResolution(__DEFAULT_RESOLUTION);
}

// ***************************************************************************
//                                FIND BLOBS

//  maskPixels : for background subtraction
// neighbour search range
// http://pointclouds.org/documentation/tutorials/cluster_extraction.php#cluster-extraction

// ***************************************************************************

bool ofxKinectBlobFinder::findBlobs(ofMesh* mesh,
	const ofVec3f boundingBoxMin, const ofVec3f boundingBoxMax,
	const ofVec3f thresh3D, const int thresh2D,
	const float minVol, const float maxVol,
	const int minPoints, const unsigned int maxBlobs)
{
	if (!bFinderInited) {
		ofLog(OF_LOG_WARNING, "ofxKinectBlobFinder: findBlobs - must init finder first");
		return false;
	}
	if (mesh->getVertices().empty()) {
		ofLog(OF_LOG_WARNING, "ofxKinectBlobFinder: findBlobs - mesh is empty");
		return false;
	}
    if (!createCloud(mesh, boundingBoxMin, boundingBoxMax) ) {
        ofLog(OF_LOG_WARNING, "ofxKinectBlobFinder: findBlobs - could not create pointcloud");
        return false;
    }

    vector<ofxKinectBlob>    tempBlobs;
    int queueIndex = 0;
    int lastQueued = 0;
    int pixIndex = 0;

    int pixelsToProcess = nPix;
    //int queue[nPix];
	vector<int> queue;
	queue.resize(nPix);

    blobs.clear();
    int numBlobs = 0;

    float minX, minY, minZ, maxX, maxY, maxZ;

    ofVec3f *minXPos, *minYPos, *minZPos, *maxXPos, *maxYPos, *maxZPos;

    while ( (pixIndex < nPix) && (pixelsToProcess > 0) &&
            (tempBlobs.size() < maxBlobs) ) {
        queueIndex = 0;
        lastQueued = 0;
        minX = minY = minZ = 100;
        maxX = maxY = maxZ = -100;
        //search for next unprocessed pixel
        while ( (pixIndex < nPix) &&
                (p3DCloud[pixIndex].flag != FLAG_IDLE) ) pixIndex++;

        if (pixIndex == nPix) break;
        else queue[0] = pixIndex;

        int queueIndexPix = queue[queueIndex];
        // while not end of queue
        while ((lastQueued < nPix) && (queueIndex <= lastQueued) &&
               (queueIndexPix >= 0) &&
               (queueIndexPix < nPix) && (pixelsToProcess > 0)) {

            pixelsToProcess--;

            p2D3& p3DCloudPoint = p3DCloud[queueIndexPix];
            // mark pixel as being processed
            p3DCloudPoint.flag = FLAG_PROCESSED;

            ofVec3f * posPtr = &p3DCloudPoint.pos;
            float pointX = (* posPtr).x;
            float pointY = (* posPtr).y;
            float pointZ = (* posPtr).z;

            // update blob properties here (max xyz, height, width,depth, centroid, volume, ...)
            //if ((pointX < 10) && (pointY < 10) && (pointZ < 10) &&
           //     (pointX > -10) && (pointY > -10) && (pointZ > -10)) {

                if (pointX < minX) { minX= pointX; minXPos = posPtr;}
                else if (pointX > maxX) { maxX= pointX; maxXPos = posPtr; }
                if (pointY < minY) { minY= pointY; minYPos = posPtr; }
                else if (pointY > maxY) { maxY= pointY; maxYPos = posPtr; }
                if (pointZ < minZ) { minZ= pointZ; minZPos = posPtr; }
                else if (pointZ > maxZ) { maxZ= pointZ; maxZPos = posPtr; }

                int i = queueIndexPix % width;
                int j = queueIndexPix / width;

                for (int u = i-thresh2D; u <= i+thresh2D; u++) {
                    for (int v =j-thresh2D; v <= j+thresh2D; v++) {
                        int neighbour = u+v*width;

                        if ( (neighbour >= 0) && (neighbour < nPix) && (p3DCloud[neighbour].flag == FLAG_IDLE) ) {
                            ofVec3f nPoint = p3DCloud[neighbour].pos;
                            if ( (abs(pointX-nPoint.x) <= thresh3D.x) &&
                                 (abs(pointY-nPoint.y) <= thresh3D.y) &&
                                 (abs(pointZ-nPoint.z) <= thresh3D.z) ) {
                                lastQueued++;
                                if (lastQueued < nPix) {
                                    queue[lastQueued] = neighbour;
                                    p3DCloud[neighbour].flag = FLAG_QUEUED;
                                }
                            }
                        }
                   }
                }
            //}
            queueIndex++;
            queueIndexPix = queue[queueIndex];

        }


        if (lastQueued > minPoints) {
            ofPoint blobDim = ofPoint( abs(maxX-minX), abs(maxY-minY), abs(maxZ-minZ));
            float blobVol = blobDim.x*blobDim.y*blobDim.z;
            // minimum number of pixels to be considered as a blob
            if ((blobVol >= minVol) && (blobVol <= maxVol)) {

                ofVec3f newMassCenter = ofVec3f(0.0f, 0.0f, 0.0f);

                ofxKinectBlob newBlob;
                newBlob.mesh.setMode(OF_PRIMITIVE_TRIANGLE_STRIP);
                for (int i = 0; i < lastQueued; i++) {
                    p3DCloud[queue[i]].flag = numBlobs;
                    //newBlob.mesh.addTexCoord(ofVec2f(queue[i] % width, queue[i] / width));
                    newBlob.mesh.addVertex(p3DCloud[queue[i]].pos);

                    newMassCenter += p3DCloud[queue[i]].pos;//*pointWeight;

                }
                newBlob.boundingBoxMin = ofPoint(minX,minY,minZ);
                newBlob.boundingBoxMax = ofPoint(maxX,maxY,maxZ);
                newBlob.minX = *minXPos;    newBlob.maxX = *maxXPos;
                newBlob.minY = *minYPos;    newBlob.maxY = *maxYPos;
                newBlob.minZ = *minZPos;    newBlob.maxZ = *maxZPos;
                newBlob.dimensions = blobDim;
                newBlob.volume = blobVol;
                newBlob.massCenter = newMassCenter/lastQueued;
                newBlob.centroid = newBlob.boundingBoxMin.getMiddle(newBlob.boundingBoxMax);
                tempBlobs.push_back(newBlob);
                numBlobs++;
            }
        }
    }

    lastBlobs.clear();
    lastBlobs = blobs;
    blobs = tempBlobs;
    nBlobs = numBlobs;
    tempBlobs.clear();

    return true;
}

// ************************************************************
//                CREATE POINT CLOUD AND SCALE IMAGES
// ************************************************************

bool ofxKinectBlobFinder::createCloud( ofMesh* mesh,
                                       const ofVec3f boundingBoxMin, const ofVec3f boundingBoxMax) {
	p2D3 point;
	ofVec3f pos;
	p3DCloud.clear();

	for (auto& v : mesh->getVertices()) {

		if ((v.z == 0) || (v.z > K_RANGE_MAX) || (v.z < K_RANGE_MIN)) {
			point.flag = FLAG_BACKGROUND;
			point.pos = ofVec3f::zero();
		}
		else {
			pos = v * 1000;
			if (bStandarized) pos = ofVec3f(pos.x, pos.z, -pos.y);

			// the order of rotation matters!!!
			pos *= scale;
			pos.rotate(rotation.x, 0, 0);
			pos.rotate(0, rotation.y, 0);
			pos.rotate(0, 0, rotation.z);
			pos += translation;

			if ((pos.x < boundingBoxMin.x) || (pos.x > boundingBoxMax.x) ||
				(pos.y < boundingBoxMin.y) || (pos.y > boundingBoxMax.y) ||
				(pos.z < boundingBoxMin.z) || (pos.z > boundingBoxMax.z)
				) {
				point.flag = FLAG_OFF_THRESHOLD;
			}
			else point.flag = FLAG_IDLE;
			point.pos = pos;
		}
		p3DCloud.push_back(point);
	}

    return true;
}

// ************************************************************
//                SET POINT CLOUD ROTATION
// ************************************************************
void ofxKinectBlobFinder::setRotation(const ofVec3f newRotation) {
    rotation = newRotation;
}
// ************************************************************
//                GET POINT CLOUD ROTATION
// ************************************************************
ofVec3f ofxKinectBlobFinder::getRotation() {
    return rotation;
};
// ************************************************************
//                SET POINT CLOUD TRANSLATION
// ************************************************************
void ofxKinectBlobFinder::setTranslation(const ofVec3f newTranslation) {
    translation = newTranslation;
}
// ************************************************************
//                GET POINT CLOUD TRANSLATION
// ************************************************************
ofVec3f ofxKinectBlobFinder::getTranslation() {
    return translation;
};
// ************************************************************
//                SET POINT CLOUD SCALE
// ************************************************************
void ofxKinectBlobFinder::setScale(const ofVec3f newScale) {
    scale = newScale;
}
// ************************************************************
//                GET POINT CLOUD SCALE
// ************************************************************
ofVec3f ofxKinectBlobFinder::getScale() {
    return scale;
};
// ************************************************************
//                SET ANALYSIS/POINTCLOUD RESOLUTON
// ************************************************************
bool ofxKinectBlobFinder::setResolution(ofxKinectBlobFinderResolution newResolution) {
    if ((p3DCloud.empty()) || (resolution != newResolution)) {
        resolution = newResolution;
        width = kWidth / resolution;
        height = kHeight / resolution;
        nPix = width*height;
		if (p3DCloud.size()) p3DCloud.clear();
		p3DCloud.resize(nPix);
        if (p3DCloud.empty()) {
            ofLog(OF_LOG_WARNING, "ofxKinectBlobFinder: setResolution - error allocating memory for pointcloud ");
            return false;
        }
    }
    return true;
}
// ************************************************************
//                GET ANALYSIS/POINTCLOUD RESOLUTON
// ************************************************************
enum ofxKinectBlobFinderResolution ofxKinectBlobFinder::getResolution() {
    return resolution;
}

// ************************************************************
//
// ************************************************************
bool ofxKinectBlobFinder::isInited() {
    return bFinderInited;
}
