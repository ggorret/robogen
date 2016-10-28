/*
 * @(#) ParametricPrismModel.cpp   1.0   Oct 5, 2016
 *
 * Andrea Maesani (andrea.maesani@epfl.ch)
 * Joshua Auerbach (joshua.auerbach@epfl.ch)
 * Gaël Gorret (gael.gorret@epfl.ch)
 *
 * The ROBOGEN Framework
 * Copyright © 2012-2015 Andrea Maesani, Joshua Auerbach
 *
 * Laboratory of Intelligent Systems, EPFL
 *
 * This file is part of the ROBOGEN Framework.
 *
 * The ROBOGEN Framework is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License (GPL)
 * as published by the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 * @(#) $Id$
 */

#include "model/components/ParametricPrismModel.h"
#include "model/ConvexBody.h"

namespace robogen {
/************************************************************
*	Attention la masse du prism est complètement fausse
*
*************************************************************/
	const float ParametricPrismModel::MASS_PRISM = inGrams(10); //poids complètement arbitraire
	const float ParametricPrismModel::MASS_CORE = MASS_PRISM + inGrams(34.3);
	const float ParametricPrismModel::WIDTHY = inMm(41);
	const float ParametricPrismModel::HEIGHTZ = inMm(35.5);
	const float ParametricPrismModel::SLOT_THICKNESS = inMm(1.5);

	ParametricPrismModel::ParametricPrismModel(dWorldID odeWorld, dSpaceID odeSpace,
			std::string id, int faceNumber):
			Model(odeWorld, odeSpace, id),
			faceNumber_(faceNumber){
				float PrismeFaceAngle = osg::DegreesToRadians(360.0/(float)faceNumber_);

				ParametricPrismModel::topFaceSlotId_ = faceNumber_;
				ParametricPrismModel::bottomFaceSlotId_ = faceNumber_ + 1;
				ParametricPrismModel::distanceFaceCenter_ = 0.5*WIDTHY/(tan(PrismeFaceAngle/2.0));
	}

	ParametricPrismModel::~ParametricPrismModel() {

	}

	
	bool ParametricPrismModel::initModel() {
		boost::shared_ptr<SimpleBody> currentBox;
		boost::shared_ptr<SimpleBody> nextBox;
		osg::Quat boxRotation;
		osg::Quat boxRotation_total;
		osg::Vec3 boxTranslation;
		osg::Vec3 SlotOrientation;
		//ParametricPrism is composed of N geomtries if odd else N/2 geometries
		float boxLenghtX;
		float PrismeFaceAngle = osg::DegreesToRadians(360.0/(float)faceNumber_);
		//If the Prisme is regular it can be separate in "faceNumber_" isoceles triangles
		if(faceNumber_%2 == 0){
			boxLenghtX = WIDTHY/(tan(PrismeFaceAngle/2.0));
			//because the prism is even that it can be construct with rectangle
			currentBox = this->addBox(MASS_PRISM, osg::Vec3(0, 0, 0),
							boxLenghtX, WIDTHY, HEIGHTZ, 0);
			boxRoot_ = currentBox;

			for(int i = 1; i<faceNumber_/2; i++)
			{
				nextBox = this->addBox(MASS_PRISM, osg::Vec3(0, 0, 0),
							boxLenghtX, WIDTHY, HEIGHTZ, i);
				boxRotation.makeRotate(i*PrismeFaceAngle, osg::Vec3(0, 0, 1));
				nextBox->setAttitude(boxRotation);
				this->fixBodies(currentBox, nextBox);
				currentBox = nextBox;
			}
		}
		//if the Prisme is odd we can't use rectangle
		else{
			boxLenghtX = 0.5*WIDTHY/(tan(PrismeFaceAngle/2.0));
			// in order to avoid a little hole at the center of the prism
			boxLenghtX = boxLenghtX + 0.1 * boxLenghtX;
			
			//the prism is odd so it construct with "faceNumber_" of boxes
			currentBox = this->addBox(MASS_PRISM, osg::Vec3(0, 0, 0),
							boxLenghtX, WIDTHY, HEIGHTZ, 0);
			boxRoot_ = currentBox;
			boxRotation.makeRotate(PrismeFaceAngle, osg::Vec3(0, 0, 1));
			boxRotation_total = boxRotation;
			osg::Vec3 relativeTranslation = osg::Vec3(0,0,0);

			/*
			*First the boxe is rotated around the axis (0,0,1) that positioned at (boxLenghtX/2, -WIDTHY/2)
			*Then the boxe is placed to the right place with a succession of translation following
			*the edge of the prism
			*/ 
	
			for(int i = 1; i<faceNumber_; i++)
			{
				nextBox = this->addBox(MASS_PRISM, osg::Vec3(0, 0, 0),
							boxLenghtX, WIDTHY, HEIGHTZ, i);
				SlotOrientation = getSlotOrientation(i-1);
				//The nextBox will add its translation to all the previous one.
				relativeTranslation = relativeTranslation
										+ osg::Vec3(WIDTHY*SlotOrientation.x(),
													WIDTHY*SlotOrientation.y(),
													WIDTHY*SlotOrientation.z());
				//relativeTranslation + vector of the fulcrum - offset of the rotation
				boxTranslation =    relativeTranslation
									+ osg::Vec3(boxLenghtX/2, -WIDTHY/2, 0) 
									- boxRotation_total*osg::Vec3(boxLenghtX/2, -WIDTHY/2, 0);

				nextBox->setAttitude(boxRotation_total);
				nextBox->setPosition(boxTranslation);
				this->fixBodies(currentBox, nextBox);
				currentBox = nextBox;
				boxRotation_total = boxRotation_total*boxRotation;
			}
		}
		ParametricPrismModel::distanceFaceCenter_ = boxLenghtX;
		return true;
	}
	
/*
	bool ParametricPrismModel::initModel(){
		boost::shared_ptr<SimpleBody> prism;
		dReal planeArray[4*(faceNumber_ + 2)]; // add 2 for the top and bottom face

		//create plane array
		for(int i = 0; i<faceNumber_; i++){
			osg::Vec3 normal;
			normal = getSlotAxis(i);
			planeArray[4*i] = normal.x();
			planeArray[4*i+1] = normal.y();
			planeArray[4*i+2] = normal.z();
			planeArray[4*i+3] = distanceFaceCenter_;
		}
		//for the top and bottom face
			planeArray[4*faceNumber_] = 0.0f;
			planeArray[4*faceNumber_+1] = 0.0f;
			planeArray[4*faceNumber_+2] = 1.0f;
			planeArray[4*faceNumber_+3] = HEIGHTZ/2.0f;
			planeArray[4*(faceNumber_+1)] = 0.0f;
			planeArray[4*(faceNumber_+1)+1] = 0.0f;
			planeArray[4*(faceNumber_+1)+2] = -1.0f;
			planeArray[4*(faceNumber_+1)+3] = HEIGHTZ/2.0f;
		

		//ConvexBody(prism, MASS_PRISM, 2*faceNumber_);
		return true;
	}
*/
	boost::shared_ptr<SimpleBody> ParametricPrismModel::getRoot() {
		return boxRoot_;
	}

	boost::shared_ptr<SimpleBody> ParametricPrismModel::getSlot(unsigned int i) {
		return boxRoot_; // With the Root we can find all the other slot. Faut que je vérifie comment ça marche
	}

	//get the position of the faces
	osg::Vec3 ParametricPrismModel::getSlotPosition(unsigned int i) {
		if (i >= faceNumber_) {
			std::cout << "[ParametricPrismModel] Invalid slot: " << i << std::endl;
			assert(i < faceNumber_);
		}
		//osg::Quat boxRotation;
		//float PrismeFaceAngle = osg::DegreesToRadians(360.0/(float)faceNumber_);

		osg::Vec3 curPos = this->getRootPosition();
		osg::Vec3 slotAxis = this->getSlotAxis(i) *
			(distanceFaceCenter_ / 2 - SLOT_THICKNESS);

		//boxRotation.makeRotate(i*PrismeFaceAngle, osg::Vec3(0, 0, 1));
		curPos = curPos + slotAxis;

		return curPos;
	}

// return the normal vector of the face asked
	osg::Vec3 ParametricPrismModel::getSlotAxis(unsigned int i) {
		if (i >= faceNumber_) {
			std::cout << "[ParametricPrismModel] Invalid slot: " << i << std::endl;
			assert(i < faceNumber_);
		}

		osg::Quat quat = this->getRootAttitude();
		osg::Vec3 axis;
		osg::Quat rotation;
			
			if (i < faceNumber_){
				axis.set(1,0,0); //normal vector of the root face.

				float PrismeFaceAngle = osg::DegreesToRadians(360.0/(float)faceNumber_);
				rotation.makeRotate(i*PrismeFaceAngle, osg::Vec3(0, 0, 1));
			}		
		#ifndef ENFORCE_PLANAR
			else if (i == topFaceSlotId_) {

				// Top face
				axis.set(0, 0, 1);
				rotation.makeRotate(0, osg::Vec3(0, 0, 1));

			} else if (i == bottomFaceSlotId_) {

				// Bottom face
				axis.set(0, 0, -1);
				rotation.makeRotate(0, osg::Vec3(0, 0, 1));
			}
		#endif
			return quat * rotation * axis;
	}

// return the orientation vector of the face asked
	osg::Vec3 ParametricPrismModel::getSlotOrientation(unsigned int i) {
		if (i >= faceNumber_) {
			std::cout << "[ParametricPrismModel] Invalid slot: " << i << std::endl;
			assert(i < faceNumber_);
		}

		osg::Quat quat = this->getRootAttitude();
		osg::Vec3 tangent;
		osg::Quat rotation;
			
			if (i < faceNumber_){
				tangent.set(0,1,0); //tangent vector of the root face.

				float PrismeFaceAngle = osg::DegreesToRadians(360.0/(float)faceNumber_);
				rotation.makeRotate(i*PrismeFaceAngle, osg::Vec3(0, 0, 1));
			}		
		#ifndef ENFORCE_PLANAR
			else if (i == topFaceSlotId_) {

				// Top face
				tangent.set(0, 0, 1);
				rotation.makeRotate(0, osg::Vec3(1, 0, 0));

			} else if (i == bottomFaceSlotId_) {

				// Bottom face
				tangent.set(0, 0, -1);
				rotation.makeRotate(0, osg::Vec3(1, 0, 0));
			}
		#endif
			return quat * rotation * tangent;
	}
}