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

namespace robogen {
/************************************************************
*	Attention la masse du prism est complètement fausse
*
*************************************************************/
	const float ParametricPrismModel::MASS_SLOT = inGrams(1);
	const float ParametricPrismModel::MASS_CONNECTION_PER_M = inGrams(1.37) * 100.;
	const float ParametricPrismModel::SLOT_WIDTH = inMm(34);
	const float ParametricPrismModel::SLOT_THICKNESS = inMm(1.5);

	ParametricPrismModel::ParametricPrismModel(dWorldID odeWorld, dSpaceID odeSpace,
			std::string id, int faceNumber):
			Model(odeWorld, odeSpace, id),
			faceNumber_(faceNumber){
	}

	ParametricPrismModel::~ParametricPrismModel() {

	}

	bool ParametricPrismModel::initModel() {
		boost::shared_ptr<SimpleBody> currentBox;
		boost::shared_ptr<SimpleBody> nextBox;
		osg::Quat boxRotation;
		//ParametricPrism is composed of N geomtries if odd else N/2 geometries
		float boxLenghtX;
		float PrismeFaceAngle = osg::DegreesToRadians(360.0/(float)faceNumber_);
		//If the Prisme is even it can be separate in "faceNumber_" isoceles triangles
		if(faceNumber_%2 == 0){
			boxLenghtX = sqrt((SLOT_WIDTH*SLOT_WIDTH) 
								/(cos(PrismeFaceAngle/2.0)*cos(PrismeFaceAngle/2.0)) 
								- SLOT_WIDTH*SLOT_WIDTH);
			//because the prism is even that it can be construct with rectangle
			currentBox = this->addBox(10*MASS_SLOT, osg::Vec3(0, 0, 0),
							boxLenghtX, SLOT_WIDTH, SLOT_WIDTH, 0);
			boxRoot_ = currentBox;
			for(int i = 1; i<faceNumber_/2; i++)
			{
				nextBox = this->addBox(10*MASS_SLOT, osg::Vec3(0, 0, 0),
							boxLenghtX, SLOT_WIDTH, SLOT_WIDTH, 0);
				boxRotation.makeRotate(i*PrismeFaceAngle, osg::Vec3(0, 0, 1));
				nextBox->setAttitude(boxRotation);
				this->fixBodies(currentBox, nextBox);
				currentBox = nextBox;
			}
		}
		//if the Prisme is odd (Je crois que ça forme aussi des isocèle)
		else{
			;
		}
		return true;
	}

	boost::shared_ptr<SimpleBody> ParametricPrismModel::getRoot() {
		return boxRoot_;
	}

	boost::shared_ptr<SimpleBody> ParametricPrismModel::getSlot(unsigned int i) {
		return boxRoot_;
	}
	osg::Vec3 ParametricPrismModel::getSlotPosition(unsigned int i) {

		osg::Vec3 slotPos;
		return slotPos;
	}

	osg::Vec3 ParametricPrismModel::getSlotAxis(unsigned int i) {
		osg::Quat quat;
		osg::Vec3 axis;
		return quat*axis;
	}

	osg::Vec3 ParametricPrismModel::getSlotOrientation(unsigned int i) {
		osg::Quat quat;
		osg::Vec3 axis;
		return quat*axis;
	}
}