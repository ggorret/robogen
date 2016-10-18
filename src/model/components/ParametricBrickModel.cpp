/*
 * @(#) ParametricBrickModel.cpp   1.0   Feb 14, 2013
 *
 * Andrea Maesani (andrea.maesani@epfl.ch)
 * Joshua Auerbach (joshua.auerbach@epfl.ch)
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
#include "model/components/ParametricBrickModel.h"
#include "model/CompositeBody.h"
namespace robogen {

const float ParametricBrickModel::MASS_SLOT = inGrams(1);
const float ParametricBrickModel::MASS_CONNECTION_PER_M = inGrams(1.37) * 100.;
const float ParametricBrickModel::SLOT_WIDTH = inMm(34);
const float ParametricBrickModel::SLOT_THICKNESS = inMm(1.5);
const float ParametricBrickModel::CONNECTION_PART_WIDTH = inMm(20);
const float ParametricBrickModel::CONNECTION_PART_THICKNESS = inMm(10);
const float ParametricBrickModel::CYLINDER_RADIUS = inMm(5);
const float ParametricBrickModel::FIXED_BAR_LENGTH = inMm(6);

ParametricBrickModel::ParametricBrickModel(dWorldID odeWorld, dSpaceID odeSpace,
		std::string id, float connectionPartLength, float angleA, float angleB):
		Model(odeWorld, odeSpace, id),
		connectionPartLength_(connectionPartLength + CYLINDER_RADIUS),
		angleA_(angleA),
		angleB_(angleB) {

}

ParametricBrickModel::~ParametricBrickModel() {

}

bool ParametricBrickModel::initModel() {

	// ParametricBrick is composed of 5 geometries,
	// now created directly with calls to this->add___

	brickRoot_ = this->addBox(inGrams(250), osg::Vec3(0, 0, 0),
				inMm(50), inMm(50), inMm(50), B_SLOT_A_ID);

	// do something similar with the slot piece
	// first place it at origin
	brickTail_ = this->addBox(inGrams(250), osg::Vec3(0, 0, 0),
				inMm(50), inMm(50), inMm(50), B_SLOT_B_ID);


	brickTail_->setPosition(osg::Vec3(inMm(10), 0, 0));

	// Fix slot B
	this->fixBodies(brickRoot_, brickTail_);


	// get composite body
	boost::shared_ptr<CompositeBody> composite = brickRoot_->getParent();


	std::cout << "Mass of composite body: " << composite->compositeMass_.mass * 1000 <<  std::endl;

	return true;

}

boost::shared_ptr<SimpleBody> ParametricBrickModel::getRoot() {
	return brickRoot_;
}

boost::shared_ptr<SimpleBody> ParametricBrickModel::getSlot(unsigned int i) {
	if (i == SLOT_A) {
		return brickRoot_;
	} else {
		return brickTail_;
	}
}

osg::Vec3 ParametricBrickModel::getSlotPosition(unsigned int i) {

	if (i > 2) {
		std::cout << "[ParametricBrickModel] Invalid slot: " << i << std::endl;
		assert(i <= 2);
	}
	osg::Vec3 slotPos;
	if (i == SLOT_A) {

		osg::Vec3 curPos = this->brickRoot_->getPosition();
		osg::Vec3 slotAxis = this->getSlotAxis(i);
		slotPos = curPos + slotAxis * (SLOT_THICKNESS / 2);

	} else {

		osg::Vec3 curPos = this->brickTail_->getPosition();
		osg::Vec3 slotAxis = this->getSlotAxis(i);
		slotPos = curPos + slotAxis * (SLOT_THICKNESS / 2);

	}
	return slotPos;

}

osg::Vec3 ParametricBrickModel::getSlotAxis(unsigned int i) {

	if (i > 2) {
		std::cout << "[ParametricBrickModel] Invalid slot: " << i << std::endl;
		assert(i <= 2);
	}

	osg::Quat quat;
	osg::Vec3 axis;

	if (i == SLOT_A) {

		quat = this->brickRoot_->getAttitude();
		axis.set(-1, 0, 0);

	} else if (i == SLOT_B) {

		quat = this->brickTail_->getAttitude();
		axis.set(1, 0, 0);

	}

	return quat * axis;

}

osg::Vec3 ParametricBrickModel::getSlotOrientation(unsigned int i) {

	if (i > 2) {
		std::cout << "[ParametricBrickModel] Invalid slot: " << i << std::endl;
		assert(i <= 2);
	}

	osg::Quat quat;
	osg::Vec3 axis;

	if (i == SLOT_A) {

		quat = this->brickRoot_->getAttitude();
		axis.set(0, 1, 0);

	} else if (i == SLOT_B) {

		quat = this->brickTail_->getAttitude();
		axis.set(0, 1, 0);

	}

	return quat * axis;

}

}
