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

	const float ParametricPrismModel::MASS_SLOT = inGrams(1);
	const float ParametricPrismModel::MASS_CONNECTION_PER_M = inGrams(1.37) * 100.;
	const float ParametricPrismModel::SLOT_WIDTH = inMm(34);
	const float ParametricPrismModel::SLOT_THICKNESS = inMm(1.5);

	//Je ne sais pas ce que ça fait
	ParametricPrismModel::ParametricPrismModel(dWorldID odeWorld, dSpaceID odeSpace,
			std::string id, int numberFace):
			Model(odeWorld, odeSpace, id),
			numberFace_(numberFace){
	}

	ParametricPrismModel::~ParametricPrismModel() {

	}

	// à compléter
	bool ParametricPrismModel::initModel() {
		return true;
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