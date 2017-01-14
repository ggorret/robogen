// TODO: define correctly the 3-faced Prism and 4-faced Prism as core

/***************************Prism launcher****************************
*	prism(n, Iscore)
*	n = faceNumber
*	IsCore = true to add the chipset support and false if not
**********************************************************************/
	
	prism(5 ,FALSE);

/**************************Constant Parameter*************************/
		SIDE = 41;
		PRISM_HEIGHT = 35.5;
		SLOT_SIDE = 35;    // (mm) must be the dimension of slot, not face (allow for fit)
		SLOT_THICKNESS = 1.5;
		HOLE_GAP = 25;    // (mm) distance between holes for attaching plates
		DISTANCE_BASIS_HOLE_CORNER = 11; // (mm) distance between corner and a hole in the basis
		SHIP_SIDE = 30;
		BATTERY_HEIGHT = 14;
		BATTERY_WIDTH = 19;
		BATTERY_LENGTH = 34;

		SUPPORT_SHIP_HEIGHT = BATTERY_HEIGHT + 4;
		BASE_THICKNESS = 1;
		SUPPORT_THICKNESS = 2;
		
		TRUE = 1;
		FALSE = 0;
		
/**************************Main Module*************************/
module prism(n, isCore){
	radius = SIDE/(2*sin(360/(2*n)));
	radius_inner = radius - (3.2/(cos(360/(2*n)))); //(SIDE-3.2*2)/(2*sin(360/(2*n)));
	//parameter for the slot hole	
	centerBase = 12;
	baseLength = 12;
	bottomDiag = 12;
	
	union(){
		if(isCore){
			shipSupport();
		}
		cornerRenforcement(n, baseLength, bottomDiag);
		difference(){
			rotate([0, 0, 360/(2*n)]){//first face always normal to x axis
				difference(){		
					cylinder(r = radius, h = PRISM_HEIGHT, $fn = n); //shape
					translate([0, 0, BASE_THICKNESS]) 
						cylinder(r = radius_inner, h = PRISM_HEIGHT, $fn = n); //shape
				}	
			}

			for (i = [0:n-1]){ //place at each face
				rotate([0, 0, i*360/n]) 
				translate(radius*cos(360/(2*n))*[1,0,0]) 
					slotNegative(n,centerBase, baseLength, bottomDiag);
			}
			basisHole(n);
		}
	}
}

/**************************Secondary Module*************************/

//is a losange shape
module cornerRenforcement(n, baseLength, bottomDiag){
	prismAngle = 360/n;
	radius = SIDE/(2*sin(360/(2*n)));
	radius_inner = radius - (3.2/(cos(360/(2*n))));

	//corner of the losange
	innerSide = 2*radius_inner*sin(prismAngle/2);
	renforcementSide = (innerSide - baseLength)/2 - bottomDiag*cos(45);	
	cornerAngle = 180-prismAngle;
	smallDiag = 2*renforcementSide*cos(cornerAngle/2);
	bigDiag = 2*renforcementSide*sin(cornerAngle/2);

	c1X = smallDiag/2;
	c1Y = 0;

	c2X = 0;
	c2Y = bigDiag/2;
	c4X = 0;
	c4Y = -c2Y;
		
	c3X = -c1X;
	c3Y = 0;
		
	for(i = [0:n-1]){
		rotate([0,0,i*prismAngle])
		translate((radius_inner-smallDiag/2)*[cos(prismAngle/2),sin(prismAngle/2),0]) 
		rotate([0,0,prismAngle/2])
		linear_extrude(height = PRISM_HEIGHT)
			polygon(points=[[c1X,c1Y], 
							 [c2X,c2Y],
							 [c3X,c3Y],
							 [c4X,c4Y]], convexity=10);
	}
}

	////////////////////////////////////////////////////////////
	// Negative shape of slot hole to be applied on all sides. 
	// The zero is in the middle of the slot, outer face
module slotNegative(n,centerBase, baseLength, bottomDiag){
	
	union(){
		slotConnection(n);
		slotHole(centerBase, baseLength, bottomDiag);
	}
}

module shipSupport() {
	
	shipDiag = sqrt(SHIP_SIDE*SHIP_SIDE*2);
	radius = shipDiag/2;
	innerRadius = shipDiag/2-SUPPORT_THICKNESS;

	translate([0,0,BASE_THICKNESS])
	rotate([0, 0, 45])
		union(){
			shipExtrusion(innerRadius);
			batteryExtrusion();
			difference(){
				cylinder(r = radius, h = SUPPORT_SHIP_HEIGHT, $fn = 4);
			
				cylinder(r = innerRadius, h = SUPPORT_SHIP_HEIGHT+1, $fn = 4);
				supportHole();
			}
		}
}

module batteryExtrusion(){
	rotate([0,0,-45])
		difference(){
				cube([BATTERY_LENGTH/3,BATTERY_WIDTH,SUPPORT_THICKNESS], true);
			
				cube([BATTERY_LENGTH/3+1,BATTERY_WIDTH-2*SUPPORT_THICKNESS,SUPPORT_THICKNESS+1], true);
			}
}

module shipExtrusion(innerRadius){
 	
	l=2*SUPPORT_THICKNESS+1;
		
	trianglePoints = [
  		[  0,  0,  0 ],  //0
  		[ -l,  0,  0 ],  //1
  		[ 0,  l,  0 ],  //2
  		[  0,  0,  l ]];  //3
  
	triangleFaces = [
  		[0,1,2],  	  // bottom
  		[0,3,1],  
  		[0,2,3],  
  		[1,3,2]];
 
	for(i = [0:3]){
		rotate([0,0,i*90])
		translate([0,-innerRadius,SUPPORT_SHIP_HEIGHT-SUPPORT_THICKNESS])
		rotate([0,180,45])  
		polyhedron( trianglePoints, triangleFaces );
	}
}


module supportHole(){
	extrudeHeight = SHIP_SIDE+1;
	c1X = 0;
	c1Y = SHIP_SIDE/2-SUPPORT_THICKNESS;

	c2X = c1X;
	c2Y = -c1Y;
		
	c3X = SUPPORT_SHIP_HEIGHT-SUPPORT_THICKNESS;
	c3Y = 0;
		
	for(i = [0:1]){
		rotate([0,0,i*90])
		rotate([0,-90,45])
		translate([0,0,-extrudeHeight/2])
		linear_extrude(height = extrudeHeight)
			polygon(points=[	 [c1X,c1Y], 
							 [c2X,c2Y],
							 [c3X,c3Y]], convexity=10);
	}
}

module slotConnection(n){ 

	rotate([0, 0, -90]) 
	translate([-SLOT_SIDE/2, -SLOT_THICKNESS, 0.5]) 
	union(){
		difference(){ // plate
			cube ([SLOT_SIDE, SLOT_THICKNESS+0.5, SLOT_SIDE+1]);  // main plate
			union(){ 	 // chamfer the edges
				translate([0,0,-1]) 
				rotate([0,0,45]) 
					cube([sqrt(2)*SLOT_THICKNESS,sqrt(2)*SLOT_THICKNESS, SLOT_SIDE+2]);
				translate([SLOT_SIDE,0,-1]) 
				rotate([0,0,45]) 
					cube([sqrt(2)*SLOT_THICKNESS,sqrt(2)*SLOT_THICKNESS, SLOT_SIDE+2]);
				translate([-1,0,0]) 
				rotate([90,45,90]) 
					cube([sqrt(2)*SLOT_THICKNESS,sqrt(2)*SLOT_THICKNESS, SLOT_SIDE+2]);
			}
		}
		translate([(SLOT_SIDE-HOLE_GAP)/2, 0, (SLOT_SIDE-HOLE_GAP)/2]) 
		rotate([90, 0, 0]) 
			cylinder(h = 6, r = 1.5/2, $fn = 15);
		translate([(SLOT_SIDE-HOLE_GAP)/2+HOLE_GAP, 0, (SLOT_SIDE-HOLE_GAP)/2]) 
		rotate([90, 0, 0]) 
			cylinder(h = 6, r = 1.5/2, $fn = 15);
		translate([(SLOT_SIDE-HOLE_GAP)/2, 0, (SLOT_SIDE-HOLE_GAP)/2+HOLE_GAP]) 
		rotate([90, 0, 0]) 
			cylinder(h = 6, r = 1.5/2, $fn = 15);
		translate([(SLOT_SIDE-HOLE_GAP)/2+HOLE_GAP, 0, (SLOT_SIDE-HOLE_GAP)/2+HOLE_GAP]) 
		rotate([90, 0, 0]) 
			cylinder(h = 6, r = 1.5/2, $fn = 15);
		
	}
}

module slotHole(centerBase, baseLength, bottomDiag){
//With the rotation done: -X=Z and Y=Y 		
		extrudeHeight = 10;

		//corner of the polygon
		c1X = centerBase;
		c1Y = -(baseLength/2);
		c2X = c1X;
		c2Y = -c1Y;
		
		c3X = c2X - bottomDiag*sin(45);
		c3Y = c2Y + bottomDiag*cos(45);
		
		c5X = centerBase-PRISM_HEIGHT + 2*SUPPORT_THICKNESS;//c4X-c3Y*tan(45);
		c5Y = 0;

		c4X = c5X + c3Y;//tan(45) = 1;
		c4Y = c3Y;

		c7X = c1X - bottomDiag*sin(45);
		c7Y = c1Y - bottomDiag*cos(45);
		
		c6X = c4X;
		c6Y = c7Y;
		
		translate([-extrudeHeight/2,0,centerBase+2])
		rotate([0,90,0])
		linear_extrude(height = extrudeHeight)
			polygon(points=[	 [c1X,c1Y], 
							 [c2X,c2Y],
							 [c3X,c3Y],
							 [c4X,c4Y],
							 [c5X,c5Y],
							 [c6X,c6Y],
							 [c7X,c7Y]], convexity=10);
}

//do the hole in the basis
module basisHole(n){
	prismAngle = 360/n;
	distanceFaceCenter = 0.5*SIDE/(tan(360/(2*n)));
	for(i = [0:n-1]){
		rotate([0,0,i*prismAngle])
		translate(-DISTANCE_BASIS_HOLE_CORNER*[cos(prismAngle/2),sin(prismAngle/2),0]) 
		translate([distanceFaceCenter,SIDE/2,-3]) 
			cylinder(h = 6, r = 1.5/2, $fn = 15);
	}
}
