module rough_terrain(unit=10, edge=15, height=2, pad=0)
{
	//unit = 100;
	pad=pad*2*unit;
	eps = 0.01*unit;

	difference(){
		union(){
			translate([-(edge/2)*unit,-(edge/2)*unit,0])
			for(i=[0:edge-1])
				for(j=[0:edge-1])
					translate([i*unit,j*unit,0])
					cube([unit-eps, unit-eps, unit*(rands(0,height,edge*edge,7)[i*edge+j])]);
		}
		union(){
			translate([-pad/2,-pad/2,0])
			cube([pad,pad,pad]);
		}
		
	}
}

rough_terrain(unit = 0.1, edge = 20, height = 0.5, pad=3);


/*	difference(){
translate([-((edge+pad)/2)*unit,-((edge+pad)/2)*unit,0])
cube([unit*(edge+pad), unit*(edge+pad), height*unit/2]);
translate([-(edge/2)*unit,-(edge/2)*unit,0])
cube([unit*(edge), unit*(edge), height*unit]);
} */