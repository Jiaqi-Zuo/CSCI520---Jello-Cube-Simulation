# CSCI 520, Assignment 1: Simulating a Jello Cube

![](https://github.com/Jiaqi-Zuo/CSCI520---Jello-Cube-Simulation/blob/b0eefc1de173d42791dcd28ac2b7be84dd9e8512/animation/jello.gif)

## Core features
- Implemented acceleration for each mass point of the jello cube, took into account the forces due to the mass-spring system(consists of structural, shear and bend springs), external forces, and collision with walls.

## Additional features:
- Implemented collision detection with an inclined plane
- Rendered the jello more realistic
	-- made the jello cube transparent by setting the alpha value and enable GL_BLEND
	-- only rendered the part not penetrated the wall or inclined plane, increased the deformation to make the cube look more bouncy. 
- Made the wall tranparent when it was back to the user, so that the bounding box was more like a showcase.
