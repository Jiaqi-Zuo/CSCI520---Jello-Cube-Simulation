/*

  USC/Viterbi/Computer Science
  "Jello Cube" Assignment 1 starter code

*/

#ifndef _PHYSICS_H_
#define _PHYSICS_H_

const double strucRL = 1.0 / 7.0, shearRL = sqrt(2.0) / 7.0, shearRL3D = sqrt(3.0) / 7.0, bendRL = 2.0 / 7.0;

void computeSpringForce(struct world* jello, int i, int j, int k, struct point& F);
void helper(struct point A, struct point B, struct point vA, struct point vB, double kE, double dE, struct point& F,int TYPE);
void computeElasticForce(struct point A, struct point B, double k, double restLen, struct point& F);
void computeDampingForce(struct point A, struct point B, struct point vA, struct point vB, double k, struct point& F);
void computeCollisionForce(struct world* jello, int i, int j, int k, struct point& F);
void isPointInside(int& pos, int currReso);
void computeForceField(struct world* jello, int i, int j, int k, double cellLen, struct point& F);

void computeAcceleration(struct world * jello, struct point a[8][8][8]);

// perform one step of Euler and Runge-Kutta-4th-order integrators
// updates the jello structure accordingly
void Euler(struct world * jello);
void RK4(struct world * jello);

#endif


