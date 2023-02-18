/*

  USC/Viterbi/Computer Science
  "Jello Cube" Assignment 1 starter code

*/

#include "jello.h"
#include "physics.h"

/* Compute Elastic Force exerted on A
* F = -k_h * (|L| - R) * (L/|L|)
*/
void computeElasticForce(struct point A, struct point B, double kE, double resLen, struct point& F)
{
    point L;
    double length;
    pDIFFERENCE(A, B, L); // let L be the vector pointing from B to A
    pNORMALIZE(L); // length = lengh of L
    double currScalar = -kE * (length - resLen);
    pMULTIPLY(L, currScalar, F);
}

/* Compute damping force exerted on A
* F = -k_d * ( (v_A - v_B) * L) / |L| * (L / |L|)
*/
void computeDampingForce(struct point A, struct point B, struct point vA, struct point vB, double kD, struct point& F)
{
    point L, vL;
    double length, projection;
    pDIFFERENCE(A, B, L); // let L be the vector pointing from B to A
    pDIFFERENCE(vA, vB, vL); // let vL be the vector pointing from vB to vA
    pNORMALIZE(L); // length = lengh of L
    DOTPRODUCTp(vL, L, projection);
    double currScalar = -kD * projection;
    pMULTIPLY(L, currScalar, F);
}

/* process each type or spring and compute the accelaration */
void computeSpringForce(struct world* jello, int i, int j, int k, struct point& F)
{
    point A = jello->p[i][j][k], vA = jello->v[i][j][k];
    point B, vB;
    double kE = jello->kElastic, dE = jello->dElastic;
    // structural springs (total : 6)
    if (i >= 1) {
        B = jello->p[i - 1][j][k]; vB = jello->v[i - 1][j][k];
        helper(A, B, vA, vB, kE, dE, F, 1);
    } if (i <= 6) {
        B = jello->p[i + 1][j][k]; vB = jello->v[i + 1][j][k];
        helper(A, B, vA, vB, kE, dE, F, 1);
    } if (j >= 1) {
        B = jello->p[i][j - 1][k]; vB = jello->v[i][j - 1][k];
        helper(A, B, vA, vB, kE, dE, F, 1);
    } if (j <= 6) {
        B = jello->p[i][j + 1][k]; vB = jello->v[i][j + 1][k];
        helper(A, B, vA, vB, kE, dE, F, 1);
    } if (k >= 1) {
        B = jello->p[i][j][k - 1]; vB = jello->v[i][j][k - 1];
        helper(A, B, vA, vB, kE, dE, F, 1);
    } if (k <= 6) {
        B = jello->p[i][j][k + 1]; vB = jello->v[i][j][k + 1];
        helper(A, B, vA, vB, kE, dE, F, 1);
    }
    // shear springs (total: 20)
    if (j >= 1 && k >= 1) {
        B = jello->p[i][j - 1][k - 1]; vB = jello->v[i][j - 1][k - 1];
        helper(A, B, vA, vB, kE, dE, F, 2);
    } if (j >= 1 && k <= 6) {
        B = jello->p[i][j - 1][k + 1]; vB = jello->v[i][j - 1][k + 1];
        helper(A, B, vA, vB, kE, dE, F, 2);
    } if (j <= 6 && k >= 1) {
        B = jello->p[i][j + 1][k - 1]; vB = jello->v[i][j + 1][k - 1];
        helper(A, B, vA, vB, kE, dE, F, 2);
    } if (j <= 6 && k <= 6) {
        B = jello->p[i][j + 1][k + 1]; vB = jello->v[i][j + 1][k + 1];
        helper(A, B, vA, vB, kE, dE, F, 2);
    } if (i >= 1 && j >= 1) {
        B = jello->p[i - 1][j - 1][k]; vB = jello->v[i - 1][j - 1][k];
        helper(A, B, vA, vB, kE, dE, F, 2);
    } if (i >= 1 && j <= 6) {
        B = jello->p[i - 1][j + 1][k]; vB = jello->v[i - 1][j + 1][k];
        helper(A, B, vA, vB, kE, dE, F, 2);
    } if (i <= 6 && j >= 1) {
        B = jello->p[i + 1][j - 1][k]; vB = jello->v[i + 1][j - 1][k];
        helper(A, B, vA, vB, kE, dE, F, 2);
    } if (i <= 6 && j <= 6) {
        B = jello->p[i + 1][j + 1][k]; vB = jello->v[i + 1][j + 1][k];
        helper(A, B, vA, vB, kE, dE, F, 2);
    } if (i >= 1 && k >= 1) {
        B = jello->p[i - 1][j][k - 1]; vB = jello->v[i - 1][j][k - 1];
        helper(A, B, vA, vB, kE, dE, F, 2);
    } if (i >= 1 && k <= 6) {
        B = jello->p[i - 1][j][k + 1]; vB = jello->v[i - 1][j][k + 1];
        helper(A, B, vA, vB, kE, dE, F, 2);
    } if (i <= 6 && k >= 1) {
        B = jello->p[i + 1][j][k - 1]; vB = jello->v[i + 1][j][k - 1];
        helper(A, B, vA, vB, kE, dE, F, 2);
    } if (i <= 6 && k <= 6) {
        B = jello->p[i + 1][j][k + 1]; vB = jello->v[i + 1][j][k + 1];
        helper(A, B, vA, vB, kE, dE, F, 2);
    }
    if (i >= 1 && j >= 1 && k >= 1) {
        B = jello->p[i - 1][j - 1][k - 1]; vB = jello->v[i - 1][j - 1][k - 1];
        helper(A, B, vA, vB, kE, dE, F, 3);
    } if (i >= 1 && j >= 1 && k <= 6) {
        B = jello->p[i - 1][j - 1][k + 1]; vB = jello->v[i - 1][j - 1][k + 1];
        helper(A, B, vA, vB, kE, dE, F, 3);
    } if (i >= 1 && j <= 6 && k >= 1) {
        B = jello->p[i - 1][j + 1][k - 1]; vB = jello->v[i - 1][j + 1][k - 1];
        helper(A, B, vA, vB, kE, dE, F, 3);
    } if (i >= 1 && j <= 6 && k <= 6) {
        B = jello->p[i - 1][j + 1][k + 1]; vB = jello->v[i - 1][j + 1][k + 1];
        helper(A, B, vA, vB, kE, dE, F, 3);
    } if (i <= 6 && j >= 1 && k >= 1) {
        B = jello->p[i + 1][j - 1][k - 1]; vB = jello->v[i + 1][j - 1][k - 1];
        helper(A, B, vA, vB, kE, dE, F, 3);
    } if (i <= 6 && j >= 1 && k <= 6) {
        B = jello->p[i + 1][j - 1][k + 1]; vB = jello->v[i + 1][j - 1][k + 1];
        helper(A, B, vA, vB, kE, dE, F, 3);
    } if (i <= 6 && j <= 6 && k >= 1) {
        B = jello->p[i + 1][j + 1][k - 1]; vB = jello->v[i + 1][j + 1][k - 1];
        helper(A, B, vA, vB, kE, dE, F, 3);
    } if (i <= 6 && j <= 6 && k <= 6) {
        B = jello->p[i + 1][j + 1][k + 1]; vB = jello->v[i + 1][j + 1][k + 1];
        helper(A, B, vA, vB, kE, dE, F, 3);
    }
    // bend springs (total: 6)
    if (i >= 2) {
        B = jello->p[i - 2][j][k]; vB = jello->v[i - 2][j][k];
        helper(A, B, vA, vB, kE, dE, F, 4);
    } if (i <= 5) {
        B = jello->p[i + 2][j][k]; vB = jello->v[i + 2][j][k];
        helper(A, B, vA, vB, kE, dE, F, 4);
    } if (j >= 2) {
        B = jello->p[i][j - 2][k]; vB = jello->v[i][j - 2][k];
        helper(A, B, vA, vB, kE, dE, F, 4);
    } if (j <= 5) {
        B = jello->p[i][j + 2][k]; vB = jello->v[i][j + 2][k];
        helper(A, B, vA, vB, kE, dE, F, 4);
    } if (k >= 2) {
        B = jello->p[i][j][k - 2]; vB = jello->v[i][j][k - 2];
        helper(A, B, vA, vB, kE, dE, F, 4);
    } if (k <= 5) {
        B = jello->p[i][j][k + 2]; vB = jello->v[i][j][k + 2];
        helper(A, B, vA, vB, kE, dE, F, 4);
    }
}

/*helper function to computer spring force */
void helper(struct point A, struct point B, struct point vA, struct point vB, double kE, double dE, struct point& F, int TYPE) {
    point currF;
    double resLEN = 0.0;
    if (TYPE == 1) resLEN = strucRL;
    else if (TYPE == 2) resLEN = shearRL;
    else if (TYPE == 3) resLEN = shearRL3D;
    else if (TYPE == 4) resLEN = bendRL;
    computeElasticForce(A, B, kE, resLEN, currF);
    pSUM(F, currF, F);
    computeDampingForce(A, B, vA, vB, dE, currF);
    pSUM(F, currF, F);
}

/* Detect collision and compute the collision force */
void computeCollisionForce(struct world* jello, int i, int j, int k, struct point& F)
{
    point A = jello->p[i][j][k], vA = jello->v[i][j][k], B, vB;
    double kE = jello->kCollision, dE = jello->dCollision;
    pMAKE(0.0, 0.0, 0.0, vB);
    double penalty;
    // the movement of cube is limited to a bounding box [-2, -2,-2] to [2, 2, 2]
    // use penalty method to compute the force
    // the magnitude of the penalty is proportional to the amount of penetration
    if (A.x < -2 || A.x > 2) {
        penalty = fabs(A.x) - 2;
        if (A.x < -2) {
            pMAKE(-2.0, A.y, A.z, B);
        }if (A.x > 2) {
            pMAKE(2.0, A.y, A.z, B);
        }
        helper(A, B, vA, vB, kE * penalty, dE * penalty, F, 0);
    }
    if (A.y < -2 || A.y > 2) {
        penalty = fabs(A.y) - 2;
        if (A.y < -2) {
            pMAKE(A.x, -2.0, A.z, B);
        }
        if (A.y > 2) {
            pMAKE(A.x, 2.0, A.z, B);
        }
        helper(A, B, vA, vB, kE * penalty, dE * penalty, F, 0);
    }
    if (A.z < -2 || A.z > 2) {
        penalty = fabs(A.z) - 2;
        if (A.z < -2) {
            pMAKE(A.x, A.y, -2.0, B);
        }
        if (A.z > 2) {
            pMAKE(A.x, A.y, 2.0, B);
        }
        helper(A, B, vA, vB, kE * penalty, dE * penalty, F, 0);
    }
}

/* helper function to check if current position is inside the force field */
void isPointInside(int& pos, int currReso) {
    if (pos >= currReso - 1) pos = currReso - 2;
    else if (pos < 0) pos = 0;
}

/* compute the force field exerted on the jello cube */
void computeForceField(struct world* jello, int i, int j, int k, double cellLen, struct point& F)
{
    int currReso = jello->resolution;
    point currP = jello->p[i][j][k];
    // convert coord from world space to force field
    int fX = int((currP.x + 2) / cellLen), fY = int((currP.y + 2) / cellLen), fZ = int((currP.z + 2) / cellLen);
    // check if the current point in force field is still inside the box
    isPointInside(fX, currReso);
    isPointInside(fY, currReso);
    isPointInside(fZ, currReso);
    int ffX = fX, ffY = fY, ffZ = fZ;
    // convert coord from force field to world space 
    // process the forces at eight neighbors for curr point
    // implement trilinear interpolation and convex combination to convert from force field back to world space
    fX = fX * cellLen - 2;
    fY = fY * cellLen - 2;
    fZ = fZ * cellLen - 2;
    double wX = (currP.x - fX) / cellLen, wY = (currP.y - fY) / cellLen, wZ = (currP.z - fZ) / cellLen;
    // handle eight corners of the current position
    // (x,y,z), (x,y,z+1), (x,y+1,z), (x,y+1,z+1), (x+1,y,z), (x+1,y,z+1), (x+1,y+1,z), (x+1,y+1,z+1)
    for (int a = 0; a <= 1; a++) {
        for (int b = 0; b <= 1; b++) {
            for (int c = 0; c <= 1; c++) {
                point tempForce = jello->forceField[(ffX + a) * currReso * currReso + (ffY + b) * currReso + (ffZ + c)];
                // apply convex combination
                double convexX = a == 0 ? (1 - wX) : wX;
                double convexY = b == 0 ? (1 - wY) : wY;
                double convexZ = c == 0 ? (1 - wZ) : wZ;
                pMULTIPLY(tempForce, convexX * convexY * convexZ, tempForce);
                pSUM(F, tempForce, F);
            }
        }
    }
}

void computeInclinedPlaneAcceleration(struct world* jello, int i, int j, int k, struct point& F) {
    point currP = jello->p[i][j][k];
    double pA = jello->a, pB = jello->b, pC = jello->c, pD = jello->d;
    // i(nclined plane equation F(x,y,z) = ax+by+cz+d
    // F(x,y,z) > 0 on one side of the plane and F(x,y,z) < 0 on the other
    double planeSide = currP.x * pA + currP.y * pB + currP.z * pC + pD;
    if (planeSide <= 0) {
        point planeN, touchPt, vB;
        double length;
        pMAKE(pA, pB, pC, planeN);
        pMAKE(0.0, 0.0, 0.0, vB);
        pNORMALIZE(planeN);
        double distance = planeSide / length; // distace from point to the plane
        pMULTIPLY(planeN, distance, planeN); // the normal vec of point on the plane
        distance = abs(distance);
        pDIFFERENCE(currP, planeN, touchPt);
        point A = currP, B = touchPt, vA = jello->v[i][j][k];
        double kE = distance * jello->kCollision, dE = distance * jello->dCollision;
        helper(A, B, vA, vB, kE, dE, F, 0);
    }
}

/* Computes acceleration to every control point of the jello cube,
   which is in state given by 'jello'.
   Returns result in array 'a'. */
void computeAcceleration(struct world* jello, struct point a[8][8][8])
{
    /* for you to implement ... */
    int i, j, k;

    for (i = 0; i <= 7; i++)
        for (j = 0; j <= 7; j++)
            for (k = 0; k <= 7; k++)
            {
                pMAKE(0.0, 0.0, 0.0, a[i][j][k]);
                computeSpringForce(jello, i, j, k, a[i][j][k]);
                computeCollisionForce(jello, i, j, k, a[i][j][k]);
                if (jello->incPlanePresent) computeInclinedPlaneAcceleration(jello, i, j, k, a[i][j][k]);
                if (jello->resolution >= 2 && jello->resolution <= 30) {
                    double cellLen = 4.0 / (jello->resolution - 1);
                    computeForceField(jello, i, j, k, cellLen, a[i][j][k]);
                }
                pMULTIPLY(a[i][j][k], (1.0 / jello->mass), a[i][j][k]);

            }
}


/* performs one step of Euler Integration */
/* as a result, updates the jello structure */
void Euler(struct world* jello)
{
    int i, j, k;
    point a[8][8][8];

    computeAcceleration(jello, a);

    for (i = 0; i <= 7; i++)
        for (j = 0; j <= 7; j++)
            for (k = 0; k <= 7; k++)
            {
                jello->p[i][j][k].x += jello->dt * jello->v[i][j][k].x;
                jello->p[i][j][k].y += jello->dt * jello->v[i][j][k].y;
                jello->p[i][j][k].z += jello->dt * jello->v[i][j][k].z;
                jello->v[i][j][k].x += jello->dt * a[i][j][k].x;
                jello->v[i][j][k].y += jello->dt * a[i][j][k].y;
                jello->v[i][j][k].z += jello->dt * a[i][j][k].z;

            }
}

/* performs one step of RK4 Integration */
/* as a result, updates the jello structure */
void RK4(struct world* jello)
{
    point F1p[8][8][8], F1v[8][8][8],
        F2p[8][8][8], F2v[8][8][8],
        F3p[8][8][8], F3v[8][8][8],
        F4p[8][8][8], F4v[8][8][8];

    point a[8][8][8];


    struct world buffer;

    int i, j, k;

    buffer = *jello; // make a copy of jello

    computeAcceleration(jello, a);

    for (i = 0; i <= 7; i++)
        for (j = 0; j <= 7; j++)
            for (k = 0; k <= 7; k++)
            {
                pMULTIPLY(jello->v[i][j][k], jello->dt, F1p[i][j][k]);
                pMULTIPLY(a[i][j][k], jello->dt, F1v[i][j][k]);
                pMULTIPLY(F1p[i][j][k], 0.5, buffer.p[i][j][k]);
                pMULTIPLY(F1v[i][j][k], 0.5, buffer.v[i][j][k]);
                pSUM(jello->p[i][j][k], buffer.p[i][j][k], buffer.p[i][j][k]);
                pSUM(jello->v[i][j][k], buffer.v[i][j][k], buffer.v[i][j][k]);
            }

    computeAcceleration(&buffer, a);

    for (i = 0; i <= 7; i++)
        for (j = 0; j <= 7; j++)
            for (k = 0; k <= 7; k++)
            {
                // F2p = dt * buffer.v;
                pMULTIPLY(buffer.v[i][j][k], jello->dt, F2p[i][j][k]);
                // F2v = dt * a(buffer.p,buffer.v);     
                pMULTIPLY(a[i][j][k], jello->dt, F2v[i][j][k]);
                pMULTIPLY(F2p[i][j][k], 0.5, buffer.p[i][j][k]);
                pMULTIPLY(F2v[i][j][k], 0.5, buffer.v[i][j][k]);
                pSUM(jello->p[i][j][k], buffer.p[i][j][k], buffer.p[i][j][k]);
                pSUM(jello->v[i][j][k], buffer.v[i][j][k], buffer.v[i][j][k]);
            }

    computeAcceleration(&buffer, a);

    for (i = 0; i <= 7; i++)
        for (j = 0; j <= 7; j++)
            for (k = 0; k <= 7; k++)
            {
                // F3p = dt * buffer.v;
                pMULTIPLY(buffer.v[i][j][k], jello->dt, F3p[i][j][k]);
                // F3v = dt * a(buffer.p,buffer.v);     
                pMULTIPLY(a[i][j][k], jello->dt, F3v[i][j][k]);
                pMULTIPLY(F3p[i][j][k], 1.0, buffer.p[i][j][k]);
                pMULTIPLY(F3v[i][j][k], 1.0, buffer.v[i][j][k]);
                pSUM(jello->p[i][j][k], buffer.p[i][j][k], buffer.p[i][j][k]);
                pSUM(jello->v[i][j][k], buffer.v[i][j][k], buffer.v[i][j][k]);
            }

    computeAcceleration(&buffer, a);


    for (i = 0; i <= 7; i++)
        for (j = 0; j <= 7; j++)
            for (k = 0; k <= 7; k++)
            {
                // F3p = dt * buffer.v;
                pMULTIPLY(buffer.v[i][j][k], jello->dt, F4p[i][j][k]);
                // F3v = dt * a(buffer.p,buffer.v);     
                pMULTIPLY(a[i][j][k], jello->dt, F4v[i][j][k]);

                pMULTIPLY(F2p[i][j][k], 2, buffer.p[i][j][k]);
                pMULTIPLY(F3p[i][j][k], 2, buffer.v[i][j][k]);
                pSUM(buffer.p[i][j][k], buffer.v[i][j][k], buffer.p[i][j][k]);
                pSUM(buffer.p[i][j][k], F1p[i][j][k], buffer.p[i][j][k]);
                pSUM(buffer.p[i][j][k], F4p[i][j][k], buffer.p[i][j][k]);
                pMULTIPLY(buffer.p[i][j][k], 1.0 / 6, buffer.p[i][j][k]);
                pSUM(buffer.p[i][j][k], jello->p[i][j][k], jello->p[i][j][k]);

                pMULTIPLY(F2v[i][j][k], 2, buffer.p[i][j][k]);
                pMULTIPLY(F3v[i][j][k], 2, buffer.v[i][j][k]);
                pSUM(buffer.p[i][j][k], buffer.v[i][j][k], buffer.p[i][j][k]);
                pSUM(buffer.p[i][j][k], F1v[i][j][k], buffer.p[i][j][k]);
                pSUM(buffer.p[i][j][k], F4v[i][j][k], buffer.p[i][j][k]);
                pMULTIPLY(buffer.p[i][j][k], 1.0 / 6, buffer.p[i][j][k]);
                pSUM(buffer.p[i][j][k], jello->v[i][j][k], jello->v[i][j][k]);
            }

    return;
}
