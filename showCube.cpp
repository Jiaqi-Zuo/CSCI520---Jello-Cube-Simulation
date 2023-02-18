/*

  USC/Viterbi/Computer Science
  "Jello Cube" Assignment 1 starter code

*/

#include "jello.h"
#include "showCube.h"

#include <windows.h>


int pointMap(int side, int i, int j)
{
  int r;

  switch (side)
  {
  case 1: //[i][j][0] bottom face
    r = 64 * i + 8 * j;
    break;
  case 6: //[i][j][7] top face
    r = 64 * i + 8 * j + 7;
    break;
  case 2: //[i][0][j] front face
    r = 64 * i + j;
    break;
  case 5: //[i][7][j] back face
    r = 64 * i + 56 + j;
    break;
  case 3: //[0][i][j] left face
    r = 8 * i + j;
    break;
  case 4: //[7][i][j] right face
    r = 448 + 8 * i + j;
    break;
  }

  return r;
}

/* helper function to render the cube inside the bounding box*/
void checkPtInside(point& currP) {
    if (currP.x < -2.0) currP.x = -2.0;
    if (currP.x > 2.0) currP.x = 2.0;
    if (currP.y < -2.0) currP.y = -2.0;
    if (currP.y > 2.0) currP.y = 2.0;
    if (currP.z < -2.0) currP.z = -2.0;
    if (currP.z > 2.0) currP.z = 2.0;
}
/* helper function to render the cube not penetrate the inclined plane */
void checkPtInsidePlane(point& currP, double pA, double pB, double pC, double pD) {
    double planeSide = currP.x * pA + currP.y * pB + currP.z * pC + pD;
    if (planeSide <= 0) {
        point planeN, touchPt;
        double length;
        pMAKE(pA, pB, pC, planeN);
        pNORMALIZE(planeN);
        double distance = planeSide / length;
        pMULTIPLY(planeN, distance, planeN);
        pDIFFERENCE(currP, planeN, touchPt);
        pCPY(touchPt, currP);
    }
}

void showCube(struct world * jello)
{
  int i,j,k,ip,jp,kp;
  point r1,r2,r3; // aux variables
  
  /* normals buffer and counter for Gourad shading*/
  struct point normal[8][8];
  int counter[8][8];

  int face;
  double faceFactor, length;

  if (fabs(jello->p[0][0][0].x) > 10)
  {
    printf ("Your cube somehow escaped way out of the box.\n");
    exit(0);
  }

  
  #define NODE(face,i,j) (*((struct point * )(jello->p) + pointMap((face),(i),(j))))

  
  #define PROCESS_NEIGHBOUR(di,dj,dk) \
    ip=i+(di);\
    jp=j+(dj);\
    kp=k+(dk);\
    if\
    (!( (ip>7) || (ip<0) ||\
      (jp>7) || (jp<0) ||\
    (kp>7) || (kp<0) ) && ((i==0) || (i==7) || (j==0) || (j==7) || (k==0) || (k==7))\
       && ((ip==0) || (ip==7) || (jp==0) || (jp==7) || (kp==0) || (kp==7))) \
    {\
      point p1 = jello->p[i][j][k];\
      point p2 = jello->p[ip][jp][kp];\
      if\
      (jello->incPlanePresent)\
      {\
        double pA = jello->a, pB = jello->b, pC = jello->c, pD = jello->d;\
        checkPtInsidePlane(p1, pA, pB, pC, pD);\
        checkPtInsidePlane(p2, pA, pB, pC, pD);\
      }\
      checkPtInside(p1);\
      checkPtInside(p2);\
      glVertex3f(p1.x,p1.y,p1.z);\
      glVertex3f(p2.x,p2.y,p2.z);\
    }\

 
  if (viewingMode==0) // render wireframe
  {
    glLineWidth(1);
    glPointSize(5);
    glDisable(GL_LIGHTING);
    //glDisable(GL_BLEND);
    for (i=0; i<=7; i++)
      for (j=0; j<=7; j++)
        for (k=0; k<=7; k++)
        {
          if (i*j*k*(7-i)*(7-j)*(7-k) != 0) // not surface point
            continue;

          glEnable(GL_POINT_SMOOTH);
          glEnable(GL_BLEND);
          glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
          glBegin(GL_POINTS); // draw point
            glColor4f(.2,0.2,0.2,1.0);  
            point currP = jello->p[i][j][k];
            if (jello->incPlanePresent) {
                double pA = jello->a, pB = jello->b, pC = jello->c, pD = jello->d;
                checkPtInsidePlane(currP, pA, pB, pC, pD);
            }
            checkPtInside(currP);
            //glVertex3f(jello->p[i][j][k].x,jello->p[i][j][k].y,jello->p[i][j][k].z);     
            glVertex3f(currP.x, currP.y, currP.z);
          glEnd();

          //
          //if ((i!=7) || (j!=7) || (k!=7))
          //  continue;
          glEnable(GL_LINE_SMOOTH);
          glBegin(GL_LINES);      
          // structural
          if (structural == 1)
          {
            glColor4f(0,0,0.8,1);
            PROCESS_NEIGHBOUR(1,0,0);
            PROCESS_NEIGHBOUR(0,1,0);
            PROCESS_NEIGHBOUR(0,0,1);
            PROCESS_NEIGHBOUR(-1,0,0);
            PROCESS_NEIGHBOUR(0,-1,0);
            PROCESS_NEIGHBOUR(0,0,-1);
          }
          
          // shear
          if (shear == 1)
          {
            glColor4f(0,0.6,0,1);
            PROCESS_NEIGHBOUR(1,1,0);
            PROCESS_NEIGHBOUR(-1,1,0);
            PROCESS_NEIGHBOUR(-1,-1,0);
            PROCESS_NEIGHBOUR(1,-1,0);
            PROCESS_NEIGHBOUR(0,1,1);
            PROCESS_NEIGHBOUR(0,-1,1);
            PROCESS_NEIGHBOUR(0,-1,-1);
            PROCESS_NEIGHBOUR(0,1,-1);
            PROCESS_NEIGHBOUR(1,0,1);
            PROCESS_NEIGHBOUR(-1,0,1);
            PROCESS_NEIGHBOUR(-1,0,-1);
            PROCESS_NEIGHBOUR(1,0,-1);
            
            PROCESS_NEIGHBOUR(1,1,1)
            PROCESS_NEIGHBOUR(-1,1,1)
            PROCESS_NEIGHBOUR(-1,-1,1)
            PROCESS_NEIGHBOUR(1,-1,1)
            PROCESS_NEIGHBOUR(1,1,-1)
            PROCESS_NEIGHBOUR(-1,1,-1)
            PROCESS_NEIGHBOUR(-1,-1,-1)
            PROCESS_NEIGHBOUR(1,-1,-1)
          }
          
          // bend
          if (bend == 1)
          {
            glColor4f(0.5,0,0,1);
            PROCESS_NEIGHBOUR(2,0,0);
            PROCESS_NEIGHBOUR(0,2,0);
            PROCESS_NEIGHBOUR(0,0,2);
            PROCESS_NEIGHBOUR(-2,0,0);
            PROCESS_NEIGHBOUR(0,-2,0);
            PROCESS_NEIGHBOUR(0,0,-2);
          }           
          glEnd();
        }
    glDisable(GL_BLEND);
    glEnable(GL_LIGHTING);
  }
  
  else
  {
    glPolygonMode(GL_FRONT, GL_FILL); 
    
    for (face=1; face <= 6; face++) 
      // face == face of a cube
      // 1 = bottom, 2 = front, 3 = left, 4 = right, 5 = far, 6 = top
    {
      
      if ((face==1) || (face==3) || (face==5))
        faceFactor=-1; // flip orientation
      else
        faceFactor=1;
      

      for (i=0; i <= 7; i++) // reset buffers
        for (j=0; j <= 7; j++)
        {
          normal[i][j].x=0;normal[i][j].y=0;normal[i][j].z=0;
          counter[i][j]=0;
        }

      /* process triangles, accumulate normals for Gourad shading */
  
      for (i=0; i <= 6; i++)
        for (j=0; j <= 6; j++) // process block (i,j)
        {
          pDIFFERENCE(NODE(face,i+1,j),NODE(face,i,j),r1); // first triangle
          pDIFFERENCE(NODE(face,i,j+1),NODE(face,i,j),r2);
          CROSSPRODUCTp(r1,r2,r3); pMULTIPLY(r3,faceFactor,r3);
          pNORMALIZE(r3);
          pSUM(normal[i+1][j],r3,normal[i+1][j]);
          counter[i+1][j]++;
          pSUM(normal[i][j+1],r3,normal[i][j+1]);
          counter[i][j+1]++;
          pSUM(normal[i][j],r3,normal[i][j]);
          counter[i][j]++;

          pDIFFERENCE(NODE(face,i,j+1),NODE(face,i+1,j+1),r1); // second triangle
          pDIFFERENCE(NODE(face,i+1,j),NODE(face,i+1,j+1),r2);
          CROSSPRODUCTp(r1,r2,r3); pMULTIPLY(r3,faceFactor,r3);
          pNORMALIZE(r3);
          pSUM(normal[i+1][j],r3,normal[i+1][j]);
          counter[i+1][j]++;
          pSUM(normal[i][j+1],r3,normal[i][j+1]);
          counter[i][j+1]++;
          pSUM(normal[i+1][j+1],r3,normal[i+1][j+1]);
          counter[i+1][j+1]++;
        }

        /* the actual rendering */
        for (j=1; j<=7; j++) 
        {

          if (faceFactor  > 0)
            glFrontFace(GL_CCW); // the usual definition of front face
          else
            glFrontFace(GL_CW); // flip definition of orientation
          
          glBegin(GL_TRIANGLE_STRIP);
          for (i=0; i<=7; i++)
          {
            point currP = NODE(face, i, j);
            if (jello->incPlanePresent) {
                checkPtInsidePlane(currP, jello->a, jello->b, jello->c, jello->d);
            }
            checkPtInside(currP);
            glNormal3f(normal[i][j].x / counter[i][j],normal[i][j].y / counter[i][j],
              normal[i][j].z / counter[i][j]);
            //glVertex3f(NODE(face,i,j).x, NODE(face,i,j).y, NODE(face,i,j).z);
            glVertex3f(currP.x, currP.y, currP.z);

            currP = NODE(face, i, j - 1);
            if (jello->incPlanePresent) {
                checkPtInsidePlane(currP, jello->a, jello->b, jello->c, jello->d);
            }
            checkPtInside(currP);
            glNormal3f(normal[i][j-1].x / counter[i][j-1],normal[i][j-1].y/ counter[i][j-1],
              normal[i][j-1].z / counter[i][j-1]);
            /*glVertex3f(NODE(face,i,j-1).x, NODE(face,i,j-1).y, NODE(face,i,j-1).z);*/
            glVertex3f(currP.x, currP.y, currP.z);
          }
          glEnd();
        }
        
    }  
  } // end for loop over faces
  glFrontFace(GL_CCW);
}

void showBoundingBox()
{
  int i,j;

  //glBegin(GL_LINES);
  //// front face
  //for(i=-2; i<=2; i++)
  //{
  //  glVertex3f(i,-2,-2);
  //  glVertex3f(i,-2,2);
  //}
  //for(j=-2; j<=2; j++)
  //{
  //  glVertex3f(-2,-2,j);
  //  glVertex3f(2,-2,j);
  //}
  //// back face
  //for(i=-2; i<=2; i++)
  //{
  //  glVertex3f(i,2,-2);
  //  glVertex3f(i,2,2);
  //}
  //for(j=-2; j<=2; j++)
  //{
  //  glVertex3f(-2,2,j);
  //  glVertex3f(2,2,j);
  //}
  //// left face
  //for(i=-2; i<=2; i++)
  //{
  //  glVertex3f(-2,i,-2);
  //  glVertex3f(-2,i,2);
  //}
  //for(j=-2; j<=2; j++)
  //{
  //  glVertex3f(-2,-2,j);
  //  glVertex3f(-2,2,j);
  //}
  //// right face
  //for(i=-2; i<=2; i++)
  //{
  //  glVertex3f(2,i,-2);
  //  glVertex3f(2,i,2);
  //}
  //for(j=-2; j<=2; j++)
  //{
  //  glVertex3f(2,-2,j);
  //  glVertex3f(2,2,j);
  //}
  //
  //glEnd();
  
  point A, B, C, D, E, F, G, H;
  /*
        E_____________F
        /|           /|
       / |          / |
      H__|__________G |
      |  |          | |
      |  |          | |
      | A|__________|_|B
      | /           |/
      D/____________C
  */

  pMAKE(-2.0, -2.0, -2.0, A); pMAKE(-2.0, 2.0, -2.0, B); pMAKE(2.0, 2.0, -2.0, C); pMAKE(2.0, -2.0, -2.0, D);
  pMAKE(-2.0, -2.0, 2.0, E); pMAKE(-2.0, 2.0, 2.0, F); pMAKE(2.0, 2.0, 2.0, G); pMAKE(2.0, -2.0, 2.0, H);
  //glDisable(GL_CULL_FACE);
  
  glBegin(GL_QUADS);
  glEnable(GL_CULL_FACE);
  glCullFace(GL_BACK);
  // floor
  glColor4f(0.6, 0.4, 0.4,1);
  glVertex3f(A.x, A.y, A.z); glVertex3f(D.x, D.y, D.z); glVertex3f(C.x, C.y, C.z); glVertex3f(B.x, B.y, B.z);
  // ceiling
  glColor4f(0.6, 0.4, 0.4,1);
  glVertex3f(E.x, E.y, E.z); glVertex3f(F.x, F.y, F.z); glVertex3f(G.x, G.y, G.z); glVertex3f(H.x, H.y, H.z);
  // left
  glColor4f(0.6, 0.4, 0.6,1);
  glVertex3f(A.x, A.y, A.z); glVertex3f(E.x, E.y, E.z); glVertex3f(H.x, H.y, H.z); glVertex3f(D.x, D.y, D.z);
  // right
  glColor4f(0.8, 0.6, 0.8,1);
  glVertex3f(B.x, B.y, B.z); glVertex3f(C.x, C.y, C.z); glVertex3f(G.x, G.y, G.z); glVertex3f(F.x, F.y, F.z);
  // mid
  glColor4f(1.0, 0.8, 0.8,1);
  glVertex3f(A.x, A.y, A.z); glVertex3f(B.x, B.y, B.z); glVertex3f(F.x, F.y, F.z); glVertex3f(E.x, E.y, E.z); 
  glVertex3f(D.x, D.y, D.z); glVertex3f(H.x, H.y, H.z); glVertex3f(G.x, G.y, G.z); glVertex3f(C.x, C.y, C.z);
  glEnd();
  
  glEnable(GL_LINE_SMOOTH);
  glEnable(GL_BLEND);
  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);  
  glBegin(GL_LINES);
  
  glColor4f(0.1, 0.1, 0.1, 1.0);
  glVertex3f(A.x, A.y, A.z); glVertex3f(B.x, B.y, B.z);
  glVertex3f(A.x, A.y, A.z); glVertex3f(D.x, D.y, D.z);
  glVertex3f(A.x, A.y, A.z); glVertex3f(E.x, E.y, E.z);
  glVertex3f(E.x, E.y, E.z); glVertex3f(H.x, H.y, H.z);
  glVertex3f(E.x, E.y, E.z); glVertex3f(F.x, F.y, F.z);
  glVertex3f(H.x, H.y, H.z); glVertex3f(D.x, D.y, D.z);
  glVertex3f(H.x, H.y, H.z); glVertex3f(G.x, G.y, G.z);
  glVertex3f(D.x, D.y, D.z); glVertex3f(C.x, C.y, C.z);
  glVertex3f(G.x, G.y, G.z); glVertex3f(F.x, F.y, F.z);
  glVertex3f(C.x, C.y, C.z); glVertex3f(B.x, B.y, B.z);
  glVertex3f(G.x, G.y, G.z); glVertex3f(C.x, C.y, C.z);
  glVertex3f(F.x, F.y, F.z); glVertex3f(B.x, B.y, B.z);
  glEnd();
  glDisable(GL_BLEND);
  return;
}

/* finding polygon of plane intersection */
void checkIntersect(point rayOri, point rayDir, double a, double b, double c, double d, std::vector<point>& intersections) {
    pDIFFERENCE(rayDir, rayOri, rayDir);
    double rayAngle = a * rayDir.x + b * rayDir.y + c * rayDir.z;
    // ray angle = 0 means ray is perpedicular to plane
    if (rayAngle != 0.0) {
        double T;
        tCOMPUTE(rayOri, rayAngle, a, b, c, d);
        // the intersection point lies on this segment when 0 <= T <= 1
        if (T >= 0.0 && T <= 1.0) {
            point intersectPt;
            tINTERSECT(rayOri, rayDir, T, intersectPt);
            intersections.push_back(intersectPt);
        }
    }
}

void showInclinedPlane(struct world* jello) {
    double pA = jello->a, pB = jello->b, pC = jello->c, pD = jello->d;
    point A, B, C, D, E, F, G, H;
    /*
          E_____________F
          /|           /|
         / |          / |
        H__|__________G |       
        |  |          | |  
        |  |          | |
        | A|__________|_|B
        | /           |/
        D/____________C     
    */

    pMAKE(-2.0, -2.0, -2.0, A); pMAKE(-2.0, 2.0, -2.0, B); pMAKE(2.0, 2.0, -2.0, C); pMAKE(2.0, -2.0, -2.0, D);
    pMAKE(-2.0, -2.0, 2.0, E); pMAKE(-2.0, 2.0, 2.0, F); pMAKE(2.0, 2.0, 2.0, G); pMAKE(2.0, -2.0, 2.0, H);
    double rayAngle, T;
    std::vector<point> intersections;
    // check along x-axis
    checkIntersect(A, D, pA, pB, pC, pD, intersections);
    checkIntersect(E, H, pA, pB, pC, pD, intersections);
    checkIntersect(B, C, pA, pB, pC, pD, intersections);
    checkIntersect(F, G, pA, pB, pC, pD, intersections);
    // check along y-axis
    checkIntersect(A, B, pA, pB, pC, pD, intersections);
    checkIntersect(E, F, pA, pB, pC, pD, intersections);
    checkIntersect(D, C, pA, pB, pC, pD, intersections);
    checkIntersect(H, G, pA, pB, pC, pD, intersections);
    // check along z-axis
    checkIntersect(A, E, pA, pB, pC, pD, intersections);
    checkIntersect(B, F, pA, pB, pC, pD, intersections);
    checkIntersect(D, H, pA, pB, pC, pD, intersections);
    checkIntersect(C, G, pA, pB, pC, pD, intersections);
    
    // inclined plane color 
    glColor4f(1., 1., 1., 0.7); // partial-transparent white
    glDisable(GL_CULL_FACE); 
    glEnable(GL_BLEND); // enable transparency
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA); 
    // draw the inclined plane
    glBegin(GL_TRIANGLE_STRIP);
    for (int indx = 0; indx < intersections.size(); indx++) {
        glVertex3f(intersections[indx].x, intersections[indx].y, intersections[indx].z);
    }
    glEnd();
    glDisable(GL_BLEND);
    glEnable(GL_CULL_FACE);
}
