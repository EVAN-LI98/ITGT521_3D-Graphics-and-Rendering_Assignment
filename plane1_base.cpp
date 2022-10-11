//|___________________________________________________________________
//!
//! \file plane1_base.cpp
//!
//! \brief Base source code for the first plane assignment.
//!
//! Author: Mores Prachyabrued.
//!
//! Keyboard controls:
//!   w   = moves the plane forward w ���Ʒɻ���ǰ
//!   s   = moves the plane backward s ���Ʒɻ����
//!   q,e = rolls the plane q��e ���Ʒɻ���ת
//!   z,c = pitches the plane z��c ���Ʒɻ�����
//!   a,d = yaws the plane a��d ���Ʒɻ�ƫ��
//! 
//!   t   = moves the camera forward t ���������ǰ
//!   g   = moves the camera backward g ����������
//!   r,y = rolls the camera r��y ���������ת
//!   f,h = pitches the camera f��h �����������
//!   v,n = yaws the camera v��n �������ƫ��
//!
//! TODO: Extend the code to satisfy the requirements given in the assignment handout
//!
//! Note: Good programmer uses good comments! :)
//|___________________________________________________________________

//|___________________
//|
//| Includes
//|___________________

#include <math.h>

#include <gmtl/gmtl.h>

#include <GL/glut.h>

//|___________________
//|
//| Constants
//|___________________

// Plane dimensions
const float P_WIDTH  = 3;
const float P_LENGTH = 3;
const float P_HEIGHT = 1.5;

// Camera's view frustum 
const float CAM_FOV  = 60.0f;     // Field of view in degs

//|___________________
//|
//| Global Variables
//|___________________

// Track window dimensions, initialized to 800x600
int w_width    = 800;
int w_height   = 600;

// Plane pose (position & orientation)
gmtl::Matrix44f plane_pose; // T, as defined in the handout, initialized to IDENTITY by default

// Camera pose
//���ӽ����
gmtl::Matrix44f cam_pose;   // C, as defined in the handout
gmtl::Matrix44f view_mat;   // View transform is C^-1 (inverse of the camera transform C)

//���渱���
gmtl::Matrix44f fixed_cam_pose;   // F, as defined in the handout
gmtl::Matrix44f fixed_view_mat;   // View transform is F^-1 (inverse of the camera transform C)


// �ɻ��Լ������x,y,z���ƶ�����ת
// Transformation matrices applied to plane and camera poses
gmtl::Matrix44f ztransp_mat;
gmtl::Matrix44f ztransn_mat;
gmtl::Matrix44f zrotp_mat;
gmtl::Matrix44f zrotn_mat;
gmtl::Matrix44f xrotp_mat;
gmtl::Matrix44f xrotn_mat;
gmtl::Matrix44f yrotp_mat;
gmtl::Matrix44f yrotn_mat;


//|___________________
//|
//| Function Prototypes
//|___________________

void InitMatrices();
void InitGL(void);
void DisplayFunc(void);
void KeyboardFunc(unsigned char key, int x, int y);
void ReshapeFunc(int w, int h);
void DrawCoordinateFrame(const float l);
void DrawPlane(const float width, const float length, const float height);

//|____________________________________________________________________
//|
//| Function: InitMatrices
//|
//! \param None.
//! \return None.
//!
//! Initializes all the matrices
//|____________________________________________________________________

void InitMatrices()
{
    const float TRANS_AMOUNT = 1.0f;
    const float ROT_AMOUNT = gmtl::Math::deg2Rad(5.0f); // specified in degs, but get converted to radians

    const float COSTHETA = cos(ROT_AMOUNT);
    const float SINTHETA = sin(ROT_AMOUNT);

    //˳��z�᷽��ǰ���ƶ� ����ο�ppt��ITGT521_1-2022_2D_3D_Transformations���е�p21
    // Positive Z-Translation
    ztransp_mat.set(1, 0, 0,            0,
                    0, 1, 0,            0,
                    0, 0, 1, TRANS_AMOUNT,
                    0, 0, 0,            1);
    ztransp_mat.setState(gmtl::Matrix44f::TRANS);

    gmtl::invert(ztransn_mat, ztransp_mat);
    
    //Χ��x����ת ����ο�ppt��ITGT521_1-2022_2D_3D_Transformations���е�p25
    // Positive X-rotation (pitch)
    xrotp_mat.set(1,        0,         0, 0,
                  0, COSTHETA, -SINTHETA, 0,
                  0, SINTHETA,  COSTHETA, 0,
                  0,        0,         0, 1);
    xrotp_mat.setState(gmtl::Matrix44f::ORTHOGONAL);

    // Negative X-rotation (pitch)
    gmtl::invert(xrotn_mat, xrotp_mat);

    //Χ��y����ת ����ο�ppt��ITGT521_1-2022_2D_3D_Transformations���е�p26
    // Positive Y-rotation (Yaw)
    yrotp_mat.set(COSTHETA, 0, SINTHETA, 0,
                         0, 1,        0, 0,
                 -SINTHETA, 0, COSTHETA, 0,
                         0, 0,        0, 1);
    yrotp_mat.setState(gmtl::Matrix44f::ORTHOGONAL);

    // Negative Y-rotation (Yaw)
    gmtl::invert(yrotn_mat, yrotp_mat);

    //Χ��z����ת ����ο�ppt��ITGT521_1-2022_2D_3D_Transformations���е�p24
    // Positive Z-rotation (roll)
    zrotp_mat.set(COSTHETA, -SINTHETA, 0, 0,
                  SINTHETA,  COSTHETA, 0, 0,
                         0,         0, 1, 0,
                         0,         0, 0, 1);
    zrotp_mat.setState(gmtl::Matrix44f::ORTHOGONAL);

    // Negative Z-rotation (roll)
    gmtl::invert(zrotn_mat, zrotp_mat);

    // Inits plane pose
    plane_pose.set(1, 0, 0, 1.0f,
                   0, 1, 0, 0.0f,
                   0, 0, 1, 4.0f,
                   0, 0, 0, 1.0f);
    plane_pose.setState(gmtl::Matrix44f::AFFINE);     // AFFINE because the plane pose can contain both translation and rotation         

    // Inits camera pose and view transform
    cam_pose.set(1, 0, 0,  2.0f,
                 0, 1, 0,  1.0f,
                 0, 0, 1, 15.0f,
                 0, 0, 0,  1.0f);
    cam_pose.setState(gmtl::Matrix44f::AFFINE);
    gmtl::invert(view_mat, cam_pose);

    // Inits fixed camera pose and fixed view transform
    gmtl::Matrix44f fixed_rotate_mat, fixed_transform_mat;
    const float COS90 = cos(gmtl::Math::deg2Rad(-90.0f));
    const float SIN90 = sin(gmtl::Math::deg2Rad(-90.0f));

    fixed_rotate_mat.set(1,     0,      0, 0,
                         0, COS90, -SIN90, 0,
                         0, SIN90,  COS90, 0,
                         0,     0,      0, 1);
    fixed_rotate_mat.setState(gmtl::Matrix44f::ORTHOGONAL);

    fixed_transform_mat.set(1, 0, 0,  0,
                            0, 1, 0, 20,
                            0, 0, 1,  0,
                            0, 0, 0,  1);
    fixed_transform_mat.setState(gmtl::Matrix44f::TRANS);

    fixed_cam_pose = fixed_transform_mat * fixed_rotate_mat;
    gmtl::invert(fixed_view_mat, fixed_cam_pose);                 // View transform is the inverse of the camera pose
}
//|____________________________________________________________________
//|
//| Function: InitGL
//|
//! \param None.
//! \return None.
//!
//! OpenGL initializations
//|____________________________________________________________________

void InitGL(void)
{
  glClearColor(0.7f, 0.7f, 0.7f, 1.0f); 
  glEnable(GL_DEPTH_TEST); 
  glShadeModel(GL_SMOOTH);
}

//|____________________________________________________________________
//|
//| Function: DisplayFunc
//|
//! \param None.
//! \return None.
//!
//! GLUT display callback function: called for every redraw event.
//|____________________________________________________________________

void DisplayFunc(void)
{
  // Modelview matrix
  gmtl::Matrix44f modelview_mat;        // M, as defined in the handout

  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

//|____________________________________________________________________
//|
//| Viewport 1 rendering: shows the moving camera's view
//|____________________________________________________________________

  glViewport(0, 0, (GLsizei) w_width/2, (GLsizei) w_height);

  glMatrixMode(GL_PROJECTION);
  glLoadIdentity();
  gluPerspective(CAM_FOV, (float)w_width/(2*w_height), 0.1f, 100.0f);     // Check MSDN: google "gluPerspective msdn"

  // Approach1
  glMatrixMode(GL_MODELVIEW);
  glLoadIdentity();                          // A good practice for beginner

  // Draws world coordinate frame
  modelview_mat = view_mat;                  // M = C^-1
  glLoadMatrixf(modelview_mat.mData);
  DrawCoordinateFrame(100);

  // Draws plane and its local frame
  modelview_mat *= plane_pose;               // M = C^-1 * T
  glLoadMatrixf(modelview_mat.mData);
  DrawPlane(P_WIDTH, P_LENGTH, P_HEIGHT);
  DrawCoordinateFrame(3);

/*
  // Approach 2 (gives the same results as the approach 1)
  glMatrixMode(GL_MODELVIEW);

  // Draws world coordinate frame
  glLoadMatrixf(view_mat.mData);             // M = C^-1
  DrawCoordinateFrame(10);

  // Draws plane and its local frame
  glMultMatrixf(plane_pose.mData);           // M = C^-1 * T (OpenGL calls build transforms in left-to-right order)
  DrawPlane(P_WIDTH, P_LENGTH, P_HEIGHT);
  DrawCoordinateFrame(3);
*/

//|____________________________________________________________________
//|
//| TODO: Viewport 2 rendering: shows the fixed top-down view
//|____________________________________________________________________

  // glViewport...

  glViewport(w_width / 2, 0, (GLsizei)w_width / 2, (GLsizei)w_height);
  
  glMatrixMode(GL_PROJECTION);
  glLoadIdentity();
  gluPerspective(CAM_FOV, (float)w_width/(2*w_height), 0.1f, 100.0f);

  // glMatrixMode(GL_MODELVIEW);
  // glLoadIdentity(); 
  // ...

  // Approach2
  glMatrixMode(GL_MODELVIEW);
  glLoadIdentity();                               // A good practice for beginner

  //Draw world coordinate frame
  modelview_mat = fixed_view_mat;                 // M = F^-1
  glLoadMatrixf(modelview_mat.mData);
  DrawCoordinateFrame(100);

  // Draws plane and its local frame
  modelview_mat *= plane_pose;                    // M = F^-1 * T
  glLoadMatrixf(modelview_mat.mData);
  DrawPlane(P_WIDTH, P_LENGTH, P_HEIGHT);
  DrawCoordinateFrame(3);

  // Draws movable camera (its local frame)
  modelview_mat = fixed_view_mat * cam_pose;      // M = F^-1 * C
  glLoadMatrixf(modelview_mat.mData);
  DrawCoordinateFrame(3);


  glFlush();
}

//|____________________________________________________________________
//|
//| Function: KeyboardFunc
//|
//! \param None.
//! \return None.
//!
//! GLUT keyboard callback function: called for every key press event.
//|____________________________________________________________________

void KeyboardFunc(unsigned char key, int x, int y)
{
  switch (key) {
//|____________________________________________________________________
//|
//| Plane controls
//|____________________________________________________________________

      //��סw�ɻ���ǰ�ƶ� ��סs�ɻ�����ƶ�
    case 'w': // Forward translation of the plane (positive Z-translation)
      plane_pose = plane_pose * ztransp_mat;
      break;
    case 's': // Backward translation of the plane
      plane_pose = plane_pose * ztransn_mat;
      break;

      //��סe�ɻ�����ʱ����ת ��סq�ɻ���˳ʱ����ת
    case 'e': // Rolls the plane (+ Z-rot)
      plane_pose = plane_pose * zrotp_mat;
      break;
    case 'q': // Rolls the plane (- Z-rot)
      plane_pose = plane_pose * zrotn_mat;
      break;

      //��סz�ɻ���ǰ��ת ��סc�ɻ������ת
    case 'z': // Pitches the plane (+ X-rot)
      plane_pose = plane_pose * xrotp_mat;
      break;
    case 'c': // Pitches the plane (- X-rot)
      plane_pose = plane_pose * xrotn_mat;
      break;

      //��סd�ɻ�������ת ��סa�ɻ�������ת
    case 'd': // Yaws the plane (+ Y-rot)
      plane_pose = plane_pose * yrotn_mat;
      break;
    case 'a': // Yaws the plane (- Y-rot)
      plane_pose = plane_pose * yrotp_mat;
      break;
    

    // TODO: Add the remaining controls/transforms        

//|____________________________________________________________________
//|
//| Camera controls
//|____________________________________________________________________
      
      //��סt�����ǰ�ƶ� ��סg�ɻ�����ƶ�
    case 't': // Forward translation of the camera (negative Z-translation - cameras looks in its (local) -Z direction)
      cam_pose = cam_pose * ztransn_mat;
      break;
    case 'g': // Backward translation of the camera
      cam_pose = cam_pose * ztransp_mat;
      break;

      //��סr�������ʱ����ת ��סy�����˳ʱ����ת
    case 'r': // Rolls the camera (+ Z-rot)
      cam_pose = cam_pose * zrotp_mat;
      break;
    case 'y': // Rolls the camera (- Z-rot)
      cam_pose = cam_pose * zrotn_mat;
      break;

        //��סf�ɻ���ǰ��ת ��סh�ɻ������ת
    case 'f': // Pitches the camera (+ X-rot)
      cam_pose = cam_pose * xrotp_mat;
      break;
    case 'h': // Pitches the camera (- X-rot)
      cam_pose = cam_pose * xrotn_mat;
      break;

        //��סv�ɻ�������ת ��סn�ɻ�������ת
    case 'v': // Yaws the camera (+ Y-rot)
      cam_pose = cam_pose * yrotn_mat;
      break;
    case 'n': // Yaws the camera (- Y-rot)
      cam_pose = cam_pose * yrotp_mat;
      break;

    // TODO: Add the remaining controls
  }

  gmtl::invert(view_mat, cam_pose);       // Updates view transform to reflect the change in camera transform
  glutPostRedisplay();                    // Asks GLUT to redraw the screen
}

//|____________________________________________________________________
//|
//| Function: ReshapeFunc
//|
//! \param None.
//! \return None.
//!
//! GLUT reshape callback function: called everytime the window is resized.
//|____________________________________________________________________

void ReshapeFunc(int w, int h)
{
  // Track the current window dimensions
  w_width  = w;
  w_height = h;
}

//|____________________________________________________________________
//|
//| Function: DrawCoordinateFrame
//|
//! \param l      [in] length of the three axes.
//! \return None.
//!
//! Draws coordinate frame consisting of the three principal axes.
//|____________________________________________________________________

void DrawCoordinateFrame(const float l)
{
  glBegin(GL_LINES);
    //����x,y,z�����ɫ
    // X axis is red
    glColor3f( 1.0f, 0.0f, 0.0f);
    glVertex3f(0.0f, 0.0f, 0.0f);
	  glVertex3f(   l, 0.0f, 0.0f);

    // Y axis is green
    glColor3f( 0.0f, 1.0f, 0.0f);
    glVertex3f(0.0f, 0.0f, 0.0f);
	  glVertex3f(0.0f,    l, 0.0f);

    // Z axis is blue
    glColor3f( 0.0f, 0.0f, 1.0f);
    glVertex3f(0.0f, 0.0f, 0.0f);
	  glVertex3f(0.0f, 0.0f,    l);
  glEnd();
}


//|____________________________________________________________________
//|
//| Function: DrawPlane
//|
//! \param width       [in] Width  of the plane.
//! \param length      [in] Length of the plane.
//! \param height      [in] Height of the plane.
//! \return None.
//!
//! Draws the plane.
//|____________________________________________________________________

//���Ʒ���ģ��
void DrawPlane(const float width, const float length, const float height)
{
  float w = width/2;
  float l = length/2;
  
  /*���Ʒɻ�
  1,�ɻ�β��,������������ ����
  2,�ɻ�β��,������������ ��
  3,�ɻ���ͷ,���
  4,�ɻ�����,��ɫ
  5,�ɻ�����,���
  6,�ɻ�����,��ɫ
  7,�ɻ�����.��ɫ
  */
     glBegin(GL_POLYGON);
    //1
    //���ŵ��²���
    glColor3f(1.0f, 1.0f, 1.0f);//��ɫ�ǰ�ɫwhite
    glVertex3f(0.0f, 1.0f, 0.5f);//
    glVertex3f(0.0f, 0.25f, 1.0f);//
    glVertex3f(0.0f, 0.5f, 4.0f);//
    glVertex3f(0.0f, 1.4f, 2.0f);//
    glColor3f(1.0f, 1.0f, 1.0f);//��ɫ�ǰ�ɫwhite

    //���ŵ��ϲ���
    glColor3f(0.0f, 3.0f, 1.0f);//��ɫ����ɫblue
    glVertex3f(0.0f, 1.4f, 2.0f);//
    glVertex3f(0.0f, 3.0f, 1.0f);//
    glVertex3f(0.0f, 3.0f, 0.0f);//
    glVertex3f(0.0f, 1.0f, 0.5f);//
    glColor3f(0.0f, 3.0f, 1.0f);//��ɫ����ɫblue
    glEnd();

    //2
    glBegin(GL_POLYGON);
    //���ŵ��Ұ벿��
    glColor3f(1.0f, 1.0f, 1.0f);//��ɫ�ǰ�white
    glVertex3f(0.0f, 2.0f, 0.25f);//
    glVertex3f(1.0f, 2.0f, -0.5f);//
    glVertex3f(2.0f, 2.0f, -0.5f);//
    glVertex3f(0.0f, 2.0f, 1.40f);//
   
    //���ŵ���벿��
    glColor3f(1.0f, 1.0f, 1.0f);//��ɫ�ǰ�white
    glVertex3f(0.0f, 2.0f, 0.25f);//
    glVertex3f(-1.0f, 2.0f, -0.5f);//
    glVertex3f(-2.0f, 2.0f, -0.5f);//
    glVertex3f(0.0f, 2.0f, 1.40f);//
    glEnd();

    //3
    //��ͷ����
    glBegin(GL_TRIANGLES);
    glColor3f(1.0f, 0.0f, 0.0f);//��ɫ�Ǻ�red
    glVertex3f(-1.0f, 0.5f, 10.0f);//
    glVertex3f(1.0f, 0.5f, 10.0f);//
    glVertex3f(0.0f, 0.0f, 13.5f);//
    glEnd();
    glBegin(GL_TRIANGLES);
    glColor3f(0.0f, 0.0f, 0.0f);//��ɫ�Ǻ�black
    glVertex3f(1.0f, 0.5f, 10.0f);//
    glVertex3f(2.0f, -0.2f, 10.0f);//
    glVertex3f(0.0f, 0.0f, 13.5f);//
    glEnd();
    glBegin(GL_TRIANGLES);
    glColor3f(1.0f, 0.0f, 0.0f);//��ɫ�Ǻ�red
    glVertex3f(2.0f, -0.2f, 10.0f);//
    glVertex3f(-2.0f, -0.2f, 10.0f);//
    glVertex3f(0.0f, 0.0f, 13.5f);//
    glEnd();
    glBegin(GL_TRIANGLES);
    glColor3f(0.0f, 0.0f, 0.0f);//��ɫ�Ǻ�black
    glVertex3f(-2.0f, -0.2f, 10.0f);//
    glVertex3f(-1.0f, 0.5f, 10.0f);//
    glVertex3f(0.0f, 0.0f, 13.5f);//
    glEnd();

    //4
    //�ɻ���ʻ��
    glBegin(GL_TRIANGLES);
    glColor3f(0.0f, 1.0f, 1.0f);//��ɫ����blue
    glVertex3f(0.0f, 0.5f, 10.0f);//
    glVertex3f(0.25f, 1.5f, 9.25f);//
    glVertex3f(-0.25f, 1.5f, 9.25f);//
    glEnd();
    glBegin(GL_TRIANGLES);
    glColor3f(0.0f, 1.0f, 1.0f);//��ɫ����blue
    glVertex3f(0.0f, 0.5f, 10.0f);//
    glVertex3f(0.25f, 1.5f, 9.25f);//
    glVertex3f(1.0f, 0.5f, 9.0f);//
    glEnd();
    glBegin(GL_TRIANGLES);
    glColor3f(0.0f, 1.0f, 1.0f);//��ɫ����blue
    glVertex3f(0.0f, 0.5f, 10.0f);//
    glVertex3f(-0.25f, 1.5f, 9.25f);//
    glVertex3f(-1.0f, 0.5f, 9.0f);//
    glEnd();

    glBegin(GL_QUADS);
    glColor3f(0.0f, 1.0f, 1.0f);//��ɫ����blue
    glVertex3f(-1.0f, 0.5f, 9.0f);//
    glVertex3f(-0.25f, 1.5f, 9.25f);//
    glVertex3f(-0.25f, 1.5f, 7.75f);//
    glVertex3f(-1.0f, 0.5f, 8.0f);//
    glEnd();
    glBegin(GL_QUADS);
    glColor3f(0.0f, 1.0f, 1.0f);//��ɫ����blue
    glVertex3f(0.25f, 1.5f, 9.25f);//
    glVertex3f(-0.25f, 1.5f, 9.25f);//
    glVertex3f(-0.25f, 1.5f, 7.75f);//
    glVertex3f(0.25f, 1.5f, 7.75f);//
    glEnd();
    glBegin(GL_QUADS);
    glColor3f(0.0f, 1.0f, 1.0f);//��ɫ����blue
    glVertex3f(1.0f, 0.5f, 9.0f);//
    glVertex3f(0.25f, 1.5f, 9.25f);//
    glVertex3f(0.25f, 1.5f, 7.75f);//
    glVertex3f(1.0f, 0.5f, 8.0f);//
    glEnd();

    glBegin(GL_TRIANGLES);
    glColor3f(0.0f, 1.0f, 1.0f);//��ɫ����blue
    glVertex3f(0.0f, 0.5f, 7.0f);//
    glVertex3f(0.25f, 1.5f, 7.75f);//
    glVertex3f(-0.25f, 1.5f, 7.75f);//
    glEnd();
    glBegin(GL_TRIANGLES);
    glColor3f(0.0f, 1.0f, 1.0f);//��ɫ����blue
    glVertex3f(0.0f, 0.5f, 7.0f);//
    glVertex3f(0.25f, 1.5f, 7.75f);//
    glVertex3f(1.0f, 0.5f, 8.0f);//
    glEnd();
    glBegin(GL_TRIANGLES);
    glColor3f(0.0f, 1.0f, 1.0f);//��ɫ����blue
    glVertex3f(0.0f, 0.5f, 7.0f);//
    glVertex3f(-0.25f, 1.5f, 7.75f);//
    glVertex3f(-1.0f, 0.5f, 8.0f);//
    glEnd();

    //5
    //����
    glBegin(GL_QUADS);
    glColor3f(1.0f, 0.0f, 0.0f);//��ɫ�Ǻ�red
    glVertex3f(-0.25f, 0.25f, 1.0f);//
    glVertex3f(0.25f, 0.25f, 1.0f);//
    glVertex3f(0.6f, 0.5f, 4.0f);//
    glVertex3f(-0.6f, 0.5f, 4.0f);//
    glEnd();
    glBegin(GL_QUADS);
    glColor3f(1.0f, 0.0f, 0.0f);//��ɫ�Ǻ�red
    glVertex3f(0.6f, 0.5f, 4.0f);//
    glVertex3f(-0.6f, 0.5f, 4.0f);//
    glVertex3f(-1.0f, 0.5f, 10.0f);//
    glVertex3f(1.0f, 0.5f, 10.0f);//
    glEnd();

    glBegin(GL_QUADS);
    glColor3f(1.0f, 0.0f, 0.0f);//��ɫ�Ǻ�red
    glVertex3f(-0.5f, -0.2f, -1.0f);//
    glVertex3f(-0.25f, 0.2f, 1.0f);//
    glVertex3f(0.25f, 0.2f, 1.0f);//
    glVertex3f(0.5f, -0.2f, -1.0f);//
    glEnd();
    glBegin(GL_POLYGON);
    glColor3f(0.0f, 0.0f, 0.0f);//��ɫ�Ǻ�black
    glVertex3f(2.0f, -0.2f, 10.0f);//
    glVertex3f(1.0f, 0.5f, 10.0f);//
    glVertex3f(0.6f, 0.5f, 4.0f);//
    glVertex3f(0.25f, 0.2f, 1.0f);//
    glVertex3f(0.5f, -0.2f, -1.0f);//
    glVertex3f(0.5f, -0.2f, -1.0f);//
    glVertex3f(1.2f, -0.2f, 4.0f);//
    glEnd();
    glBegin(GL_POLYGON);
    glColor3f(0.0f, 0.0f, 0.0f);//��ɫ�Ǻ�black
    glVertex3f(0.0f, 0.0f, 13.5f);//
    glVertex3f(-2.0f,-0.2f, 10.0f);//
    glVertex3f(-0.5f,-0.2f, -1.0f);//
    glVertex3f(0.5f, -0.2f, -1.0f);//
    glVertex3f(2.0f, -0.2f, 10.0f);//
    glEnd();

    glBegin(GL_POLYGON);
    glColor3f(0.0f, 0.0f, 0.0f);//��ɫ�Ǻ�black
    glVertex3f(-2.0f, -0.2f, 10.0f);//
    glVertex3f(-1.0f, 0.5f, 10.0f);//
    glVertex3f(-0.6f, 0.5f, 4.0f);//
    glVertex3f(-0.25f, 0.2f, 1.0f);//
    glVertex3f(-0.5f, -0.2f, -1.0f);//
    glVertex3f(-0.5f, -0.2f, -1.0f);//
    glVertex3f(-1.2f, -0.2f, 4.0f);//
    glEnd();

    //6
    //�ɻ�����
    glBegin(GL_POLYGON);
    glColor3f(1.0f, 0.0f, 0.0f);//��ɫ�Ǻ�red 
    glVertex3f(-1.75f, 0.0f, 10.0f);//
    glVertex3f(-5.0f, 0.0f, 7.5f);//
    glVertex3f(-5.0f, 0.25f, 4.1f);//
    glVertex3f(-1.0f, 0.25f, 4.0f);//
    glVertex3f(-1.75f, 0.0f, 10.0f);//
    glEnd();
    glBegin(GL_POLYGON);
    glColor3f(0.0f, 0.0f, 0.0f);//��ɫ�Ǻ�black
    glVertex3f(-5.0f, 0.0f, 7.5f);//
    glVertex3f(-5.0f, 0.25f, 4.1f);//
    glVertex3f(-9.0f, 0.45f, 2.5f);//
    glVertex3f(-10.0f, 0.2f, 5.5f);//
    glEnd();
    glBegin(GL_POLYGON);

    glBegin(GL_POLYGON);
    glColor3f(1.0f, 0.0f, 0.0f);//��ɫ�Ǻ�red 
    glVertex3f(1.75f, 0.0f, 10.0f);//
    glVertex3f(5.0f, 0.0f, 7.5f);//
    glVertex3f(5.0f, 0.25f, 4.1f);//
    glVertex3f(1.0f, 0.25f, 4.0f);//
    glVertex3f(1.75f, 0.0f, 10.0f);//
    glEnd();
    glBegin(GL_POLYGON);
    glColor3f(0.0f, 0.0f, 0.0f);//��ɫ�Ǻ�black
    glVertex3f(5.0f, 0.0f, 7.5f);//
    glVertex3f(5.0f, 0.25f, 4.1f);//
    glVertex3f(9.0f, 0.45f, 2.5f);//
    glVertex3f(10.0f, 0.2f, 5.5f);//
    glEnd();

    //7
    //�ɻ�����
    glBegin(GL_POLYGON);
    glColor3f(1.0f, 1.0f, 0.0f);//��ɫ�ǻ�ɫyellow
    glVertex3f(-0.5f, -0.25f,-1.0f);//
    glVertex3f(-0.55f, 0.1f, 1.0f);//
    glVertex3f(-2.75f, 0.1f,-0.25f);//
    glVertex3f(-2.5f, -0.25f,-1.1f);//
    glEnd();
    glBegin(GL_POLYGON);
    glColor3f(1.0f, 1.0f, 0.0f);//��ɫ�ǻ�ɫyellow
    glVertex3f(0.5f, -0.25f, -1.0f);//
    glVertex3f(0.55f, 0.1f, 1.0f);//
    glVertex3f(2.75f, 0.1f, -0.25f);//
    glVertex3f(2.5f, -0.25f, -1.1f);//
    glEnd();

    /*
    glBegin(GL_POLYGON); �����
    glBegin(GL_QUADS); �ı���
    glBegin(GL_TRIANGLES); ������
    glEnd(); ����
    */
}

//|____________________________________________________________________
//|
//| Function: main
//|
//! \param None.
//! \return None.
//!
//! Program entry point
//|____________________________________________________________________

int main(int argc, char **argv)
{ 
  InitMatrices();

  glutInit(&argc, argv);

  glutInitDisplayMode(GLUT_SINGLE | GLUT_RGB | GLUT_DEPTH);
  glutInitWindowSize(w_width, w_height);
  
  glutCreateWindow("Plane Episode 1");

  glutDisplayFunc(DisplayFunc);
  glutReshapeFunc(ReshapeFunc);
  glutKeyboardFunc(KeyboardFunc);
  
  InitGL();

  glutMainLoop();

  return 0;
}