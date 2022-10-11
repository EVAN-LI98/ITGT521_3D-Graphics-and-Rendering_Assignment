//|___________________________________________________________________
//!
//! \file plane1_base.cpp
//!
//! \brief Base source code for the first plane assignment.
//!
//! Author: Mores Prachyabrued.
//!
//! Keyboard controls:
//!   w   = moves the plane forward w 控制飞机向前
//!   s   = moves the plane backward s 控制飞机向后
//!   q,e = rolls the plane q和e 控制飞机旋转
//!   z,c = pitches the plane z和c 控制飞机俯仰
//!   a,d = yaws the plane a和d 控制飞机偏航
//! 
//!   t   = moves the camera forward t 控制相机向前
//!   g   = moves the camera backward g 控制相机向后
//!   r,y = rolls the camera r和y 控制相机旋转
//!   f,h = pitches the camera f和h 控制相机俯仰
//!   v,n = yaws the camera v和n 控制相机偏航
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
//主视角相机
gmtl::Matrix44f cam_pose;   // C, as defined in the handout
gmtl::Matrix44f view_mat;   // View transform is C^-1 (inverse of the camera transform C)

//侧面副相机
gmtl::Matrix44f fixed_cam_pose;   // F, as defined in the handout
gmtl::Matrix44f fixed_view_mat;   // View transform is F^-1 (inverse of the camera transform C)


// 飞机以及相机的x,y,z轴移动和旋转
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

    //顺着z轴方向前后移动 具体参考ppt《ITGT521_1-2022_2D_3D_Transformations》中的p21
    // Positive Z-Translation
    ztransp_mat.set(1, 0, 0,            0,
                    0, 1, 0,            0,
                    0, 0, 1, TRANS_AMOUNT,
                    0, 0, 0,            1);
    ztransp_mat.setState(gmtl::Matrix44f::TRANS);

    gmtl::invert(ztransn_mat, ztransp_mat);
    
    //围绕x轴旋转 具体参考ppt《ITGT521_1-2022_2D_3D_Transformations》中的p25
    // Positive X-rotation (pitch)
    xrotp_mat.set(1,        0,         0, 0,
                  0, COSTHETA, -SINTHETA, 0,
                  0, SINTHETA,  COSTHETA, 0,
                  0,        0,         0, 1);
    xrotp_mat.setState(gmtl::Matrix44f::ORTHOGONAL);

    // Negative X-rotation (pitch)
    gmtl::invert(xrotn_mat, xrotp_mat);

    //围绕y轴旋转 具体参考ppt《ITGT521_1-2022_2D_3D_Transformations》中的p26
    // Positive Y-rotation (Yaw)
    yrotp_mat.set(COSTHETA, 0, SINTHETA, 0,
                         0, 1,        0, 0,
                 -SINTHETA, 0, COSTHETA, 0,
                         0, 0,        0, 1);
    yrotp_mat.setState(gmtl::Matrix44f::ORTHOGONAL);

    // Negative Y-rotation (Yaw)
    gmtl::invert(yrotn_mat, yrotp_mat);

    //围绕z轴旋转 具体参考ppt《ITGT521_1-2022_2D_3D_Transformations》中的p24
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

      //按住w飞机向前移动 按住s飞机向后移动
    case 'w': // Forward translation of the plane (positive Z-translation)
      plane_pose = plane_pose * ztransp_mat;
      break;
    case 's': // Backward translation of the plane
      plane_pose = plane_pose * ztransn_mat;
      break;

      //按住e飞机向逆时针旋转 按住q飞机向顺时针旋转
    case 'e': // Rolls the plane (+ Z-rot)
      plane_pose = plane_pose * zrotp_mat;
      break;
    case 'q': // Rolls the plane (- Z-rot)
      plane_pose = plane_pose * zrotn_mat;
      break;

      //按住z飞机向前旋转 按住c飞机向后旋转
    case 'z': // Pitches the plane (+ X-rot)
      plane_pose = plane_pose * xrotp_mat;
      break;
    case 'c': // Pitches the plane (- X-rot)
      plane_pose = plane_pose * xrotn_mat;
      break;

      //按住d飞机向左旋转 按住a飞机向右旋转
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
      
      //按住t相机向前移动 按住g飞机向后移动
    case 't': // Forward translation of the camera (negative Z-translation - cameras looks in its (local) -Z direction)
      cam_pose = cam_pose * ztransn_mat;
      break;
    case 'g': // Backward translation of the camera
      cam_pose = cam_pose * ztransp_mat;
      break;

      //按住r相机向逆时针旋转 按住y相机向顺时针旋转
    case 'r': // Rolls the camera (+ Z-rot)
      cam_pose = cam_pose * zrotp_mat;
      break;
    case 'y': // Rolls the camera (- Z-rot)
      cam_pose = cam_pose * zrotn_mat;
      break;

        //按住f飞机向前旋转 按住h飞机向后旋转
    case 'f': // Pitches the camera (+ X-rot)
      cam_pose = cam_pose * xrotp_mat;
      break;
    case 'h': // Pitches the camera (- X-rot)
      cam_pose = cam_pose * xrotn_mat;
      break;

        //按住v飞机向左旋转 按住n飞机向右旋转
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
    //绘制x,y,z轴的颜色
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

//绘制飞行模型
void DrawPlane(const float width, const float length, const float height)
{
  float w = width/2;
  float l = length/2;
  
  /*绘制飞机
  1,飞机尾部,上下两个部分 蓝白
  2,飞机尾翼,左右两个部分 白
  3,飞机机头,红黑
  4,飞机机舱,蓝色
  5,飞机机身,红黑
  6,飞机机翼,红色
  7,飞机后翼.黄色
  */
     glBegin(GL_POLYGON);
    //1
    //竖着的下部分
    glColor3f(1.0f, 1.0f, 1.0f);//颜色是白色white
    glVertex3f(0.0f, 1.0f, 0.5f);//
    glVertex3f(0.0f, 0.25f, 1.0f);//
    glVertex3f(0.0f, 0.5f, 4.0f);//
    glVertex3f(0.0f, 1.4f, 2.0f);//
    glColor3f(1.0f, 1.0f, 1.0f);//颜色是白色white

    //竖着的上部分
    glColor3f(0.0f, 3.0f, 1.0f);//颜色是蓝色blue
    glVertex3f(0.0f, 1.4f, 2.0f);//
    glVertex3f(0.0f, 3.0f, 1.0f);//
    glVertex3f(0.0f, 3.0f, 0.0f);//
    glVertex3f(0.0f, 1.0f, 0.5f);//
    glColor3f(0.0f, 3.0f, 1.0f);//颜色是蓝色blue
    glEnd();

    //2
    glBegin(GL_POLYGON);
    //横着的右半部分
    glColor3f(1.0f, 1.0f, 1.0f);//颜色是白white
    glVertex3f(0.0f, 2.0f, 0.25f);//
    glVertex3f(1.0f, 2.0f, -0.5f);//
    glVertex3f(2.0f, 2.0f, -0.5f);//
    glVertex3f(0.0f, 2.0f, 1.40f);//
   
    //横着的左半部分
    glColor3f(1.0f, 1.0f, 1.0f);//颜色是白white
    glVertex3f(0.0f, 2.0f, 0.25f);//
    glVertex3f(-1.0f, 2.0f, -0.5f);//
    glVertex3f(-2.0f, 2.0f, -0.5f);//
    glVertex3f(0.0f, 2.0f, 1.40f);//
    glEnd();

    //3
    //机头部分
    glBegin(GL_TRIANGLES);
    glColor3f(1.0f, 0.0f, 0.0f);//颜色是红red
    glVertex3f(-1.0f, 0.5f, 10.0f);//
    glVertex3f(1.0f, 0.5f, 10.0f);//
    glVertex3f(0.0f, 0.0f, 13.5f);//
    glEnd();
    glBegin(GL_TRIANGLES);
    glColor3f(0.0f, 0.0f, 0.0f);//颜色是黑black
    glVertex3f(1.0f, 0.5f, 10.0f);//
    glVertex3f(2.0f, -0.2f, 10.0f);//
    glVertex3f(0.0f, 0.0f, 13.5f);//
    glEnd();
    glBegin(GL_TRIANGLES);
    glColor3f(1.0f, 0.0f, 0.0f);//颜色是红red
    glVertex3f(2.0f, -0.2f, 10.0f);//
    glVertex3f(-2.0f, -0.2f, 10.0f);//
    glVertex3f(0.0f, 0.0f, 13.5f);//
    glEnd();
    glBegin(GL_TRIANGLES);
    glColor3f(0.0f, 0.0f, 0.0f);//颜色是黑black
    glVertex3f(-2.0f, -0.2f, 10.0f);//
    glVertex3f(-1.0f, 0.5f, 10.0f);//
    glVertex3f(0.0f, 0.0f, 13.5f);//
    glEnd();

    //4
    //飞机驾驶舱
    glBegin(GL_TRIANGLES);
    glColor3f(0.0f, 1.0f, 1.0f);//颜色是蓝blue
    glVertex3f(0.0f, 0.5f, 10.0f);//
    glVertex3f(0.25f, 1.5f, 9.25f);//
    glVertex3f(-0.25f, 1.5f, 9.25f);//
    glEnd();
    glBegin(GL_TRIANGLES);
    glColor3f(0.0f, 1.0f, 1.0f);//颜色是蓝blue
    glVertex3f(0.0f, 0.5f, 10.0f);//
    glVertex3f(0.25f, 1.5f, 9.25f);//
    glVertex3f(1.0f, 0.5f, 9.0f);//
    glEnd();
    glBegin(GL_TRIANGLES);
    glColor3f(0.0f, 1.0f, 1.0f);//颜色是蓝blue
    glVertex3f(0.0f, 0.5f, 10.0f);//
    glVertex3f(-0.25f, 1.5f, 9.25f);//
    glVertex3f(-1.0f, 0.5f, 9.0f);//
    glEnd();

    glBegin(GL_QUADS);
    glColor3f(0.0f, 1.0f, 1.0f);//颜色是蓝blue
    glVertex3f(-1.0f, 0.5f, 9.0f);//
    glVertex3f(-0.25f, 1.5f, 9.25f);//
    glVertex3f(-0.25f, 1.5f, 7.75f);//
    glVertex3f(-1.0f, 0.5f, 8.0f);//
    glEnd();
    glBegin(GL_QUADS);
    glColor3f(0.0f, 1.0f, 1.0f);//颜色是蓝blue
    glVertex3f(0.25f, 1.5f, 9.25f);//
    glVertex3f(-0.25f, 1.5f, 9.25f);//
    glVertex3f(-0.25f, 1.5f, 7.75f);//
    glVertex3f(0.25f, 1.5f, 7.75f);//
    glEnd();
    glBegin(GL_QUADS);
    glColor3f(0.0f, 1.0f, 1.0f);//颜色是蓝blue
    glVertex3f(1.0f, 0.5f, 9.0f);//
    glVertex3f(0.25f, 1.5f, 9.25f);//
    glVertex3f(0.25f, 1.5f, 7.75f);//
    glVertex3f(1.0f, 0.5f, 8.0f);//
    glEnd();

    glBegin(GL_TRIANGLES);
    glColor3f(0.0f, 1.0f, 1.0f);//颜色是蓝blue
    glVertex3f(0.0f, 0.5f, 7.0f);//
    glVertex3f(0.25f, 1.5f, 7.75f);//
    glVertex3f(-0.25f, 1.5f, 7.75f);//
    glEnd();
    glBegin(GL_TRIANGLES);
    glColor3f(0.0f, 1.0f, 1.0f);//颜色是蓝blue
    glVertex3f(0.0f, 0.5f, 7.0f);//
    glVertex3f(0.25f, 1.5f, 7.75f);//
    glVertex3f(1.0f, 0.5f, 8.0f);//
    glEnd();
    glBegin(GL_TRIANGLES);
    glColor3f(0.0f, 1.0f, 1.0f);//颜色是蓝blue
    glVertex3f(0.0f, 0.5f, 7.0f);//
    glVertex3f(-0.25f, 1.5f, 7.75f);//
    glVertex3f(-1.0f, 0.5f, 8.0f);//
    glEnd();

    //5
    //机身
    glBegin(GL_QUADS);
    glColor3f(1.0f, 0.0f, 0.0f);//颜色是红red
    glVertex3f(-0.25f, 0.25f, 1.0f);//
    glVertex3f(0.25f, 0.25f, 1.0f);//
    glVertex3f(0.6f, 0.5f, 4.0f);//
    glVertex3f(-0.6f, 0.5f, 4.0f);//
    glEnd();
    glBegin(GL_QUADS);
    glColor3f(1.0f, 0.0f, 0.0f);//颜色是红red
    glVertex3f(0.6f, 0.5f, 4.0f);//
    glVertex3f(-0.6f, 0.5f, 4.0f);//
    glVertex3f(-1.0f, 0.5f, 10.0f);//
    glVertex3f(1.0f, 0.5f, 10.0f);//
    glEnd();

    glBegin(GL_QUADS);
    glColor3f(1.0f, 0.0f, 0.0f);//颜色是红red
    glVertex3f(-0.5f, -0.2f, -1.0f);//
    glVertex3f(-0.25f, 0.2f, 1.0f);//
    glVertex3f(0.25f, 0.2f, 1.0f);//
    glVertex3f(0.5f, -0.2f, -1.0f);//
    glEnd();
    glBegin(GL_POLYGON);
    glColor3f(0.0f, 0.0f, 0.0f);//颜色是黑black
    glVertex3f(2.0f, -0.2f, 10.0f);//
    glVertex3f(1.0f, 0.5f, 10.0f);//
    glVertex3f(0.6f, 0.5f, 4.0f);//
    glVertex3f(0.25f, 0.2f, 1.0f);//
    glVertex3f(0.5f, -0.2f, -1.0f);//
    glVertex3f(0.5f, -0.2f, -1.0f);//
    glVertex3f(1.2f, -0.2f, 4.0f);//
    glEnd();
    glBegin(GL_POLYGON);
    glColor3f(0.0f, 0.0f, 0.0f);//颜色是黑black
    glVertex3f(0.0f, 0.0f, 13.5f);//
    glVertex3f(-2.0f,-0.2f, 10.0f);//
    glVertex3f(-0.5f,-0.2f, -1.0f);//
    glVertex3f(0.5f, -0.2f, -1.0f);//
    glVertex3f(2.0f, -0.2f, 10.0f);//
    glEnd();

    glBegin(GL_POLYGON);
    glColor3f(0.0f, 0.0f, 0.0f);//颜色是黑black
    glVertex3f(-2.0f, -0.2f, 10.0f);//
    glVertex3f(-1.0f, 0.5f, 10.0f);//
    glVertex3f(-0.6f, 0.5f, 4.0f);//
    glVertex3f(-0.25f, 0.2f, 1.0f);//
    glVertex3f(-0.5f, -0.2f, -1.0f);//
    glVertex3f(-0.5f, -0.2f, -1.0f);//
    glVertex3f(-1.2f, -0.2f, 4.0f);//
    glEnd();

    //6
    //飞机机翼
    glBegin(GL_POLYGON);
    glColor3f(1.0f, 0.0f, 0.0f);//颜色是红red 
    glVertex3f(-1.75f, 0.0f, 10.0f);//
    glVertex3f(-5.0f, 0.0f, 7.5f);//
    glVertex3f(-5.0f, 0.25f, 4.1f);//
    glVertex3f(-1.0f, 0.25f, 4.0f);//
    glVertex3f(-1.75f, 0.0f, 10.0f);//
    glEnd();
    glBegin(GL_POLYGON);
    glColor3f(0.0f, 0.0f, 0.0f);//颜色是黑black
    glVertex3f(-5.0f, 0.0f, 7.5f);//
    glVertex3f(-5.0f, 0.25f, 4.1f);//
    glVertex3f(-9.0f, 0.45f, 2.5f);//
    glVertex3f(-10.0f, 0.2f, 5.5f);//
    glEnd();
    glBegin(GL_POLYGON);

    glBegin(GL_POLYGON);
    glColor3f(1.0f, 0.0f, 0.0f);//颜色是红red 
    glVertex3f(1.75f, 0.0f, 10.0f);//
    glVertex3f(5.0f, 0.0f, 7.5f);//
    glVertex3f(5.0f, 0.25f, 4.1f);//
    glVertex3f(1.0f, 0.25f, 4.0f);//
    glVertex3f(1.75f, 0.0f, 10.0f);//
    glEnd();
    glBegin(GL_POLYGON);
    glColor3f(0.0f, 0.0f, 0.0f);//颜色是黑black
    glVertex3f(5.0f, 0.0f, 7.5f);//
    glVertex3f(5.0f, 0.25f, 4.1f);//
    glVertex3f(9.0f, 0.45f, 2.5f);//
    glVertex3f(10.0f, 0.2f, 5.5f);//
    glEnd();

    //7
    //飞机后翼
    glBegin(GL_POLYGON);
    glColor3f(1.0f, 1.0f, 0.0f);//颜色是黄色yellow
    glVertex3f(-0.5f, -0.25f,-1.0f);//
    glVertex3f(-0.55f, 0.1f, 1.0f);//
    glVertex3f(-2.75f, 0.1f,-0.25f);//
    glVertex3f(-2.5f, -0.25f,-1.1f);//
    glEnd();
    glBegin(GL_POLYGON);
    glColor3f(1.0f, 1.0f, 0.0f);//颜色是黄色yellow
    glVertex3f(0.5f, -0.25f, -1.0f);//
    glVertex3f(0.55f, 0.1f, 1.0f);//
    glVertex3f(2.75f, 0.1f, -0.25f);//
    glVertex3f(2.5f, -0.25f, -1.1f);//
    glEnd();

    /*
    glBegin(GL_POLYGON); 多边形
    glBegin(GL_QUADS); 四边形
    glBegin(GL_TRIANGLES); 三角形
    glEnd(); 结束
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