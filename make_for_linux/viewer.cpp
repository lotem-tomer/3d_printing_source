#if defined(_WIN32)
#include <windows.h>
#endif

#include <GL/gl.h>
#include <GL/glut.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <time.h>
#include <unistd.h>
#include <math.h>
#include <vector>
#include <string>

#include "Polyhedron_viewer.hpp"
#include "Sphere_bound.hpp"

#define WINDOW_WIDTH    720
#define WINDOW_HEIGHT   576

static int s_win_width(WINDOW_WIDTH);
static int s_win_height(WINDOW_HEIGHT);
static char* s_prog_name(0);

static bool s_draw_lines = true;

static Polyhedron_viewer* s_polyhedron_viewer(nullptr);

static const float s_radius_scale = 1.1f;
static const float s_field_of_view = 0.785398f;    // 45 degrees
static const float s_nearest_clipping_plane = 0.1f;
static const float s_far_plane_scale = 64;
static const float s_aspect_ratio = 1.333333f;
static bool s_gfx_initialized = false;

/*! Intialize the graphics pipe */
void init_gfx()
{
  float params[] = {0.0f, 0.0f, 0.0f, 1.0f};
  glLightModelfv(GL_LIGHT_MODEL_AMBIENT, params);

  float l[] = {1, 1, 1, 1};
  glLightfv(GL_LIGHT0, GL_AMBIENT, l);
  glLightfv(GL_LIGHT0, GL_DIFFUSE, l);
  glLightfv(GL_LIGHT0, GL_SPECULAR, l);

  float pos[] = {1, 1, 1, 1};
  glLightfv(GL_LIGHT0, GL_POSITION, pos);

  float ambient[] = {0.16f, 0.16f, 0.16f, 1};
  glMaterialfv(GL_FRONT, GL_AMBIENT, ambient);

  float diffuse[] = {0.8f, 0.8f, 0.8f, 1};
  glMaterialfv(GL_FRONT, GL_DIFFUSE, diffuse);

  float specular[] = {0, 0, 0, 1};
  glMaterialfv(GL_FRONT, GL_SPECULAR, specular);

  float emission[] = {0, 0, 0, 1};
  glMaterialfv(GL_FRONT, GL_EMISSION, emission);
  glMaterialf(GL_FRONT, GL_SHININESS, 25.6f);

  glEnable(GL_LIGHT0);
  glEnable(GL_DEPTH_TEST);
  glEnable(GL_CULL_FACE);
  glColor3f(0.5f, 0.5f, 1);
  glLineWidth(2);

  s_gfx_initialized = true;
}

/*! Set the clipping planes so that the frustum contains the bounding-sphere
 * \param center the center of the bounding sphere
 * \param radius the radius of the bounding sphere
 */
void set_clipping_planes(const float* center, float radius)
{
  radius *= s_radius_scale;
  float my_sin = sinf(s_field_of_view / 2);
  float dist = radius / my_sin;

  //! \todo Introduce a user-option that makes the frustum tight
  float near_plane = dist - radius;
  if (near_plane < s_nearest_clipping_plane) {
    near_plane = s_nearest_clipping_plane;
    dist = near_plane + radius;
  }
  float far_plane = dist + radius;

  near_plane = s_nearest_clipping_plane;
  far_plane = dist * s_far_plane_scale;

  glMatrixMode(GL_MODELVIEW);
  glLoadIdentity();
  glTranslatef(-center[0], -center[1], -center[2] - dist);

  // Projection matrix
  float top = tanf(0.5f * s_field_of_view) * near_plane;
  float right = top * s_aspect_ratio;
  float left = -right;
  float bottom = -top;
  glMatrixMode(GL_PROJECTION);
  glLoadIdentity();
  glFrustum(left, right, bottom, top, near_plane, far_plane);
}

/*! Handle window rsizing */
void reshape(GLint width, GLint height)
{
  s_win_width = width;
  s_win_height = height;
  glViewport(0, 0, width, height);
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
  if (!s_gfx_initialized) init_gfx();

  float center[3];
  float radius;
  Sphere_bound sphere_bound;
  sphere_bound(s_polyhedron_viewer->get_polyhedron(), center, radius);
  set_clipping_planes(center, radius);
}

/*! Invoked when the window should be rendered as a response to user events */
void display(void)
{
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
  s_polyhedron_viewer->set_draw_lines(s_draw_lines);
  s_polyhedron_viewer->draw();
  glutSwapBuffers();
}

/*! Clear before exit */
void clear()
{ if (! s_polyhedron_viewer->is_empty()) s_polyhedron_viewer->clear(); }

/*! Print help message */
void print_keyboard_help(void)
{
  std::cout << "  h\t- display help" << std::endl
            << "  Esc\t- exit " << s_prog_name
            << std::endl;
}

/*! Hnalde key strokes */
void keyboard(unsigned char c, int x, int y)
{
  switch (c) {
   case 'l':
   case 'L': s_draw_lines = !s_draw_lines; break;
   case 27: clear(); exit(0); break;
   case 'h': print_keyboard_help(); break;
  }
  glutPostRedisplay();              // force a redraw
}

/*! Handle mouse events */
void mouse(int button, int state, int x, int y){}

/*! Handle motion events */
void motion(int x, int y){}

/*! Extract the file name from the command line */
bool check_filename(const char* filename)
{
  struct stat buf;
  int rc = stat(filename, &buf);
  if (rc < 0 || ((buf.st_mode & S_IFDIR) == S_IFDIR)) return false;
  return true;
}

/*! */
int main(int argc, char* argv[])
{
  s_prog_name = strrchr(argv[0], '\\');
  s_prog_name = (s_prog_name) ? s_prog_name+1 : argv[0];

  // Default values:
  bool full_screen(false);
  bool interactive(true);

  if (argc < 2) {
    std::cerr << "Error: Input file missing!" << std::endl;
    return -1;
  }
  const auto* filename = argv[1];
  glutInit(&argc, argv);

  try {
    if (!check_filename(filename)) {
      std::cerr << "Error: Input file cannot be found!" << std::endl;
      return -1;
    }
    s_polyhedron_viewer = new Polyhedron_viewer;
    if (! s_polyhedron_viewer->parse(filename)) {
      return -1;
    }
  }
  catch(std::exception& e) {
    std::cerr << "error: " << e.what() << std::endl;
    return 1;
  }
  catch(...) {
    std::cerr << "Exception of unknown type!" << std::endl;
  }

  // glut callback routinces:
  glutInitWindowSize(s_win_width, s_win_height);
  glutInitWindowPosition (0, 0);
  glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGBA | GLUT_DEPTH | GLUT_ACCUM);
  // glutInitDisplayString("rgba double depth>=16 acc");
  glutCreateWindow(s_prog_name);
  if (full_screen) glutFullScreen();
  if (!interactive) glutSetCursor(GLUT_CURSOR_NONE);

  glutReshapeFunc(reshape);
  glutDisplayFunc(display);
  glutKeyboardFunc(keyboard);
  glutMouseFunc(mouse);
  glutMotionFunc(motion);
  glutIdleFunc(NULL);
  glutMainLoop();
  return 0;
}
