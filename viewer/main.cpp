
#include <glad/glad.h>
#include <GLFW/glfw3.h>

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>

#include <viewer/shader.hpp>
#include <viewer/camera.hpp>
#include <viewer/model.hpp>

#include <orion/rtc_parser.hpp>
#include <orion/raytracer.hpp>

#include <iostream>

void framebuffer_size_callback(GLFWwindow* window, int width, int height);
void mouse_callback(GLFWwindow* window, double xpos, double ypos);
void scroll_callback(GLFWwindow* window, double xoffset, double yoffset);
void processInput(GLFWwindow *window);

// settings (defaults)
unsigned int SCR_WIDTH = 600;
unsigned int SCR_HEIGHT = 400;

// camera
Camera camera(glm::vec3(0.0f, 0.0f, 3.0f));
float lastX = SCR_WIDTH / 2.0f;
float lastY = SCR_HEIGHT / 2.0f;
bool firstMouse = true;

// timing
float deltaTime = 0.0f;
float lastFrame = 0.0f;

GLFWwindow* window = NULL;

int main(int argc, char** argv)
{
    if (argc != 2) {
        printf("Usage:\nrviewer path/to/rtc/file.rtc\n");
        return 1;
    }

    // load lights and camera settings
    std::string rtc_path(argv[1]);
    orion::rtc_data rtc_data = orion::parse_rtc(rtc_path.c_str());
    SCR_WIDTH = rtc_data.xres;
    SCR_HEIGHT = rtc_data.yres;
    lastX = SCR_WIDTH / 2.0f;
    lastY = SCR_HEIGHT / 2.0f;

    auto otg = [](orion::vec3f v) -> glm::vec3 {
        return glm::vec3(v.x(), v.y(), v.z());
    };
    camera = Camera(otg(rtc_data.view_point), 
                    otg(rtc_data.view_point - rtc_data.look_at),
                    otg(rtc_data.vector_up));

    // load primary light source
    orion::Light light = rtc_data.lights[0];

    // glfw: initialize and configure
    // ------------------------------
    glfwInit();
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

#ifdef __APPLE__
    glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE); // uncomment this statement to fix compilation on OS X
#endif

    // glfw window creation
    // --------------------
    window = glfwCreateWindow(SCR_WIDTH, SCR_HEIGHT, "Orion Viewer", NULL, NULL);
    if (window == NULL)
    {
        std::cout << "Failed to create GLFW window" << std::endl;
        glfwTerminate();
        return -1;
    }
    glfwMakeContextCurrent(window);
    glfwSetFramebufferSizeCallback(window, framebuffer_size_callback);
    glfwSetCursorPosCallback(window, mouse_callback);
    glfwSetScrollCallback(window, scroll_callback);

    // tell GLFW to capture our mouse
    glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_DISABLED);

    // glad: load all OpenGL function pointers
    // ---------------------------------------
    if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress))
    {
        std::cout << "Failed to initialize GLAD" << std::endl;
        return -1;
    }

    // configure global opengl state
    // -----------------------------
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_CULL_FACE);

    // build and compile shaders
    // -------------------------
    Shader ourShader("shaders/untextured.vs", "shaders/untextured.fs");
    // Shader ourShader("shaders/simple_shader.vs", "shaders/simple_shader.fs");
    std::cout << "Shaders loaded successfully!" << std::endl;

    // load models
    // -----------
    std::string rtc_dir = rtc_path.substr(0, rtc_path.find_last_of('/'));
    Model ourModel = *(new Model(rtc_dir + "/" + rtc_data.obj_file));
    std::cout << "Models loaded successfully!" << std::endl;

    
    // draw in wireframe
    //glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
    glm::mat4 model = glm::mat4(1.0f);

    // render loop
    // -----------
    while (!glfwWindowShouldClose(window))
    {
        // per-frame time logic
        // --------------------
        float currentFrame = glfwGetTime();
        deltaTime = currentFrame - lastFrame;
        lastFrame = currentFrame;

        // input
        // -----
        processInput(window);

        // render
        // ------
        glClearColor(0.15f, 0.15f, 0.15f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        // don't forget to enable shader before setting uniforms
        ourShader.use();

        // view/projection transformations (the default zoom 45 is multiplied by 57/45 so it's about one radian)
        glm::mat4 projection = glm::perspective(glm::radians(camera.Zoom*57.f/45.f), (float)SCR_WIDTH / (float)SCR_HEIGHT, 0.1f, 100.0f);
        // glm::mat4 projection = glm::perspective(rtc_data.y_view, rtc_data.aspect_ratio, 0.1f, 100.0f);
        glm::mat4 view = camera.GetViewMatrix();
        ourShader.setMat4("vp", projection * view);

        // render the loaded model
        // model = glm::translate(model, glm::vec3(0.0f, -1.75f, 0.0f)); // translate it down so it's at the center of the scene
        // model = glm::scale(model, glm::vec3(0.3f, 0.3f, 0.3f));	// it's a bit too big for our scene, so scale it down
        model = glm::rotate(model, 0.1f*deltaTime, glm::vec3(0,1,0));
        ourShader.setMat4("modelMat", model);
        ourShader.setMat4("modelNorm", glm::transpose(glm::inverse(model)));
        // set light

        ourShader.setVec3("light.position", otg(light.position));
        ourShader.setVec3("light.ambient",  otg(light.color));
        ourShader.setVec3("light.diffuse",  otg(light.color));
        ourShader.setVec3("light.specular", otg(light.color));

        ourModel.Draw(ourShader);


        // glfw: swap buffers and poll IO events (keys pressed/released, mouse moved etc.)
        // -------------------------------------------------------------------------------
        glfwSwapBuffers(window);
        glfwPollEvents();
    }

    // glfw: terminate, clearing all previously allocated GLFW resources.
    // ------------------------------------------------------------------
    glfwTerminate();
    return 0;
}

// process all input: query GLFW whether relevant keys are pressed/released this frame and react accordingly
// ---------------------------------------------------------------------------------------------------------
void processInput(GLFWwindow *window)
{
    if (glfwGetKey(window, GLFW_KEY_ESCAPE) == GLFW_PRESS)
        glfwSetWindowShouldClose(window, true);

    if (glfwGetKey(window, GLFW_KEY_W) == GLFW_PRESS)
        camera.ProcessKeyboard(FORWARD, deltaTime);
    if (glfwGetKey(window, GLFW_KEY_S) == GLFW_PRESS)
        camera.ProcessKeyboard(BACKWARD, deltaTime);
    if (glfwGetKey(window, GLFW_KEY_A) == GLFW_PRESS)
        camera.ProcessKeyboard(LEFT, deltaTime);
    if (glfwGetKey(window, GLFW_KEY_D) == GLFW_PRESS)
        camera.ProcessKeyboard(RIGHT, deltaTime);
}

// glfw: whenever the window size changed (by OS or user resize) this callback function executes
// ---------------------------------------------------------------------------------------------
void framebuffer_size_callback(GLFWwindow* window, int width, int height)
{
    // Ignore unused parameter warning
    (void)window;

    // make sure the viewport matches the new window dimensions; note that width and 
    // height will be significantly larger than specified on retina displays.
    glViewport(0, 0, width, height);
}

// glfw: whenever the mouse moves, this callback is called
// -------------------------------------------------------
void mouse_callback(GLFWwindow* window, double xpos, double ypos)
{
    // Ignore unused parameter warning
    (void)window;

    if (firstMouse)
    {
        lastX = xpos;
        lastY = ypos;
        firstMouse = false;
    }

    float xoffset = xpos - lastX;
    float yoffset = lastY - ypos; // reversed since y-coordinates go from bottom to top

    lastX = xpos;
    lastY = ypos;

    camera.ProcessMouseMovement(xoffset, yoffset);
}

// glfw: whenever the mouse scroll wheel scrolls, this callback is called
// ----------------------------------------------------------------------
void scroll_callback( __attribute__((__unused__)) GLFWwindow* window, double xoffset, double yoffset)
{
    // Ignore unused parameter warning
    (void)window;
    (void)xoffset;
    
    camera.ProcessMouseScroll(yoffset);
}

