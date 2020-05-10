#pragma once


#include "gui/Window.hpp"
#include "gui/CommonControls.hpp"
#include "gui/Image.hpp"
#include "3d/CameraControls.hpp"
#include "gpu/Buffer.hpp"

#include <vector>
#include <memory>

#include "RayTracer.hpp"

#include "AreaLight.hpp"
#include "PathTraceRenderer.hpp"


namespace FW
{
    //------------------------------------------------------------------------

    class App : public Window::Listener, public CommonControls::StateObject
    {
    private:
        enum Action
        {
            Action_None,

            Action_LoadMesh,
            Action_ReloadMesh,
            Action_SaveMesh,
            Action_LoadBVH,

            Action_ResetCamera,
            Action_EncodeCameraSignature,
            Action_DecodeCameraSignature,

            Action_NormalizeScale,
            Action_FlipXY,
            Action_FlipYZ,
            Action_FlipZ,

            Action_NormalizeNormals,
            Action_FlipNormals,
            Action_RecomputeNormals,

            Action_FlipTriangles,

            Action_CleanMesh,
            Action_CollapseVertices,
            Action_DupVertsPerSubmesh,
            Action_FixMaterialColors,
            Action_DownscaleTextures,
            Action_ChopBehindNear,

            Action_PathTraceMode,
            Action_PlaceLightSourceAtCamera,

            Action_AddNewLight,
            Action_RemoveSelectedLight,
            Action_ChangeSelectedLight
        };

        enum CullMode
        {
            CullMode_None = 0,
            CullMode_CW,
            CullMode_CCW,
        };

        struct RayVertex
        {
            Vec3f pos;
            U32 color;
        };

        enum bvh_build_method { None, SAH };

        enum SamplingType { AO_sampling, AA_sampling };

        // this structure holds the necessary arguments when rendering using command line parameters
        struct
        {
            bool batch_render;
            SplitMode splitMode; // the BVH builder to use
            int spp; // samples per pixel to use
            SamplingType sample_type; // AO or AA sampling; AO includes one extra sample for the primary ray
            bool output_images; // might be useful to compare images with the example
            bool use_textures; // whether or not textures are used
            bool use_arealights; // whether or not area light sampling is used
            bool enable_reflections; // whether to compute reflections in whitted integrator
            float ao_length;
        } m_settings;

        struct
        {
            std::string state_name; // filenames of the state and scene files
            std::string scene_name;
            int rayCount;
            int build_time, trace_time;
        } m_results;

    public:
        App(std::vector<std::string>& cmd_args);
        virtual ~App(void);

        virtual bool handleEvent(const Window::Event& ev);
        virtual void readState(StateDump& d);
        virtual void writeState(StateDump& d) const;

    private:
        void process_args(std::vector<std::string>& args);

        void waitKey(void);
        void renderFrame(GLContext* gl);
        void renderScene(GLContext* gl, const Mat4f& worldToCamera, const Mat4f& projection);
        void loadMesh(const String& fileName);
        void saveMesh(const String& fileName);
        void loadRayDump(const String& fileName);

        static void downscaleTextures(MeshBase* mesh);
        static void chopBehindPlane(MeshBase* mesh, const Vec4f& pleq);

        static bool fileExists(const String& fileName);

        // 
        void constructTracer(void);

        void blitRttToScreen(GLContext* gl);


    private:
        App(const App&); // forbidden
        App& operator=(const App&); // forbidden

    private:
        Window m_window;
        CommonControls m_commonCtrl;
        CameraControls m_cameraCtrl;

        Action m_action;
        String m_meshFileName;
        CullMode m_cullMode;
        Timer m_timer;

        std::unique_ptr<RayTracer> m_rt;
        std::vector<Vec3f> m_rtVertexPositions; // kept only for MD5 checksums
        std::vector<RTTriangle> m_rtTriangles;
        std::vector<RTTriangle*> m_rtLightTriangles; // Contains all triangles with an emission of over 0 in the scene.

        std::unique_ptr<MeshWithColors> m_mesh;

        std::vector<AreaLight*> m_areaLights;
        float m_selectedLightIntensity;
        int m_selectedLightId;
        int m_lightColorRed;
        int m_lightColorGreen;
        int m_lightColorBlue;

        std::unique_ptr<PathTraceRenderer> m_pathtrace_renderer;

        int m_numBounces;
        float m_lightSize;
        Timer m_updateClock;

        bool m_RTMode;
        bool m_useRussianRoulette;

        // whether or not using normal maps
        bool m_normalMapped;

        // termination probability for Russian Roulette
        float m_terminationProb;

        // whether or not sampling light-emitting triangles
        bool m_enableEmittingTriangles;

        // whether or not change ray direction according to reflections and refractions rules
        bool m_enableReflectionsAndRefractions;

        // number of rays per pixel for Anti-Aliasing
        int m_AARaysNumber;

        // Gauss filter width
        float m_GaussFilterWidth;

        // if true - making perfect specular Reflections and Refractions
        bool experimental_bPureRef;

        // if true - accounting only diffuse part to throughput
        bool experimental_bOnlyDiffuseThroughput;

        bool clear_on_next_frame = false;
        Mat4f previous_camera = Mat4f(0);
        Image m_img;
        int m_numDebugPathCount = 1;
        int m_currentVisualizationIndex = 0;
        float m_visualizationAlpha = 1;
        bool m_playbackVisualization = false;
        bool m_clearVisualization = false;

        std::vector<PathVisualizationNode> m_visualization;
    };


    //------------------------------------------------------------------------
}
