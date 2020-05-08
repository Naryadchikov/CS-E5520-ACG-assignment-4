#pragma once


#include "3d/CameraControls.hpp"
#include "3d/Mesh.hpp"
#include "base/Random.hpp"
#include "base/MulticoreLauncher.hpp"

#include <vector>
#include <memory>
#include <string>


namespace FW
{
    //------------------------------------------------------------------------
    typedef Mesh<VertexPNTC> MeshWithColors;

    //------------------------------------------------------------------------
    class RayTracer;
    struct RaycastResult;
    struct RTTriangle;
    class Image;
    class AreaLight;

    /// Defines a block which is rendered by a single thread as a single task.
    struct PathTracerBlock
    {
        int m_x; ///< X coordinate of the leftmost pixel of the block.
        int m_y; ///< Y coordinate of the topmost pixel of the block.
        int m_width; ///< Pixel width of the block.
        int m_height; ///< Pixel height of the block.
    };

    struct PathTracerContext
    {
        PathTracerContext();

        ~PathTracerContext();

        std::vector<PathTracerBlock> m_blocks; ///< Render blocks for rendering tasks. Index by .idx.

        bool m_bForceExit;

        bool m_bResidual;

        const MeshWithColors* m_scene;

        RayTracer* m_rt;

        std::vector<AreaLight*> m_areaLights;

        int m_pass; ///< Pass number, increased by one for each full render iteration.

        int m_bounces;

        std::unique_ptr<Image> m_image;

        Image* m_destImage;

        const CameraControls* m_camera;

        std::vector<RTTriangle*> m_lightTriangles; // Contains all triangles with an emission of over 0 in the scene.
    };

    class PathVisualizationLine
    {
    public:

        Vec3f start, stop, color;

        PathVisualizationLine(const Vec3f& start, const Vec3f& stop, const Vec3f color = Vec3f(1)) : start(start),
                                                                                                     stop(stop),
                                                                                                     color(color)
        {
        }
    };

    class PathVisualizationLabel
    {
    public:

        std::string text;

        Vec3f position;

        PathVisualizationLabel(std::string text, Vec3f position) : text(text), position(position)
        {
        }
    };

    class PathVisualizationNode
    {
    public:

        std::vector<PathVisualizationLabel> labels;

        std::vector<PathVisualizationLine> lines;
    };

    // This class contains functionality to render pictures using a ray tracer.
    class PathTraceRenderer
    {
    public:

        PathTraceRenderer();

        ~PathTraceRenderer();

        // whether or not visualization data should be generated
        static bool debugVis;

        PathTracerContext m_context;

        // are we still processing?
        bool isRunning(void) const { return m_launcher.getNumTasks() > 0; }

        // negative #bounces = -N means start russian roulette from Nth bounce
        // positive N means always trace up to N bounces
        void startPathTracingProcess(const MeshWithColors* scene, const std::vector<AreaLight*>& areaLights,
                                     RayTracer* rt, Image* dest, int bounces, const CameraControls& camera,
                                     const std::vector<RTTriangle*>& lightTriangles);

        static Vec3f tracePath(float x, float y, PathTracerContext& ctx, int samplerBase, Random& rnd,
                               std::vector<PathVisualizationNode>& visualization);

        static Vec3f pathIteration(PathTracerContext& ctx, Random& R, const RaycastResult result, int samplerBase,
                                   int bounce, Vec3f& Rd, Vec3f& n, Vec3f& throughput);

        static void chooseAndSampleLight(PathTracerContext& ctx, int samplerBase, int bounce, float& pdf, Vec3f& Pl,
                                         Vec3f& lightNormal, Vec3f& lightEmission, Random& R,
                                         const RaycastResult& result);

        static void sampleEmissionTriangle(float& pdf, Vec3f& p, Random& rnd, RTTriangle* tri);

        static float filterGauss(float x, float y);

        static void pathTraceBlock(MulticoreLauncher::Task& t);

        static void getTextureParameters(const RaycastResult& hit, Vec3f& diffuse, Vec3f& n, Vec3f& specular);

        void updatePicture(Image* display); // normalize by 1/w

        void checkFinish(void);

        void stop(void);

        void setNormalMapped(bool b) { m_normalMapped = b; }

        void setTerminationProb(float value) { m_terminationProb = FW::clamp(value, 0.f, 1.f); }

        void setEnableEmittingTriangles(bool b) { m_enableEmittingTriangles = b; }

        void setAARaysNumber(int value) { m_AARaysNumber = value; }

        void setGaussFilterWidth(float value) { m_GaussFilterWidth = value; }

    protected:

        __int64 m_s64TotalRays;

        float m_raysPerSecond;

        MulticoreLauncher m_launcher;

        // whether or not using normal maps
        static bool m_normalMapped;

        // termination probability for Russian Roulette
        static float m_terminationProb;

        // whether or not sampling light-emitting triangles
        static bool m_enableEmittingTriangles;

        // number of rays per pixel for Anti-Aliasing
        static int m_AARaysNumber;

        // Gauss filter width
        static float m_GaussFilterWidth;
    };
} // namespace FW
