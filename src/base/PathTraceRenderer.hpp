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

        AreaLight* m_light;

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
        void startPathTracingProcess(const MeshWithColors* scene, AreaLight*, RayTracer* rt, Image* dest, int bounces,
                                     const CameraControls& camera, const std::vector<RTTriangle*> lightTriangles);

        static Vec3f tracePath(float x, float y, PathTracerContext& ctx, int samplerBase, Random& rnd,
                               std::vector<PathVisualizationNode>& visualization);

        static Vec3f pathIteration(PathTracerContext& ctx, Random& R, const RaycastResult result, int samplerBase,
                                   int bounce, Vec3f& Rd, Vec3f& n, Vec3f& throughput);

        static void chooseLightSample(PathTracerContext& ctx, int samplerBase, int bounce, float& pdf, Vec3f& Pl,
                                      Vec3f& lightNormal, Vec3f& lightEmission, Random& R, const RaycastResult result);

        static void chooseRandomLightSample(PathTracerContext& ctx, float& pdf, Vec3f& Pl, Vec3f& lightNormal,
                                            Vec3f& lightEmission, Random& R);

        static void sampleEmissionTriangle(float& pdf, Vec3f& p, Random& rnd, RTTriangle* tri);

        static float filterGauss(float x, float y);

        static void pathTraceBlock(MulticoreLauncher::Task& t);

        static void getTextureParameters(const RaycastResult& hit, Vec3f& diffuse, Vec3f& n, Vec3f& specular);

        void updatePicture(Image* display); // normalize by 1/w

        void checkFinish(void);

        void stop(void);

        void setNormalMapped(bool b)
        {
            m_normalMapped = b;
        }

    protected:

        __int64 m_s64TotalRays;

        float m_raysPerSecond;

        MulticoreLauncher m_launcher;

        static bool m_normalMapped;
    };
} // namespace FW
