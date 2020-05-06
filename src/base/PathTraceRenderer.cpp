#include "PathTraceRenderer.hpp"
#include "RayTracer.hpp"
#include "AreaLight.hpp"
#include "sobol.hpp"

#include <atomic>
#include <chrono>
#include <algorithm>
#include <string>


#define  Pr  .299
#define  Pg  .587
#define  Pb  .114


namespace FW
{
    bool PathTraceRenderer::m_normalMapped = false;
    bool PathTraceRenderer::debugVis = false;

    //  public-domain function by Darel Rex Finley
    //
    //  The passed-in RGB values can be on any desired scale, such as 0 to
    //  to 1, or 0 to 255.  (But use the same scale for all three!)
    //
    //  The "change" parameter works like this:
    //    0.0 creates a black-and-white image.
    //    0.5 reduces the color saturation by half.
    //    1.0 causes no change.
    //    2.0 doubles the color saturation.
    //  Note:  A "change" value greater than 1.0 may project your RGB values
    //  beyond their normal range, in which case you probably should truncate
    //  them to the desired range before trying to use them in an image.
    void PathTraceRenderer::changeSaturation(Vec4f& color, float change)
    {
        double P = FW::sqrt(
            color.x * color.x * Pr +
            color.y * color.y * Pg +
            color.z * color.z * Pb
        );

        color.x = P + (color.x - P) * change;
        color.y = P + (color.y - P) * change;
        color.z = P + (color.z - P) * change;

        color = FW::clamp(color, 0.f, 1.f);
    }

    void PathTraceRenderer::getTextureParameters(const RaycastResult& hit, Vec3f& diffuse, Vec3f& n, Vec3f& specular)
    {
        // YOUR CODE HERE (R1)
        // Read value from albedo texture into diffuse.
        // If textured, use the texture; if not, use Material.diffuse.
        // Note: You can probably reuse parts of the radiosity assignment.
        const auto mat = hit.tri->m_material;

        // fetch barycentric coordinates
        float alpha = hit.u;
        float beta = hit.v;

        Vec2f uv;

        // Diffuse part
        if (mat->textures[MeshBase::TextureType_Diffuse].exists())
        {
            // read diffuse texture
            const Texture& tex = mat->textures[MeshBase::TextureType_Diffuse];
            const Image& texImg = *tex.getImage();

            uv = (1.f - alpha - beta) * hit.tri->m_vertices[0].t +
                alpha * hit.tri->m_vertices[1].t +
                beta * hit.tri->m_vertices[2].t;

            diffuse = texImg.getVec4f(FW::getTexelCoords(uv, texImg.getSize())).getXYZ();

            // gamma correction 
            diffuse.x = powf(diffuse.x, 2.2f);
            diffuse.y = powf(diffuse.y, 2.2f);
            diffuse.z = powf(diffuse.z, 2.2f);
        }
        else
        {
            // no texture, use constant albedo from material structure
            diffuse = mat->diffuse.getXYZ();
        }

        // Specular part
        specular = mat->specular;

        if (mat->textures[MeshBase::TextureType_Specular].exists())
        {
            // read specular texture
            const Texture& tex = mat->textures[MeshBase::TextureType_Specular];
            const Image& texImg = *tex.getImage();

            Vec2i texelCoords = getTexelCoords(uv, texImg.getSize());

            specular = texImg.getVec4f(texelCoords).getXYZ();
        }

        // Vertex normals interpolation
        n = ((1.f - alpha - beta) * hit.tri->m_vertices[0].n +
            alpha * hit.tri->m_vertices[1].n +
            beta * hit.tri->m_vertices[2].n).normalized();

        // normal mapping
        if (m_normalMapped && mat->textures[MeshBase::TextureType_Normal].exists())
        {
            // read normal texture
            const Texture& tex = mat->textures[MeshBase::TextureType_Normal];
            const Image& texImg = *tex.getImage();

            Vec2i texelCoords = FW::getTexelCoords(uv, texImg.getSize());
            Vec3f tn = (2.f * texImg.getVec4f(texelCoords).getXYZ() - 1.f).normalized();

            Vec3f deltaP1 = hit.tri->m_vertices[1].p - hit.tri->m_vertices[0].p;
            Vec3f deltaP2 = hit.tri->m_vertices[2].p - hit.tri->m_vertices[0].p;

            Vec2f deltaUV1 = hit.tri->m_vertices[1].t - hit.tri->m_vertices[0].t;
            Vec2f deltaUV2 = hit.tri->m_vertices[2].t - hit.tri->m_vertices[0].t;

            float r = 1.f / (deltaUV1.x * deltaUV2.y - deltaUV1.y * deltaUV2.x);

            Vec3f tangent = (r * (deltaUV2.y * deltaP1 - deltaUV1.y * deltaP2)).normalized();
            Vec3f bitangent = -(r * (deltaUV1.x * deltaP2 - deltaUV2.x * deltaP1)).normalized();

            Mat3f TBN;
            TBN.setCol(0, tangent);
            TBN.setCol(1, bitangent);
            TBN.setCol(2, n);

            n = (TBN * tn).normalized();
        }
    }

    PathTracerContext::PathTracerContext()
        : m_bForceExit(false),
          m_bResidual(false),
          m_scene(nullptr),
          m_rt(nullptr),
          m_light(nullptr),
          m_pass(0),
          m_bounces(0),
          m_destImage(0),
          m_camera(nullptr)
    {
    }

    PathTracerContext::~PathTracerContext()
    {
    }

    PathTraceRenderer::PathTraceRenderer()
    {
        m_raysPerSecond = 0.0f;
    }

    PathTraceRenderer::~PathTraceRenderer()
    {
        stop();
    }

    // This function traces a single path and returns the resulting color value that will get rendered on the image. 
    // Filling in the blanks here is all you need to do this time around.
    Vec3f PathTraceRenderer::tracePath(float x, float y, PathTracerContext& ctx, int samplerBase, Random& R,
                                       std::vector<PathVisualizationNode>& visualization)
    {
        //const MeshWithColors* scene = ctx.m_scene;
        RayTracer* rt = ctx.m_rt;
        Image* image = ctx.m_image.get();
        const CameraControls& cameraCtrl = *ctx.m_camera;

        // make sure we're on CPU
        //image->getMutablePtr();

        // get camera orientation and projection
        Mat4f worldToCamera = cameraCtrl.getWorldToCamera();
        Mat4f projection = Mat4f::fitToView(Vec2f(-1.f, -1.f), Vec2f(2.f, 2.f), image->getSize()) *
            cameraCtrl.getCameraToClip();

        // inverse projection from clip space to world space
        Mat4f invP = (projection * worldToCamera).inverted();

        // point on front plane in homogeneous coordinates
        Vec4f P0(x, y, -1.f, 1.f);
        // point on back plane in homogeneous coordinates
        Vec4f P1(x, y, 1.f, 1.f);

        // apply inverse projection, divide by w to get object-space points
        Vec4f Roh = (invP * P0);
        Vec3f Ro = (Roh * (1.f / Roh.w)).getXYZ();
        Vec4f Rdh = (invP * P1);
        Vec3f Rd = (Rdh * (1.f / Rdh.w)).getXYZ();

        // Subtract front plane point from back plane point,
        // yields ray direction.
        // NOTE that it's not normalized; the direction Rd is defined
        // so that the segment to be traced is [Ro, Ro+Rd], i.e.,
        // intersections that come _after_ the point Ro+Rd are to be discarded.
        Rd = Rd - Ro;

        // YOUR CODE HERE (R2-R4):
        // Implement path tracing with direct light and shadows, scattering and Russian roulette.
        Vec3f Ei(0.f);
        Vec3f n(0.f);
        Vec3f throughput(1.f);
        Vec3f db;
        int bounce;

        for (bounce = 0; bounce < FW::abs(ctx.m_bounces) + 1; ++bounce)
        {
            RaycastResult result = rt->raycast(Ro + 0.001f * n, Rd);

            if (result.tri != nullptr)
            {
                if (bounce == 0)
                {
                    Rd = result.dir;

                    if (result.tri->m_material->emission.length() > 0.f)
                    {
                        Ei = result.tri->m_material->emission;
                    }
                }

                Ro = result.point;
                db = throughput;
                Ei += db * pathIteration(ctx, R, result, samplerBase, bounce, Rd, n, throughput);

                if (debugVis)
                {
                    // Example code for using the visualization system. You can expand this to include further bounces, 
                    // shadow rays, and whatever other useful information you can think of.
                    PathVisualizationNode node;
                    node.lines.push_back(PathVisualizationLine(result.orig, result.point));
                    // Draws a line between two points
                    node.lines.push_back(PathVisualizationLine(result.point, result.point + result.tri->normal() * .1f,
                                                               Vec3f(1, 0, 0)));
                    // You can give lines a color as optional parameter.
                    node.labels.push_back(PathVisualizationLabel(
                        "diffuse: " + std::to_string(Ei.x) + ", " + std::to_string(Ei.y) + ", " + std::to_string(
                            Ei.z),
                        result.point)); // You can also render text labels with world-space locations.

                    visualization.push_back(node);
                }
            }
            else
            {
                break;
            }
        }

        // Russian roulette
        if (ctx.m_bounces < 0)
        {
            // TODO: should be in ctx, so it is configurable via UI
            unsigned int rrBoost = 5;
            unsigned int probability = 100 / rrBoost;

            while (R.getU32() % 100 < probability)
            {
                RaycastResult result = rt->raycast(Ro + 0.001f * n, Rd);

                // if we hit something, fetch a color and insert into image
                if (result.tri != nullptr)
                {
                    Ro = result.point;
                    db = throughput;
                    Ei += (float)rrBoost * db * pathIteration(ctx, R, result, samplerBase, bounce, Rd, n, throughput);
                    //rrBoost = rrBoost * rrBoost; // should it be doubled because overall probability would decrease in twice?
                    ++bounce;
                }
                else
                {
                    break;
                }
            }
        }

        Ei *= 1.f / FW_PI;

        return Ei;
    }

    Vec3f PathTraceRenderer::pathIteration(PathTracerContext& ctx, Random& R, const RaycastResult result,
                                           int samplerBase, int dimension, Vec3f& Rd, Vec3f& n, Vec3f& throughput)
    {
        Vec3f diffuse;
        Vec3f specular;

        getTextureParameters(result, diffuse, n, specular);

        throughput *= diffuse;

        if (FW::dot(Rd, n) > 0.f)
        {
            n = -n; // flip normal
        }

        Vec3f Ei(0.f);

        float pdf;
        Vec3f Pl;
        Vec3f lightNormal;
        Vec3f lightEmission;

        chooseLightSample(ctx, samplerBase, dimension, pdf, Pl, lightNormal, lightEmission, R, result);

        // construct vector from current vertex (o) to light sample
        Vec3f o = result.point;
        Vec3f d = Pl - o;

        // trace shadow ray to see if it's blocked
        if (!ctx.m_rt->raycast(o + 0.001f * n, 0.998f * d))
        {
            // if not, add the appropriate emission, 1/r^2 and clamped cosine terms, accounting for the PDF as well
            Vec3f unionD = d.normalized();
            float cosThetaL = FW::clamp(FW::dot(unionD, -lightNormal), 0.f, 1.f);
            float cosTheta = FW::clamp(FW::dot(unionD, n), 0.f, 1.f);
            float distance = d.length();

            Vec3f color = diffuse;

            // glossy reflection (Blinn-Phong)
            if (specular.length() > 0.f)
            {
                float glossiness = result.tri->m_material->glossiness;

                // normalization factor for Blinn-Phong model
                float normFactor = (glossiness + 2.f) / (4.f * FW_PI * (2.f - pow(2.f, -glossiness / 2.f)));

                // Calculate the half vector between the light vector and the view vector
                Vec3f H = (unionD - Rd.normalized()).normalized();

                // Intensity of the specular light
                float nDotH = FW::dot(n, H);

                // adding specular part to diffuse one
                color += FW::pow(FW::max(0.f, nDotH), glossiness) * normFactor * specular;
            }

            Ei += cosThetaL * cosTheta / FW::max(pdf * distance * distance, 1e-7f) * lightEmission * color;
        }

        // cosine weighted direction
        Vec2f rv;
        int r = R.getS32(1, 10000);
        rv.x = 2.f * sobol::sample(samplerBase + r, 4 * dimension + 2) - 1.f;
        rv.y = 2.f * sobol::sample(samplerBase + r, 4 * dimension + 3) - 1.f;

        float sqrtX = FW::sqrt(rv.x);
        float theta = 2.f * FW_PI * rv.y;
        Vec3f cwd = formBasis(n) * Vec3f(sqrtX * FW::cos(theta),
                                         sqrtX * FW::sin(theta),
                                         FW::sqrt(FW::max(0.f, 1.f - rv.x)));

        // setting new Rd
        Rd = (*ctx.m_camera).getFar() * cwd;

        return Ei;
    }

    void PathTraceRenderer::chooseLightSample(PathTracerContext& ctx, int samplerBase, int dimension, float& pdf,
                                              Vec3f& Pl, Vec3f& lightNormal, Vec3f& lightEmission, Random& R,
                                              const RaycastResult result)
    {
        AreaLight* light = ctx.m_light;
        std::vector<RTTriangle*> lightTriangles = ctx.m_lightTriangles;

        light->sample(pdf, Pl, samplerBase, dimension, R);

        float largestEmissivePower = (
                light->getSize().x * light->getSize().y * 4 * light->getEmission() /
                FW::pow((Pl - result.point).length(), 2)
            )
            .length();

        auto lightId = lightTriangles.size();

        for (size_t i = 0; i < lightTriangles.size(); ++i)
        {
            sampleEmissionTriangle(pdf, Pl, R, lightTriangles[i]);

            float emissivePower = (
                    lightTriangles[i]->area() * lightTriangles[i]->m_material->emission /
                    FW::pow((Pl - result.point).length(), 2)
                )
                .length();

            if (emissivePower > largestEmissivePower)
            {
                lightId = i;
            }
        }

        if (lightId == lightTriangles.size())
        {
            light->sample(pdf, Pl, 0, R);

            lightNormal = light->getNormal();
            lightEmission = light->getEmission();
        }
        else
        {
            RTTriangle* lightTri = lightTriangles[lightId];
            MeshBase::Material* emissiveMat = lightTri->m_material;

            sampleEmissionTriangle(pdf, Pl, R, lightTri);

            lightNormal = lightTri->normal();
            lightEmission = emissiveMat->emission;
        }
    }

    void PathTraceRenderer::chooseRandomLightSample(PathTracerContext& ctx, float& pdf, Vec3f& Pl, Vec3f& lightNormal,
                                                    Vec3f& lightEmission, Random& R)
    {
        AreaLight* light = ctx.m_light;
        std::vector<RTTriangle*> lightTriangles = ctx.m_lightTriangles;

        size_t lightId = R.getU64(0, lightTriangles.size());

        if (lightId == lightTriangles.size())
        {
            light->sample(pdf, Pl, 0, R);

            lightNormal = light->getNormal();
            lightEmission = light->getEmission();
        }
        else
        {
            RTTriangle* lightTri = lightTriangles[lightId];
            MeshBase::Material* emissiveMat = lightTri->m_material;

            sampleEmissionTriangle(pdf, Pl, R, lightTri);

            lightNormal = lightTri->normal();
            lightEmission = emissiveMat->emission;
        }
    }

    void PathTraceRenderer::sampleEmissionTriangle(float& pdf, Vec3f& p, Random& rnd, RTTriangle* tri)
    {
        float alpha;
        float beta;

        do
        {
            alpha = rnd.getF32(0.f, 1.f);
            beta = rnd.getF32(0.f, 1.f);
        }
        while (alpha + beta > 1.f);

        p = tri->centroid() +
            (1.f - alpha - beta) * (tri->m_vertices[0].p - tri->centroid()) +
            alpha * (tri->m_vertices[1].p - tri->centroid()) +
            beta * (tri->m_vertices[2].p - tri->centroid());

        pdf = 1.f / tri->area();
    }

    float PathTraceRenderer::filterGauss(float x, float y)
    {
        float sigma = 1.f;
        float s = 2.f * sigma * sigma;

        return 1.f / FW::sqrt(FW_PI * s) * FW::exp(-1.f * (x * x + y * y) / s);
    }

    // This function is responsible for asynchronously generating paths for a given block.
    void PathTraceRenderer::pathTraceBlock(MulticoreLauncher::Task& t)
    {
        PathTracerContext& ctx = *(PathTracerContext*)t.data;

        Image* image = ctx.m_image.get();
        const CameraControls& cameraCtrl = *ctx.m_camera;

        // make sure we're on CPU
        image->getMutablePtr();

        auto width = image->getSize().x;
        auto height = image->getSize().y;

        // TODO: should be in ctx, so it is configurable via UI
        float filterWidth = 1.f;
        float saturationChange = 1.f;

        float filterScale = 2.f / filterWidth;

        // get camera orientation and projection
        Mat4f worldToCamera = cameraCtrl.getWorldToCamera();
        Mat4f projection = Mat4f::fitToView(Vec2f(-1, -1), Vec2f(2, 2), image->getSize()) * cameraCtrl.
            getCameraToClip();

        // inverse projection from clip space to world space
        Mat4f invP = (projection * worldToCamera).inverted();

        // get the block which we are rendering
        PathTracerBlock& block = ctx.m_blocks[t.idx];

        // Not used but must be passed to tracePath
        std::vector<PathVisualizationNode> dummyVisualization;

        static std::atomic<uint32_t> seed = 0;
        uint32_t current_seed = seed.fetch_add(1);
        Random R(t.idx + current_seed); // this is bogus, just to make the random numbers change each iteration
        int base = t.idx * R.getS32(1, 9) + R.getS32(1, 10000);

        for (int i = 0; i < block.m_width * block.m_height; ++i)
        {
            if (ctx.m_bForceExit)
            {
                return;
            }

            float rs1 = filterWidth * (sobol::sample(base + i, 0) - 0.5f) + 0.5f;
            float rs2 = filterWidth * (sobol::sample(base + i, 1) - 0.5f) + 0.5f;
            int pixel_x = block.m_x + (i % block.m_width);
            int pixel_y = block.m_y + (i / block.m_width);
            float xr = (float)pixel_x + rs1;
            float yr = (float)pixel_y + rs2;

            if (xr < 0.f)
            {
                xr *= -1.f;
            }
            if (xr > width)
            {
                xr -= rs1;
            }

            if (yr < 0.f)
            {
                yr *= -1.f;
            }
            if (yr > height)
            {
                yr -= rs2;
            }

            float x = 2.f * xr / width - 1.f;
            float y = -2.f * yr / height + 1.f;

            Vec3f Ei = tracePath(x, y, ctx, base + i, R, dummyVisualization);
            Vec4f finalColor = filterGauss(filterScale * (rs1 - 0.5f), filterScale * (rs2 - 0.5f)) * Vec4f(Ei, 1.f);

            changeSaturation(finalColor, saturationChange);

            // Put pixel.
            Vec4f prev = image->getVec4f(Vec2i(pixel_x, pixel_y));
            prev += finalColor;
            image->setVec4f(Vec2i(pixel_x, pixel_y), prev);
        }
    }

    void PathTraceRenderer::startPathTracingProcess(const MeshWithColors* scene, AreaLight* light, RayTracer* rt,
                                                    Image* dest, int bounces, const CameraControls& camera,
                                                    const std::vector<RTTriangle*> lightTriangles)
    {
        FW_ASSERT(!m_context.m_bForceExit);

        m_context.m_bForceExit = false;
        m_context.m_bResidual = false;
        m_context.m_camera = &camera;
        m_context.m_rt = rt;
        m_context.m_scene = scene;
        m_context.m_light = light;
        m_context.m_pass = 0;
        m_context.m_bounces = bounces;
        m_context.m_image.reset(new Image(dest->getSize(), ImageFormat::RGBA_Vec4f));

        m_context.m_destImage = dest;
        m_context.m_image->clear();

        m_context.m_lightTriangles = lightTriangles;

        // Add rendering blocks.
        m_context.m_blocks.clear();
        {
            int block_size = 32;
            int image_width = dest->getSize().x;
            int image_height = dest->getSize().y;
            int block_count_x = (image_width + block_size - 1) / block_size;
            int block_count_y = (image_height + block_size - 1) / block_size;

            for (int y = 0; y < block_count_y; ++y)
            {
                int block_start_y = y * block_size;
                int block_end_y = FW::min(block_start_y + block_size, image_height);
                int block_height = block_end_y - block_start_y;

                for (int x = 0; x < block_count_x; ++x)
                {
                    int block_start_x = x * block_size;
                    int block_end_x = FW::min(block_start_x + block_size, image_width);
                    int block_width = block_end_x - block_start_x;

                    PathTracerBlock block;
                    block.m_x = block_size * x;
                    block.m_y = block_size * y;
                    block.m_width = block_width;
                    block.m_height = block_height;

                    m_context.m_blocks.push_back(block);
                }
            }
        }

        dest->clear();

        // Fire away!

        // If you change this, change the one in checkFinish too.
        m_launcher.setNumThreads(m_launcher.getNumCores());
        //m_launcher.setNumThreads(1);

        m_launcher.popAll();
        m_launcher.push(pathTraceBlock, &m_context, 0, (int)m_context.m_blocks.size());
    }

    void PathTraceRenderer::updatePicture(Image* dest)
    {
        FW_ASSERT(m_context.m_image != 0);
        FW_ASSERT(m_context.m_image->getSize() == dest->getSize());

        for (int i = 0; i < dest->getSize().y; ++i)
        {
            for (int j = 0; j < dest->getSize().x; ++j)
            {
                Vec4f D = m_context.m_image->getVec4f(Vec2i(j, i));
                if (D.w != 0.0f)
                    D = D * (1.0f / D.w);

                // Gamma correction.
                Vec4f color = Vec4f(
                    FW::pow(D.x, 1.0f / 2.2f),
                    FW::pow(D.y, 1.0f / 2.2f),
                    FW::pow(D.z, 1.0f / 2.2f),
                    D.w
                );

                dest->setVec4f(Vec2i(j, i), color);
            }
        }
    }

    void PathTraceRenderer::checkFinish()
    {
        // have all the vertices from current bounce finished computing?
        if (m_launcher.getNumTasks() == m_launcher.getNumFinished())
        {
            // yes, remove from task list
            m_launcher.popAll();

            ++m_context.m_pass;

            // you may want to uncomment this to write out a sequence of PNG images
            // after the completion of each full round through the image.
            //String fn = sprintf( "pt-%03dppp.png", m_context.m_pass );
            //File outfile( fn, File::Create );
            //exportLodePngImage( outfile, m_context.m_destImage );

            if (!m_context.m_bForceExit)
            {
                // keep going

                // If you change this, change the one in startPathTracingProcess too.
                m_launcher.setNumThreads(m_launcher.getNumCores());
                //m_launcher.setNumThreads(1);

                m_launcher.popAll();
                m_launcher.push(pathTraceBlock, &m_context, 0, (int)m_context.m_blocks.size());
                //::printf( "Next pass!" );
            }
            else ::printf("Stopped.");
        }
    }

    void PathTraceRenderer::stop()
    {
        m_context.m_bForceExit = true;

        if (isRunning())
        {
            m_context.m_bForceExit = true;
            while (m_launcher.getNumTasks() > m_launcher.getNumFinished())
            {
                Sleep(1);
            }
            m_launcher.popAll();
        }

        m_context.m_bForceExit = false;
    }
} // namespace FW