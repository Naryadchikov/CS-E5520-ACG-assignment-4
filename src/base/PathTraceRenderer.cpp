#include "PathTraceRenderer.hpp"
#include "RayTracer.hpp"
#include "AreaLight.hpp"
#include "sobol.hpp"

#include <atomic>
#include <chrono>
#include <algorithm>
#include <string>


namespace FW
{
    bool PathTraceRenderer::m_normalMapped = false;

    bool PathTraceRenderer::debugVis = false;

    float PathTraceRenderer::m_terminationProb = 0.2f;

    bool PathTraceRenderer::m_enableEmittingTriangles = false;

    bool PathTraceRenderer::m_enableReflectionsAndRefractions = false;

    int PathTraceRenderer::m_AARaysNumber = 4;

    float PathTraceRenderer::m_GaussFilterWidth = 1.f;

    bool PathTraceRenderer::bIsV1Active = true;

    // Shouldn't exist...
    bool PathTraceRenderer::bMagicButton = false;

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
        Vec3f T;
        int bounce;

        for (bounce = 0; bounce < FW::abs(ctx.m_bounces) + 1; ++bounce)
        {
            RaycastResult result = rt->raycast(Ro + 0.001f * n, Rd);

            if (result.tri != nullptr)
            {
                if (m_enableEmittingTriangles && bounce == 0 && result.tri->m_material->emission.length() > 0.f)
                {
                    Ei += result.tri->m_material->emission;
                }

                T = throughput;

                if (bIsV1Active)
                {
                    Ei += T * pathIteration(ctx, R, result, samplerBase, bounce, Rd, n, throughput,
                                            visualization);
                }
                else
                {
                    Ei += T * pathIterationV2(ctx, R, result, samplerBase, bounce, Rd, n, throughput, visualization);
                }

                // Update ray origin for next iteration
                Ro = result.point;

                if (debugVis)
                {
                    PathVisualizationNode node;

                    // ray
                    node.lines.push_back(PathVisualizationLine(result.orig, result.point));

                    // hit normal
                    node.lines.push_back(PathVisualizationLine(result.point, result.point + result.tri->normal() * .1f,
                                                               Vec3f(1, 0, 0)));

                    node.labels.push_back(PathVisualizationLabel(
                        "Ei: " + std::to_string(Ei.x) + ", " + std::to_string(Ei.y) + ", " +
                        std::to_string(Ei.z), result.point));

                    visualization.push_back(node);
                }
            }
            else
            {
                break;
            }
        }

        // Russian Roulette
        if (ctx.m_bounces < 0)
        {
            while (m_terminationProb < R.getF32(0.f, 1.f))
            {
                RaycastResult result = rt->raycast(Ro + 0.001f * n, Rd);

                // if we hit something, fetch a color and insert into image
                if (result.tri != nullptr)
                {
                    T = throughput;

                    if (bIsV1Active)
                    {
                        Ei += T * pathIteration(ctx, R, result, samplerBase, bounce, Rd, n, throughput, visualization) /
                            (1.f - m_terminationProb);
                    }
                    else
                    {
                        Ei += T * pathIterationV2(ctx, R, result, samplerBase, bounce, Rd, n, throughput,
                                                  visualization) /
                            (1.f - m_terminationProb);
                    }

                    // Update ray origin for next iteration
                    Ro = result.point;

                    ++bounce;

                    if (debugVis)
                    {
                        PathVisualizationNode node;

                        // ray
                        node.lines.push_back(PathVisualizationLine(result.orig, result.point, Vec3f(0, 0, 1)));

                        // hit normal
                        node.lines.push_back(PathVisualizationLine(result.point,
                                                                   result.point + result.tri->normal() * .1f,
                                                                   Vec3f(1, 0, 0)));

                        node.labels.push_back(PathVisualizationLabel(
                            "Ei: " + std::to_string(Ei.x) + ", " + std::to_string(Ei.y) + ", " +
                            std::to_string(Ei.z), result.point));

                        visualization.push_back(node);
                    }
                }
                else
                {
                    break;
                }
            }
        }

        return Ei;
    }

    Vec3f PathTraceRenderer::pathIteration(PathTracerContext& ctx, Random& R, const RaycastResult result,
                                           int samplerBase, int bounce, Vec3f& Rd, Vec3f& n, Vec3f& throughput,
                                           std::vector<PathVisualizationNode>& visualization)
    {
        Vec3f diffuse;
        Vec3f specular;

        getTextureParameters(result, diffuse, n, specular);

        Vec3f diffuseBRDF = diffuse / FW_PI; // Lambertian diffuse BRDF, it is albedo/pi
        Vec3f specularBRDF(0.f); // will update it later

        Vec3f rd = Rd.normalized(); // normalized ray direction
        float nDotR = FW::dot(rd, n);
        float ri = result.tri->m_material->indexOfRefraction;

        if (nDotR > 0.f)
        {
            n = -n; // flip normal

            if (m_enableReflectionsAndRefractions)
            {
                // we're inside the medium
                ri = ri != 0.f
                         ? 1.f / ri
                         : 0.f;
            }
        }

        if (m_enableReflectionsAndRefractions)
        {
            ri = ri != 0.f
                     ? 1.f / ri
                     : 0.f;
        }

        Vec3f Ei(0.f);
        float pdf_light;
        Vec3f Pl;
        Vec3f lightNormal;
        Vec3f lightEmission;

        chooseAndSampleLight(ctx, samplerBase, bounce, pdf_light, Pl, lightNormal, lightEmission, R, result);

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

            // glossy reflection (Blinn-Phong)
            if (specular.length() > 0.f)
            {
                float glossiness = result.tri->m_material->glossiness;

                // normalization factor for Blinn-Phong model
                float normFactor = (glossiness + 2.f) / (4.f * FW_PI * (2.f - pow(2.f, -glossiness / 2.f)));

                // Calculate the half vector between the light vector and the view vector
                Vec3f H = (unionD - rd).normalized();

                // Intensity of the specular light
                float nDotH = FW::dot(n, H);

                specularBRDF = FW::pow(FW::max(0.f, nDotH), glossiness) * normFactor * specular;
            }

            Ei = cosThetaL * cosTheta / FW::max(pdf_light * distance * distance, 1e-7f) * lightEmission *
                (diffuseBRDF + specularBRDF);

            if (debugVis)
            {
                PathVisualizationNode node;

                // shadow ray
                node.lines.push_back(PathVisualizationLine(o, Pl, Vec3f(1, 1, 0)));

                // light normal
                node.lines.push_back(PathVisualizationLine(Pl, Pl + lightNormal * .1f,
                                                           Vec3f(1, 0, 0)));

                visualization.push_back(node);
            }
        }

        /* Choosing new ray direction for next iteration */
        // ray direction for new bounce
        Vec3f newRayDirection;

        // probability of choosing that new ray
        float pdf_newRayDirection = 1.f;

        // dot product of hit normal and new ray direction
        float nDotNewRd;

        // Diffuse BRDF case
        if (!m_enableReflectionsAndRefractions || specular.length() == 0.f)
        {
            // Update ray direction for next iteration
            newRayDirection = formBasis(n) * sampleCosineWeightedDirection(samplerBase, bounce, R);

            nDotNewRd = FW::dot(n, newRayDirection);

            pdf_newRayDirection = FW::abs(nDotNewRd) / FW_PI;
        }
        else // Perfect specular Reflections and Refractions
        {
            float r0 = (1.f - ri) / (1.f + ri);
            r0 *= r0;

            float cost1 = -nDotR; // cosine of theta_1
            float cost2 = 1.f - ri * ri * (1.f - cost1 * cost1); // cosine of theta_2
            float fresnel = r0 + (1.f - r0) * FW::pow(1.f - cost1, 5.f); // Schlick-approximation

            float randProb = R.getF32();

            if (cost2 > 0 && randProb > fresnel)
            {
                // refraction direction
                newRayDirection = (ri * rd + (ri * cost1 - FW::sqrt(cost2)) * n).normalized();
            }
            else
            {
                // reflection direction
                newRayDirection = (rd + 2 * FW::abs(nDotR) * n).normalized();
            }

            Ei *= 1.15f;

            nDotNewRd = FW::dot(n, newRayDirection);

            pdf_newRayDirection = 1.f;
        }

        // throughput for next iterations
        if (m_enableReflectionsAndRefractions)
        {
            throughput *= FW::abs(nDotNewRd) * (diffuseBRDF + specularBRDF) / pdf_newRayDirection;
        }
        else
        {
            throughput *= diffuse; // same as throughput *= FW::abs(nDotNewRd) * diffuseBRDF / pdf_newRayDirection
        }

        // Update ray direction for next iteration
        Rd = (*ctx.m_camera).getFar() * newRayDirection;

        return Ei;
    }

    // Do not know why it is not working!
    Vec3f PathTraceRenderer::pathIterationV2(PathTracerContext& ctx, Random& R, const RaycastResult result,
                                             int samplerBase, int bounce, Vec3f& Rd, Vec3f& n, Vec3f& throughput,
                                             std::vector<PathVisualizationNode>& visualization)
    {
        Vec3f rd = Rd.normalized(); // normalized ray direction

        /* Getting material parameters */
        Vec3f diffuse;
        Vec3f specular;

        getTextureParameters(result, diffuse, n, specular);

        float nDotR = FW::dot(rd, n);

        if (nDotR > 0.f)
        {
            n = -n; // flip normal
        }

        // the specular exponent (higher values give a sharper specular reflection)
        float glossiness = result.tri->m_material->glossiness;

        /* Choosing new ray direction for next iteration */
        // Pure mirror reflection direction
        Vec3f mirrorReflectionDirection = (rd + 2 * FW::abs(nDotR) * n).normalized();

        // Ray direction for new bounce
        Vec3f newRayDirection;

        // Dot product of hit normal and new ray direction
        float nDotNewRd;

        // probability of choosing that new ray
        float pdf_newRayDirection;

        // Randomly select whether we’ll compute a diffuse sample or a specular sample
        float u = R.getF32();
        float k_d = (diffuse.x + diffuse.y + diffuse.z) / 3.f;
        float k_s = (specular.x + specular.y + specular.z) / 3.f;
        //float k_d_prob = k_d / (k_d + k_s);
        float k_d_prob = 1;
        float k_s_prob = k_s / (k_d + k_s);

        // take a diffuse sample and compute it's contribution
        if (k_d_prob > u)
        {
            newRayDirection = formBasis(n) * sampleCosineWeightedDirection(samplerBase, bounce, R);

            nDotNewRd = FW::dot(n, newRayDirection);

            pdf_newRayDirection = FW::abs(nDotNewRd) / FW_PI;

            pdf_newRayDirection /= k_d_prob;
        }
        else // take a specular sample and compute it's contribution (no absorption probability)
        {
            newRayDirection = formBasis(mirrorReflectionDirection) * samplePhongReflectionDirection(
                glossiness, samplerBase, bounce, R);

            nDotNewRd = FW::dot(n, newRayDirection);

            float cosAlpha = FW::dot(mirrorReflectionDirection, newRayDirection);

            float pdf_phong = (glossiness + 1.f) / (2.f * FW_PI) * FW::pow(FW::max(cosAlpha, 0.f), glossiness);

            pdf_newRayDirection = pdf_phong;

            pdf_newRayDirection /= k_s_prob;
        }

        // Lambertian diffuse BRDF, it is albedo/pi
        Vec3f diffuseBRDF = diffuse / FW_PI;

        // Specular BRDF Phong model
        float cosAlpha = FW::dot(mirrorReflectionDirection, newRayDirection);
        Vec3f specularBRDF = (glossiness + 2.f) / (2.f * FW_PI) * FW::pow(FW::max(cosAlpha, 0.f), glossiness) *
            specular;

        /* Light sampling */
        Vec3f Ei(0.f);
        float pdf_light;
        Vec3f Pl;
        Vec3f lightNormal;
        Vec3f lightEmission;

        chooseAndSampleLight(ctx, samplerBase, bounce, pdf_light, Pl, lightNormal, lightEmission, R, result);

        // construct vector from current vertex (o) to light sample
        Vec3f hitOrigin = result.point;
        Vec3f lightDirection = Pl - hitOrigin;

        // trace shadow ray to see if it's blocked
        if (!ctx.m_rt->raycast(hitOrigin + 0.001f * n, 0.998f * lightDirection))
        {
            // if not, add the appropriate emission, 1/r^2 and clamped cosine terms, accounting for the PDF as well
            Vec3f unionLightDirection = lightDirection.normalized();
            float cosThetaL = FW::clamp(FW::dot(unionLightDirection, -lightNormal), 0.f, 1.f);
            float cosTheta = FW::clamp(FW::dot(unionLightDirection, n), 0.f, 1.f);
            float distance = lightDirection.length();

            Ei = cosThetaL * cosTheta / FW::max(pdf_light * distance * distance, 1e-7f) * lightEmission *
                (diffuseBRDF + specularBRDF);

            if (debugVis)
            {
                PathVisualizationNode node;

                // shadow ray
                node.lines.push_back(PathVisualizationLine(hitOrigin, Pl, Vec3f(1, 1, 0)));

                // light normal
                node.lines.push_back(PathVisualizationLine(Pl, Pl + lightNormal * .1f,
                                                           Vec3f(1, 0, 0)));

                visualization.push_back(node);
            }
        }

        // throughput for next iterations
        throughput *= FW::abs(nDotNewRd) * (diffuseBRDF + specularBRDF) / pdf_newRayDirection;

        // Update ray direction for next iteration
        Rd = (*ctx.m_camera).getFar() * newRayDirection;

        return Ei;
    }

    void PathTraceRenderer::chooseAndSampleLight(PathTracerContext& ctx, int samplerBase, int bounce, float& pdf,
                                                 Vec3f& Pl, Vec3f& lightNormal, Vec3f& lightEmission, Random& R,
                                                 const RaycastResult& result)
    {
        AreaLight* lightToSample = ctx.m_areaLights[0];
        float largestEmissivePower = 0.f;

        // choosing area light with highest emissive power
        for (auto light : ctx.m_areaLights)
        {
            lightToSample->sample(pdf, Pl, samplerBase, bounce, R);

            float emissivePower = (
                    light->getSize().x * light->getSize().y * 4 * light->getEmission() /
                    FW::pow((Pl - result.point).length(), 2)
                )
                .length();

            if (emissivePower > largestEmissivePower)
            {
                largestEmissivePower = emissivePower;
                lightToSample = light;
            }
        }

        std::vector<RTTriangle*> lightTriangles = ctx.m_lightTriangles;
        auto lightId = lightTriangles.size();

        // choosing light with highest emissive power
        // from all light triangles and previously selected area light
        if (m_enableEmittingTriangles)
        {
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
        }

        // sampling selected light
        if (lightId == lightTriangles.size())
        {
            lightToSample->sample(pdf, Pl, samplerBase, bounce, R);

            lightNormal = lightToSample->getNormal();
            lightEmission = lightToSample->getEmission();
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

    // 1st bounce draws from 3rd and 4th dimensions
    // 2nd bounce gets dimensions 5th and 6th
    // and so on
    Vec3f PathTraceRenderer::sampleCosineWeightedDirection(int samplerBase, int bounce, Random R)
    {
        int rnd = R.getU32(1, 10000);

        // variables for low discrepancy sampling with Sobol sequence
        float rs1;
        float rs2;

        // No magic here :(
        if (!bMagicButton)
        {
            // Normal case scenario - getting values from 0 to 1
            rs1 = sobol::sample(samplerBase + rnd, bounce + 2);
            rs2 = sobol::sample(samplerBase + rnd, bounce + 3);
        }
            // Magic is happening here! Accidentally discovered it by mistake...
            // Boosting speed in several times, giving more pleasant result with almost no noise...
        else
        {
            // Weird case scenario - getting values from -1 to 1
            // So if we get values from [-1 to 0) we are going to take square root from negative number next...
            rs1 = 2.f * sobol::sample(samplerBase + rnd, bounce + 2) - 1.f;
            rs2 = 2.f * sobol::sample(samplerBase + rnd, bounce + 3) - 1.f;
        }

        // If we are using MagicButton, it could be NaN, if we get negative 'rs1' value.
        // That means that we will terminate next iteration in path sequence.
        // That termination explains better speed performance, but why is the result picture itself better?
        float r = FW::sqrt(rs1);

        float theta = 2.f * FW_PI * rs2;

        return Vec3f(r * cos(theta),
                     r * sin(theta),
                     FW::sqrt(FW::max(0.f, 1.f - rs1)));
    }

    // 1st bounce draws from 3rd and 4th dimensions
    // 2nd bounce gets dimensions 5th and 6th
    // and so on
    Vec3f PathTraceRenderer::samplePhongReflectionDirection(float glossiness, int samplerBase, int bounce, Random R)
    {
        int rnd = R.getU32(1, 10000);

        // variables for low discrepancy sampling with Sobol sequence
        float rs1 = sobol::sample(samplerBase + rnd, bounce + 2);
        float rs2 = sobol::sample(samplerBase + rnd, bounce + 3);

        float r = FW::pow(rs1, 1.f / (glossiness + 1.f));
        float theta = 2.f * FW_PI * rs2;

        return Vec3f(r * cos(theta),
                     r * sin(theta),
                     FW::sqrt(FW::max(0.f, 1.f - rs1)));
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
        int base = t.idx * R.getU32(1, 9) + R.getU32(1, 10000);

        for (int i = 0; i < block.m_width * block.m_height; ++i)
        {
            if (ctx.m_bForceExit)
            {
                return;
            }

            Vec4f color;

            int pixel_x = block.m_x + (i % block.m_width);
            int pixel_y = block.m_y + (i / block.m_width);

            // Multiple rays for each pixel for Anti-Aliasing
            for (int j = 0; j < m_AARaysNumber; ++j)
            {
                // AA samples (1st and 2nd dimension)
                float rs1 = sobol::sample(base + i + j, 0) - 0.5f;
                float rs2 = sobol::sample(base + i + j, 1) - 0.5f;

                float xr = (float)pixel_x + m_GaussFilterWidth * rs1;
                float yr = (float)pixel_y + m_GaussFilterWidth * rs2;

                // checking for missing the image size
                if (xr < 0.f)
                {
                    xr = -xr;
                }
                if (xr > width)
                {
                    xr -= rs1;
                }

                if (yr < 0.f)
                {
                    yr = -yr;
                }
                if (yr > height)
                {
                    yr -= rs2;
                }

                float x = 2.f * xr / width - 1.f;
                float y = -2.f * yr / height + 1.f;

                Vec3f Ei = tracePath(x, y, ctx, base + i, R, dummyVisualization);

                color += filterGauss(2.f * rs1, 2.f * rs2) * Vec4f(Ei, 1.f);
            }

            color /= color.w;

            // Put pixel
            Vec4f prev = image->getVec4f(Vec2i(pixel_x, pixel_y));
            prev += color;
            image->setVec4f(Vec2i(pixel_x, pixel_y), prev);
        }
    }

    float PathTraceRenderer::filterGauss(float x, float y)
    {
        float sigma = FW::clamp(m_GaussFilterWidth / 3.f, 1.f, m_GaussFilterWidth);
        float s = 2.f * sigma * sigma;

        return 1.f / FW::sqrt(FW_PI * s) * FW::exp(-1.f * (x * x + y * y) / s);
    }

    void PathTraceRenderer::startPathTracingProcess(const MeshWithColors* scene,
                                                    const std::vector<AreaLight*>& areaLights,
                                                    RayTracer* rt, Image* dest, int bounces,
                                                    const CameraControls& camera,
                                                    const std::vector<RTTriangle*>& lightTriangles)
    {
        FW_ASSERT(!m_context.m_bForceExit);

        m_context.m_bForceExit = false;
        m_context.m_bResidual = false;
        m_context.m_camera = &camera;
        m_context.m_rt = rt;
        m_context.m_scene = scene;

        m_context.m_areaLights = areaLights;
        m_context.m_lightTriangles = lightTriangles;

        m_context.m_pass = 0;
        m_context.m_bounces = bounces;
        m_context.m_image.reset(new Image(dest->getSize(), ImageFormat::RGBA_Vec4f));

        m_context.m_destImage = dest;
        m_context.m_image->clear();

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
