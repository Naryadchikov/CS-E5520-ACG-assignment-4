#include "AreaLight.hpp"
#include "sobol.hpp"


namespace FW
{
    void AreaLight::draw(const Mat4f& worldToCamera, const Mat4f& projection)
    {
        glUseProgram(0);

        glMatrixMode(GL_PROJECTION);
        glLoadMatrixf((float*)&projection);
        glMatrixMode(GL_MODELVIEW);

        Mat4f S = Mat4f::scale(Vec3f(m_size, 1));
        Mat4f M = worldToCamera * m_xform * S;

        glLoadMatrixf((float*)&M);

        glBegin(GL_TRIANGLES);

        float maxComp = m_E.max();

        if (maxComp != 0.f)
        {
            glColor3f(m_E.x / maxComp, m_E.y / maxComp, m_E.z / maxComp);
        }
        else
        {
            glColor3f(0.f, 0.f, 0.f);
        }

        glVertex3f(1, 1, 0);
        glVertex3f(1, -1, 0);
        glVertex3f(-1, -1, 0);
        glVertex3f(1, 1, 0);
        glVertex3f(-1, -1, 0);
        glVertex3f(-1, 1, 0);

        glEnd();
    }

    void AreaLight::sample(float& pdf, Vec3f& p, int base, Random& R)
    {
        Vec4f rndPoint = Vec4f(R.getVec2f(), 0.f, 1.f);
        Mat4f S = Mat4f::scale(Vec3f(m_size, 1.f));

        p = (m_xform * S * rndPoint).getXYZ();
        pdf = 1.f / (4.f * m_size.x * m_size.y);
    }

    void AreaLight::sample(float& pdf, Vec3f& p, int base, int bounce, Random& R)
    {
        int rnd = R.getU32(1, 10000);

        // low discrepancy sampling with Sobol sequence
        float x = sobol::sample(base + rnd, bounce + 2);
        float y = sobol::sample(base + rnd, bounce + 3);

        Vec4f rndPoint = Vec4f(x, y, 0.f, 1.f);
        Mat4f S = Mat4f::scale(Vec3f(m_size, 1.f));

        p = (m_xform * S * rndPoint).getXYZ();
        pdf = 1.f / (4.f * m_size.x * m_size.y);
    }
} // namespace FW
